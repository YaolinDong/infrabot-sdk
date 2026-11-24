/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#include "infrabot/firmware_update.hpp"
#include <arpa/inet.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>
#include <openssl/crypto.h>
#include <openssl/evp.h>
#include <openssl/sha.h>
#include <cstdio>
#include <openssl/rand.h>
#include <algorithm>
#include <array>
#include <chrono>
#include <cstring>
#include <thread>
#include <fstream>
#include <stdexcept>
#include <vector>

namespace infrabot {
namespace {
using Bytes=std::vector<unsigned char>;
using Clock=std::chrono::steady_clock;
constexpr uint32_t base=0x08020400, bank_b=0x08100400, capacity=0xdfc00;
enum { hello=1, enter, begin, data, commit, confirm, revert };
struct Socket { int fd=-1; ~Socket(){if(fd>=0)close(fd);} };
void require(bool v,const char *s){if(!v)throw std::runtime_error(s);}
uint64_t get(const unsigned char *p,unsigned n){uint64_t v=0;while(n)v=v*256+p[--n];return v;}
void put(Bytes &b,uint64_t v,unsigned n){while(n--){b.push_back(v&255);v>>=8;}}
Bytes readfile(const std::string &name,size_t max){
    std::ifstream f(name,std::ios::binary|std::ios::ate);
    require(bool(f),"cannot open firmware package/key");
    auto n=f.tellg();require(n>=0 && size_t(n)<=max,"file size outside limit");
    Bytes b(static_cast<size_t>(n));f.seekg(0);require(bool(f.read(reinterpret_cast<char*>(b.data()),n)),"short file");return b;
}
Bytes readkey(const std::string &path){
    static constexpr char trusted[]=INFRABOT_RELEASE_PUBLIC;
    auto b=path.empty()?Bytes(trusted,trusted+sizeof(trusted)-1):readfile(path,128);while(!b.empty() && (b.back()=='\n'||b.back()=='\r'))b.pop_back();
    require(b.size()==64,"expected Ed25519 public key (64 hex digits)");Bytes k;
    auto nib=[](int c)->unsigned{if(c>='0'&&c<='9')return c-'0';
        if(c>='a'&&c<='f')return c-'a'+10;
        if(c>='A'&&c<='F')return c-'A'+10;
        throw std::runtime_error("invalid key encoding");};
    for(unsigned i=0;i<64;i+=2)k.push_back(nib(b[i])*16+nib(b[i+1]));
    OPENSSL_cleanse(b.data(),b.size());return k;
}
/* Firmware 0.1.1+ keeps "IBFWVER1<version>\0" in its image (app_info.c).
 * The image is already signature-checked when this runs.  Older images have
 * no record: no version is sent and the screen shows "?". */
std::string image_version(const unsigned char *image,uint32_t size){
    static const char magic[]="IBFWVER1";
    const unsigned char *begin=image+64,*end=image+size-64;
    auto at=std::search(begin,end,magic,magic+8);
    if(at==end)return {};
    std::string v;
    for(auto c=at+8;c<end && *c && v.size()<23;++c){
        if(*c<0x20 || *c>=0x7f)return {};
        v.push_back(char(*c));
    }
    return v;
}
bool signed_image(const Bytes &pub,const unsigned char *p,uint32_t n){
    if(n<72)return false;
    unsigned char digest[32];SHA256(p+64,n-64,digest);
    if(CRYPTO_memcmp(digest,p+32,32))return false;
    EVP_PKEY *pk=EVP_PKEY_new_raw_public_key(EVP_PKEY_ED25519,nullptr,pub.data(),pub.size());
    EVP_MD_CTX *ctx=EVP_MD_CTX_new();
    bool ok=pk && ctx && EVP_DigestVerifyInit(ctx,nullptr,nullptr,nullptr,pk)==1 &&
        EVP_DigestVerify(ctx,p+64+n-64,64,p,64)==1;
    EVP_MD_CTX_free(ctx);EVP_PKEY_free(pk);return ok;
}
struct Answer {unsigned role,status;uint64_t session;uint32_t offset;Bytes payload;};
struct Client {
    Socket socket;uint32_t seq=0;uint64_t session=0;
    Client(const std::string &host,uint16_t port){
        sockaddr_in a{};a.sin_family=AF_INET;a.sin_port=htons(port);
        require(inet_pton(AF_INET,host.c_str(),&a.sin_addr)==1,"expected board IPv4 address");
        socket.fd=::socket(AF_INET,SOCK_DGRAM,0);
        require(socket.fd>=0 && connect(socket.fd,(sockaddr*)&a,sizeof(a))==0,"cannot open update socket");
        require(RAND_bytes(reinterpret_cast<unsigned char*>(&seq),sizeof(seq))==1,"random sequence failed");
    }
    bool exchange(unsigned op,uint32_t offset,const unsigned char *payload,size_t len,Answer &out,int tries,int ms){
        Bytes b={'I','F','U','2',2,(unsigned char)op,0,0};put(b,session,8);
        put(b,++seq,4);put(b,offset,4);if(len)b.insert(b.end(),payload,payload+len);

        for(int i=0;i<tries;i++){
            auto until=Clock::now()+std::chrono::milliseconds(ms);
            if(send(socket.fd,b.data(),b.size(),0)<0){std::this_thread::sleep_until(until);continue;}
            while(Clock::now()<until){
                pollfd p{socket.fd,POLLIN,0};
                int wait=int(std::chrono::duration_cast<std::chrono::milliseconds>(until-Clock::now()).count())+1;
                if(poll(&p,1,wait)<=0)break;
                unsigned char buf[128];auto n=recv(socket.fd,buf,sizeof(buf),0);
                if(n<0)break;
                if(n<24 || memcmp(buf,"IFU2",4)||buf[4]!=2||buf[5]!=(op|128)||get(buf+16,4)!=seq)continue;
                if(op!=hello && get(buf+8,8)!=session)continue;
                out={buf[7],buf[6],get(buf+8,8),uint32_t(get(buf+20,4)),Bytes(buf+24,buf+n)};
                return true;
            }
            std::this_thread::sleep_until(until);
        }
        return false;
    }
    void accepted(const Answer &r){
        static const char *const msg[]={"ok","busy: drives, setup, persistence or other updater active",
            "invalid command/layout","expired session","wrong chunk offset","image authentication failed","flash verification failed","hardware RNG not ready","waiting for board screen","denied on board or no valid local permission","screen approval expired"};
        if(r.status)throw std::runtime_error(r.status<11?msg[r.status]:"unknown loader error");
    }
    Answer discover(int attempts,bool probe=false){
        Answer a{};require(exchange(hello,probe?1:0,nullptr,0,a,attempts,500),
            "no IFU2 reply (check IP/link; a v1 loader needs one-time SWD migration)");
        accepted(a);require((a.session || probe) && a.role>=1 && a.role<=3,"invalid board response");
        if(!probe)session=a.session;
        return a;
    }
    void permission(unsigned op,const unsigned char *tag,uint32_t initial_base,const std::string &version={}){
        if(tag)std::fprintf(stderr,"请核对屏幕上的镜像标识：%02x%02x%02x%02x\n",tag[0],tag[1],tag[2],tag[3]);
        if(tag && !version.empty())std::fprintf(stderr,"新固件版本：%s\n",version.c_str());
        Bytes request;
        if(tag){request.assign(tag,tag+32);request.insert(request.end(),version.begin(),version.end());}
        std::fprintf(stderr,"请在控制板屏幕上选择允许或拒绝（60 秒）。\n");
        auto end=Clock::now()+std::chrono::seconds(65);
        while(Clock::now()<end){
            Answer a{};
            if(exchange(op,0,request.data(),request.size(),a,2,250)){
                // Firmware 0.1.0 accepts only the 32-byte tag: retry without the version.
                if(a.status==2 && request.size()>32){request.resize(32);continue;}
                if(a.status==3 && ((op==enter && a.role==2)||(op==revert && a.role==1)))return;
                if(a.status!=8){accepted(a);return;}
            }else{
                // A permitted reset intentionally drops the pending request.
                // Probe without replacing the ticket needed after reboot.
                if(exchange(hello,1,nullptr,0,a,1,200) &&
                   ((op==enter && a.role==2) || (op==revert && a.role==1 && a.offset!=initial_base)))return;

            }
            std::this_thread::sleep_for(std::chrono::milliseconds(250));
        }
        throw std::runtime_error("screen approval timed out");
    }
    void command(unsigned op,uint32_t offset,const unsigned char *p,size_t n,uint32_t ack){
        Answer a{};require(exchange(op,offset,p,n,a,12,250),"transfer timeout; rerun updater");
        accepted(a);require(a.role==2 && a.offset==ack,"unexpected loader offset");
    }
};
}
std::string firmware_status(const std::string &ip,const std::string &key,uint16_t port){
    (void)key;Client c(ip,port);auto a=c.discover(4,true);
    if(a.role==2)return "recovery-loader";
    require(a.payload.size()==32,"missing installed image ID");
    std::string result=a.role==3?"trial ":"confirmed ";
    result+=(a.offset==base?"A ":"B ");const char hex[]="0123456789abcdef";
    for(auto byte:a.payload){result+=hex[byte>>4];result+=hex[byte&15];}
    return result;
}
bool update_firmware(const std::string &ip,const std::string &path,const std::string &key,
                     std::function<void(unsigned,unsigned)> progress,uint16_t port){
    Bytes bundle=readfile(path,16+2*(capacity+64));
    require(bundle.size()>=16 && !memcmp(bundle.data(),"IFABv002",8),"expected signed v2 A/B firmware bundle");
    unsigned na=get(bundle.data()+8,4),nb=get(bundle.data()+12,4);
    require(na>=72 && nb>=72 && na<=capacity+64 && nb<=capacity+64 &&
            bundle.size()==16ull+na+nb,"invalid A/B package sizes");
    Client c(ip,port);
    const unsigned char *images[]={bundle.data()+16,bundle.data()+16+na};
    Bytes pub=readkey(key);
    uint32_t sizes[]={na-64,nb-64},bases[]={base,bank_b};
    for(unsigned i=0;i<2;i++){
        auto p=images[i];require(sizes[i]>=72,"short signed image");uint32_t pc=get(p+68,4);
        require(!memcmp(p,"IFH743v2",8) && get(p+8,4)==bases[i] && get(p+12,4)==sizes[i] &&
                get(p+64,4)==0x20020000 && (pc&1) && (pc&~1U)>=bases[i] && (pc&~1U)-bases[i]<sizes[i]-64,
                "incorrect H743 slot, image length or vectors");
        require(signed_image(pub,p,sizes[i]),"firmware signature or image digest invalid");
    }
    require(!memcmp(images[0]+16,images[1]+16,16),"mixed A/B releases");
    auto same=[&](const Answer &a,unsigned i){return a.offset==bases[i] && a.payload.size()==32 &&
        !CRYPTO_memcmp(images[i]+32,a.payload.data(),32);};
    Answer a=c.discover(8);
    if(a.role==1){
        if(same(a,0)||same(a,1))return false;
        const unsigned next=a.offset==base?1:0;
        c.permission(enter,images[next]+32,a.offset,image_version(images[next],sizes[next]));
        a=c.discover(60);
    }
    require(a.role==2 && (a.offset==base || a.offset==bank_b),"loader not ready; old firmware retained");
    const unsigned dest=a.offset==bank_b;
    const unsigned char *image=images[dest];uint32_t length=sizes[dest];
    c.command(begin,0,image,64,0);
    for(uint32_t off=0;off<length;){
        auto n=std::min(1024U,length-off);
        c.command(data,off,image+64+off,n,off+n);off+=n;if(progress)progress(off,length);
    }
    if(c.exchange(commit,length,nullptr,0,a,3,1000) && a.role==2)c.accepted(a);
    auto deadline=Clock::now()+std::chrono::seconds(90);
    bool confirmation_sent=false;
    while(Clock::now()<deadline){
        if(c.exchange(hello,0,nullptr,0,a,1,500) && !a.status){
            if(a.role==1){
                if(same(a,dest))return true;
                throw std::runtime_error("previous confirmed application is running; update rolled back");
            }
            if(a.role==3 && same(a,dest)){
                c.session=a.session;
                confirmation_sent=true;
                if(c.exchange(confirm,0,nullptr,0,a,1,300) && a.role==3 && a.status!=1)c.accepted(a);
            }
        }
        usleep(200000);
    }
    throw std::runtime_error(confirmation_sent?
      "confirmation result unknown: probe again; previous slot retained":
      "no trial confirmed; trial watchdog/reset selects previous firmware");
}

void rollback_firmware(const std::string &ip,const std::string &key,uint16_t port){
    (void)key;Client c(ip,port);Answer a=c.discover(6);
    require(a.role==1,"rollback must be requested at the normal application screen");
    uint32_t previous=a.offset==base?bank_b:base;
    c.permission(revert,nullptr,a.offset);
    a=c.discover(60,true);
    require(a.role==1 && a.offset==previous,"previous firmware not selected; probe before retry");
}
}
