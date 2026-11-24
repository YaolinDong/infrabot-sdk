/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#include "infrabot/firmware_update.hpp"
#include <cstdio>
#include <exception>
int main(int argc,char **argv){
  if(argc!=4 && argc!=5){std::fprintf(stderr,"Usage: %s BOARD_IP APPLICATION.ifw PUBLIC_KEY_FILE [PORT] (or: --probe/--rollback BOARD_IP PUBLIC_KEY_FILE [PORT])\n",argv[0]);return 2;}
  try {
    int port=argc==5?std::stoi(argv[4]):5002;
    if(port<1||port>65535)return 2;
    if(std::string(argv[1])=="--rollback"){infrabot::rollback_firmware(argv[2],argv[3],port);std::puts("Confirmed previous firmware.");return 0;}
    if(std::string(argv[1])=="--probe"){std::puts(infrabot::firmware_status(argv[2],argv[3],port).c_str());return 0;}
    int old=-1;
    if(!infrabot::update_firmware(argv[1],argv[2],argv[3],[&](unsigned n,unsigned total){
      int pc=int(100ull*n/total);if(pc!=old){old=pc;std::fprintf(stderr,"\rTransfer %d%%",pc);}
    },port)){std::puts("The board already runs this firmware; nothing was sent.");return 0;}
    std::puts("\nConfirmed application is running.");return 0;
  } catch(const std::exception &e){std::fprintf(stderr,"\nUpdate failed: %s\n",e.what());return 1;}
}
