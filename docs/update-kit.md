# InfraBot firmware update kit / InfraBot 固件升级工具

Use this kit on a Linux host matching the kit's architecture, with compatible glibc/libstdc++, Python 3 and OpenSSL 3 (CLI and libcrypto). SWD also requires OpenOCD with CMSIS-DAP and STM32H7 support. Extract the complete kit. Keep the signed IFW and the publisher's public key under `firmware/`.

适用于已安装 v2 签名 A/B Loader 的 STM32H743 控制板。请完整解压工具包，在与打包机架构和运行库兼容的 Linux 上运行。需要 Python 3 和 OpenSSL 3；SWD 还需要带 CMSIS-DAP 和 STM32H7 配置的 OpenOCD。包内的发布者公钥须与板上 Loader 匹配。

## Before flashing / 升级前

Support the arm, explicitly disable its drives, disconnect motor power and keep the controller powered. 支撑好机械臂，明确失能各驱动，断开电机动力，保留控制板供电。

Run these checks from the extracted directory. 在解压目录中检查文件完整性、签名和当前槽位：

```sh
sha256sum -c SHA256SUMS
./flash-net --prepare
./flash-net --probe
```

## Install and confirm / 烧录与确认

Choose one transport. 二选一：

```sh
./flash-net 192.168.0.50
# Alternatively, use a connected SWD probe / 或使用已连接的 SWD 探针
./flash-swd
```

Press Enter or type `yes` to continue, or `no` to cancel. For Ethernet, match the image ID on the touchscreen and select Allow within 60 seconds. Wait for `Confirmed application is running.`, then query the running slot and image ID again. Reinstalling the same image can keep the current slot.

回车或输入 `yes` 继续，输入 `no` 取消。网口升级时核对屏幕镜像标识，60 秒内点允许；拒绝或超时会取消写入。传输到 100% 后仍需等待运行确认。相同镜像可以保留当前槽位。

```sh
./flash-net --probe 192.168.0.50
```

To request the previous confirmed application, run the rollback command and approve it on the board. 如需回退到之前已确认的应用，执行以下命令并在板上确认：

```sh
bin/infrabot_update --rollback 192.168.0.50 firmware/release.pub
bin/infrabot_update --probe 192.168.0.50 firmware/release.pub
```

Routine updates write the application. Blank boards, legacy systems and loader replacement require separate factory SWD provisioning or service. Keep SWD backups private (default `~/infrabot-flash-backups`).

常规升级只写应用。空板、旧单镜像或 v1 系统的首次安装、迁移及 Loader 更换须通过配套的厂内 SWD 流程完成。请私下保存 SWD 备份。
