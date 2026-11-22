# Firmware updates

`release.pub` is the public Ed25519 key used to verify official updates. Download the signed IFW application package for your controller from the firmware release page. Pass the file to either launcher, or place it here as `infrabot_firmware.ifw` to use the default path. [Firmware releases](https://github.com/YaolinDong/infrabot-firmware/releases)

```sh
./flash-net --prepare --package /path/to/infrabot_firmware.ifw
```

This command checks the signature offline. Firmware binaries are distributed separately from the source and Python wheel.

The previous basename `fdcantest.ifw` remains a fallback for one SDK version. When both files are present the tools verify and use `infrabot_firmware.ifw`.
