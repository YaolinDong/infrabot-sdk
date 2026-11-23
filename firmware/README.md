# Firmware updates

`release.pub` is the public Ed25519 key used to verify official updates. Download the signed IFW application package for your controller from the firmware release page. Pass the file to either launcher, or place it here as `infrabot_firmware.ifw` to use the default path. [Firmware releases](https://github.com/YaolinDong/infrabot-firmware/releases)

```sh
./flash-net --prepare --package /path/to/infrabot_firmware.ifw
./flash-net --prepare --download v0.1.1   # or fetch it from the release page
```

`--prepare` checks the signature without contacting the board. Downloaded packages are kept in `cache/<version>/`, which is not tracked. Firmware binaries are distributed separately from the source and Python wheel.

The previous basename `fdcantest.ifw` remains a fallback for one SDK version. When both files are present the tools verify and use `infrabot_firmware.ifw`.
