# Install a signed firmware release

Download the signed H7C application package from [firmware releases](https://github.com/YaolinDong/infrabot-firmware/releases). The Python wheel and SDK source ship the public verification key. The same IFW contains applications for both slots and supports Ethernet and SWD maintenance on boards provisioned with the matching publisher's v2 loader.

## Download from a release

With network access, `--download` fetches the package from the firmware releases page before any board operation:

```bash
./flash-net --prepare --download          # latest release
./flash-net --prepare --download v0.1.1   # a specific release
./flash-net 192.168.0.50 --download v0.1.1
```

The package is saved under `firmware/cache/<version>/` and its signature is checked against `firmware/release.pub` before use. A cached version is verified again and not downloaded twice. `latest` always asks the server, so pin a version for repeatable updates. Downloads honor `https_proxy`. Use `--url` for a mirror with the same `download/<version>/infrabot_firmware.ifw` layout. `--download` and `--package` cannot be combined, and `flash-swd` accepts the same options.

## Check the package and board

```bash
./flash-net --prepare --package /path/to/infrabot_firmware.ifw
./flash-net --probe 192.168.0.50
```

The first command verifies the package locally. The probe asks the current board for its confirmed slot and image identifier. To use the default file name, save the release as `firmware/infrabot_firmware.ifw`. The previous `fdcantest.ifw` name is accepted for one SDK release when the new name is absent.

## Update a provisioned board

Support the arm, explicitly disable the drives and disconnect motor power. Keep the controller powered. For Ethernet, connect on the configured subnet, check the requested image ID on the touchscreen and choose Allow within the approval window.

```bash
./flash-net 192.168.0.50 --package /path/to/infrabot_firmware.ifw
./flash-net --probe 192.168.0.50
```

Wait for `Confirmed application is running.` and compare the probed image ID with the selected release. If the board already runs an image from the package, the tool prints `The board already runs this firmware; nothing was sent.` and asks for no approval. The board verifies the signed image, writes the inactive slot and trial-boots it. A failed, unconfirmed trial returns to the previous confirmed application. To update with a CMSIS-DAP/SWD probe, use `./flash-swd --package /path/to/infrabot_firmware.ifw`. SWD makes a local backup before programming.

For an empty board, a legacy loader, or a change of publisher key, follow the firmware project's [factory provisioning and recovery instructions](https://github.com/YaolinDong/infrabot-firmware/blob/main/docs/flashing.md) with the matching factory image. Keep the verified backup private.

## Publish a firmware release

Attach the signed package to the firmware tag's GitHub Release as `infrabot_firmware.ifw`. Keep that exact asset name for every release so `--download` and the `latest` link resolve.

## Distribute a Linux update kit

```bash
python3 -B tools/update/package_update.py --package /path/to/infrabot_firmware.ifw
```

The resulting architecture-specific archive under `build/releases/` includes the updater, signed IFW, public key, checksums and operator instructions. The kit supports offline verification before connecting to a board. [Standalone kit guide](update-kit.md)
