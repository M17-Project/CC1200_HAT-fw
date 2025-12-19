# CC1200_HAT-fw
Basic firmware for the [CC1200 HAT RF board](https://github.com/M17-Project/CC1200_HAT-hw). Generated with CubeMX 6.16.1.

### Firmware function
This firmware turns the CC1200 HAT into an RF modem for M17.

Intended host: Raspberry Pi running [m17-gateway](https://github.com/jancona/m17) or [rpi-interface](https://github.com/M17-Project/rpi-interface).

### Flashing and usage
Please consult the [M17 Wiki](https://wiki.m17foundation.org/index.php?title=M17_hotspot) for instructions on how to flash and use the CC1200 shield.

## Building on Linux
### Install Prerequisites
```
sudo apt-get install -y gcc-arm-none-eabi binutils-arm-none-eabi
```

### Build
```
cd Release
make -j$(nproc) all
arm-none-eabi-objcopy -O binary CC1200_HAT-fw.elf CC1200_HAT-fw.bin
```
