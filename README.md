# CC1200_HAT-fw

Basic firmware for the [CC1200 HAT RF board](https://github.com/M17-Project/CC1200_HAT-hw). Generated with CubeIDE 2.0.0.

### Flashing and Usage

Please consult the [M17 Wiki](https://wiki.m17foundation.org/index.php?title=M17_hotspot) for instructions on how to flash and use the CC1200 hat.

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
