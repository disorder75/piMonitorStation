#!/bin/bash
echo "starting kernel compiling..."
KERNEL=kernel && make ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- bcmrpi_defconfig -j 4 && make ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- zImage modules dtbs -j 4

echo "Build done. Now copy all the structure from mnt into the device or sd card"
