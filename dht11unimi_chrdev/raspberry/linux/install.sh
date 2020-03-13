#!/bin/bash

KERNEL=kernel

# Mount the SD (check with lsblk your mounting point)
echo "creating destination dir on filesystem"
mkdir mnt && mkdir mnt/fat32 && mkdir mnt/fat32/overlays/ && mkdir mnt/ext4

echo "mounting SD on filesystem"
sudo mount /dev/sdb1 mnt/fat32 && sudo mount /dev/sdb2 mnt/ext4

# Install modules
echo "building and installing modules on mounted sd"
sudo env PATH=$PATH make ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- INSTALL_MOD_PATH=mnt/ext4 modules_install

echo "backup previously kernel image"
sudo cp mnt/fat32/$KERNEL.img mnt/fat32/$KERNEL-backup.img
echo "coping kernel from zImage"
sudo cp arch/arm/boot/zImage mnt/fat32/$KERNEL.img
echo "copying device tree and overlays"
sudo cp arch/arm/boot/dts/*.dtb mnt/fat32/ && sudo cp arch/arm/boot/dts/overlays/*.dtb* mnt/fat32/overlays/ && sudo cp arch/arm/boot/dts/overlays/README mnt/fat32/overlays/

echo "installation done, unmounting SD "
sudo umount mnt/fat32 && sudo umount mnt/ext4
echo "done"

