#
export ARCH=arm64
export CROSS_COMPILE=~/gcc/10.2/bin/aarch64-none-linux-gnu-
make -j16 Image.gz dtbs KERNELRELEASE=6.6.9-qcomlt-arm64-00188-g31f49428dc7d
cat arch/arm64/boot/Image.gz arch/arm64/boot/dts/qcom/qrb5165-rb5.dtb > Image.gz+dtb
mkbootimg --kernel Image.gz+dtb \
              --ramdisk initrd.img \
              --output boot-rb5.img \
              --pagesize 4096 \
              --base 0x80000000 \
              --cmdline "root=PARTLABEL=rootfs console=tty0 console=ttyMSM0,115200n8 pcie_pme=nomsi"
