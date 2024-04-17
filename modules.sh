export ARCH=arm64
export CROSS_COMPILE=~/gcc/10.2/bin/aarch64-none-linux-gnu-
rm -rf ../mod
make -j16 modules KERNELRELEASE=6.6.9-qcomlt-arm64-00188-g31f49428dc7d
make modules_install KERNELRELEASE=6.6.9-qcomlt-arm64-00188-g31f49428dc7d INSTALL_MOD_STRIP=1 INSTALL_MOD_PATH=../mod
