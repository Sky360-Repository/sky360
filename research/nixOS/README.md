# sky 360 Cyclop

This is the fisheye robotic part for the `qhy183` camera for nixOS

## orange pi 5/5B/Plus
currently a manual step is needed to install uboot...this is currently at work https://github.com/ryan4yin/nixos-rk3588
- boot with ambian, image can be found here: https://www.armbian.com/orangepi-5/
- for Orange Pi 5 Plus /boot/armbianEnv.txt 
```
fdtfile=rockchip/rk3588-orangepi-5-plus.dtb
```
- for Orange Pi 5B edit /boot/armbianEnv.txt 
```
fdtfile=rockchip/rk3588s-orangepi-5.dtb
```

```shell-session
$ nix-shell -p wget zstd 

[nix-shell:~]$ wget cyclop-x86_64-linux.img.zst - TBD
[nix-shell:~]$ unzstd -d cyclop-x86_64-linux.img.zst
[nix-shell:~]$ dmesg --follow
```
## write SD Card to USB
```console
[nix-shell:~]$ sudo dd conv=fsync if=cyclop-{board}.img of=/dev/sdX status=progress
```

## For development

### Build 
```bash
make build
```
### run 
```bash
make run

### Debug Serial port 
make debug 