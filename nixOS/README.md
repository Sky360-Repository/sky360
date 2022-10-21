# sky 360 Cyclop

This is the fisheye robotic part for the `qhy183` camera for nixOS

## write Image to SD-Card

```shell-session
$ nix-shell -p wget zstd

[nix-shell:~]$ wget cyclop-x86_64-linux.img.zst - TBD
[nix-shell:~]$ unzstd -d cyclop-x86_64-linux.img.zst
[nix-shell:~]$ dmesg --follow
```
## write SD Card to USB
```console
[nix-shell:~]$ sudo dd if=cyclop-aarch64-linux.img of=/dev/sdX bs=4096 conv=fsync status=progress
```

## For development

### build
```bash
make build
```
### run 
```bash
make run
