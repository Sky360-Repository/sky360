# sky360
Observational Citizen Science of Earths atmosphere and beyond.

Look at our github [issues](https://github.com/Sky360-Repository/sky360/issues).
Feel free to suggest ideas and help vote on some. 

# Installation (work in progress)

## Sentinal
Currently the development is done via x86-64

for nvidia boards (tbd) -https://github.com/aduril/jetpack-nixos

### write Image to SD-Card

```shell-session
$ nix-shell -p wget zstd

#[nix-shell:~]$ wget sentinal-x86_64-linux.img.zst - TBD
[nix-shell:~]$ unzstd -d sentinal-x86_64-linux.img.zst
[nix-shell:~]$ dmesg --follow
```
```console
[nix-shell:~]$ sudo dd if=sentinal-aarch64-linux.img of=/dev/sdX bs=4096 conv=fsync status=progress
```

## Development team and repositories

## FAQ

Read [here](./FAQ.md)

