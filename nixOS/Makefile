

build-opi5p: # orange-pi-5-plus
	nix build --option system aarch64 .#cyclop-orange_pi_5_plus-sd_image
build-opi5: # orange-pi-5-plus
	nix build --option system aarch64 .#cyclop-orange_pi_5-sd_image

unpack-opi5p:
	unzstd result/sd-image/sky360-cyclop-orange_pi_5-plus.img.zst -o cyclop.img
unpack-opi5:
	unzstd result/sd-image/sky360-cyclop-orange_pi_5.img.zst -o cyclop.img
dd:
	dd bs=8m if=./cyclop.img of=/dev/sda status=progress
	blockdev --flushbufs /dev/sda

debug:
	screen /dev/ttyUSB0 1500000
