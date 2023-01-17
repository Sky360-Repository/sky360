build::
	docker build . -t sky360/cyclop:develop 
run:
	docker run \
		--privileged \
		-p 7624:7624 \
		-v /run/udev:/run/udev \
		-it \
		sky360/cyclop:develop
patch:
	docker run \
		--net=host \
		-p 7624:7624 \
		-v /bin/systemctl:/bin/systemctl \
		-v /etc:/etc \
		-v /lib:/lib \
		-v /run/udev:/run/udev \
		-v /sys/devices:/sys/devices \
		-v /usr:/usr \
		-it \
		sky360/cyclop:develop \
		/bin/bash ./install_qhy_sdk_and_udev_rules_to_host.sh
devel:
	docker run \
		--privileged \
		-p 7624:7624 \
		-v /run/udev:/run/udev \
		-it \
		sky360/cyclop:develop \
		/bin/bash
