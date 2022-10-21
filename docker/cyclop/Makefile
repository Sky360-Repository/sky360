build::
	docker build . -t sky360/cyclop:develop 
run:
	docker run \
		--privileged \
		-v /run/udev:/run/udev \
		-it \
		sky360/cyclop:develop
patch:
	docker run \
		--net=host \
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
		-v /run/udev:/run/udev \
		-it \
		sky360/cyclop:develop \
		/bin/bash
