# sky 360 Cyclop

This is the fisheye robotic part docker images for the `qhy183` camera. this docker images support amd64 and arm64 cpu architecture.

## requirements
- host ubuntu server + docker  (tested)

### QHY183 Camera requirement 

in order to use the QHY183 camera, the physical host requries to be patched. The docker image provides this patch. 
The patch does the following on the physical host:
- Install QHY_CCD SDK
- Reload udev rules
- Trigger udev `add` event for all QHY devices. (in case the camera is connected already - this will cause the firmware to be flushed to the camera. 

To run the patch please use the following command:
```bash
docker run \
	--net=host \
	-v /bin/systemctl:/bin/systemctl \
	-v /etc:/etc \
	-v /lib:/lib \
	-v /run/udev:/run/udev \
	-v /sys/devices:/sys/devices \
	-v /usr:/usr \
	-it sky360/cyclop:latest \
	/bin/bash ./install_qhy_sdk_and_udev_rules_to_host.sh
```
## run 
This will run the indi_server with the indi_qhy_ccd driver
```bash
docker run --privileged -it sky360/cyclop:latest 
```

## For development

### build
```bash
make build
```
### run 
```bash
make run
```


