lsusb -d 1618:c183 | while read _ bus _ device _; do
    fxload -v -t fx3 -I /lib/firmware/qhy/QHY183.img -D /dev/bus/usb/${bus}/${device%:}
done
