#!/bin/bash
case $1 in
	list)
		lsusb
	;;
	find_port)	
		ZeroBUS=$(lsusb | grep $2 | cut -c  5-7 )
		# Strip leading zeros
		BUS=$(echo $ZeroBUS | sed 's/^0*//')

		if [ -z "$BUS" ]; then
			    echo "port not found"
			    exit 1
		fi

		# Build "usbX" usb number
		USB=usb$BUS
		echo "port found:" $USB $2
		;;
	off)
		echo "Powering off:" $2
		echo $2 | sudo tee /sys/bus/usb/drivers/usb/unbind &>/dev/null &
		;;
	on)
		echo "Powering on:" $2
		echo $2 | sudo tee /sys/bus/usb/drivers/usb/bind &>/dev/null &
		;;

	reset)
		echo "Resetting:" $2
		$0 off $2 &>/dev/null &
		sleep 1
		$0 on $2 &>/dev/null &
		;;
esac
