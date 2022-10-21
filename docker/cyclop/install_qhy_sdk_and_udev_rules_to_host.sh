case $(dpkg --print-architecture) in
amd64)
	echo "installing qhy sdk for amd64"
	cd /home/sdk_linux64_22.10.14/
	./install.sh ;;
arm64)
	echo "installing qhy sdk for arm64" 
	cd /home/sdk_Arm64_22.10.14/ 
	./install.sh ;;
*) 	
	echo "installing qhy sdk is not supported for this cpu architecture" ;;
esac

udevadm control --reload-rules || echo "done"
udevadm trigger --action=add --attr-match=idVendor=1618
