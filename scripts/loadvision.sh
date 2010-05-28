#!/bin/sh
echo "> Load modules"
KERNEL_RELEASE=`uname -r`
DRIVERS_PATH=/lib/modules/${KERNEL_RELEASE}/kernel/drivers
/sbin/insmod ${DRIVERS_PATH}/ieee1394/ieee1394.ko
/sbin/insmod ${DRIVERS_PATH}/ieee1394/ohci1394.ko
/sbin/insmod ${DRIVERS_PATH}/ieee1394/raw1394.ko
/sbin/insmod ${DRIVERS_PATH}/ieee1394/video1394.ko
echo "> Increase R/W privileges"
sudo /bin/chmod 777 /dev/raw1394
sudo /bin/chmod -R 777 /dev/video1394
