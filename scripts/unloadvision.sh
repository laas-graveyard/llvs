#!/bin/bash
echo "> Unload modules"
/sbin/rmmod sbp2
sudo /sbin/rmmod video1394
sudo /sbin/rmmod raw1394
sudo /sbin/rmmod ohci1394
sudo /sbin/rmmod ieee1394
