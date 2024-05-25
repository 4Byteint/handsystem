cmd_/home/pi/handsystem/linuxcan/usbcanII/modules.order := {   echo /home/pi/handsystem/linuxcan/usbcanII/usbcanII.ko; :; } | awk '!x[$$0]++' - > /home/pi/handsystem/linuxcan/usbcanII/modules.order
