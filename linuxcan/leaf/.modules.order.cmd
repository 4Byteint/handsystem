cmd_/home/pi/handsystem/linuxcan/leaf/modules.order := {   echo /home/pi/handsystem/linuxcan/leaf/leaf.ko; :; } | awk '!x[$$0]++' - > /home/pi/handsystem/linuxcan/leaf/modules.order
