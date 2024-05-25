cmd_/home/pi/handsystem/linuxcan/pcican/modules.order := {   echo /home/pi/handsystem/linuxcan/pcican/kvpcican.ko; :; } | awk '!x[$$0]++' - > /home/pi/handsystem/linuxcan/pcican/modules.order
