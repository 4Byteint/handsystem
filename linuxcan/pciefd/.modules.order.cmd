cmd_/home/pi/handsystem/linuxcan/pciefd/modules.order := {   echo /home/pi/handsystem/linuxcan/pciefd/kvpciefd.ko; :; } | awk '!x[$$0]++' - > /home/pi/handsystem/linuxcan/pciefd/modules.order
