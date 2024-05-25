cmd_/home/pi/handsystem/linuxcan/mhydra/modules.order := {   echo /home/pi/handsystem/linuxcan/mhydra/mhydra.ko; :; } | awk '!x[$$0]++' - > /home/pi/handsystem/linuxcan/mhydra/modules.order
