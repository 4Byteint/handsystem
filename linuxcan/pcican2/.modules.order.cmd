cmd_/home/pi/handsystem/linuxcan/pcican2/modules.order := {   echo /home/pi/handsystem/linuxcan/pcican2/kvpcicanII.ko; :; } | awk '!x[$$0]++' - > /home/pi/handsystem/linuxcan/pcican2/modules.order
