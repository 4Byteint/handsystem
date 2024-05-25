cmd_/home/pi/handsystem/linuxcan/common/modules.order := {   echo /home/pi/handsystem/linuxcan/common/kvcommon.ko; :; } | awk '!x[$$0]++' - > /home/pi/handsystem/linuxcan/common/modules.order
