cmd_/home/pi/handsystem/linuxcan/pcican2/Module.symvers := sed 's/\.ko$$/\.o/' /home/pi/handsystem/linuxcan/pcican2/modules.order | scripts/mod/modpost -m -a  -o /home/pi/handsystem/linuxcan/pcican2/Module.symvers -e -i Module.symvers -i /home/pi/handsystem/linuxcan/pcican2/../common/Module.symvers   -T -