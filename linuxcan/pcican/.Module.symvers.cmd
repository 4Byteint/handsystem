cmd_/home/pi/handsystem/linuxcan/pcican/Module.symvers := sed 's/\.ko$$/\.o/' /home/pi/handsystem/linuxcan/pcican/modules.order | scripts/mod/modpost -m -a  -o /home/pi/handsystem/linuxcan/pcican/Module.symvers -e -i Module.symvers -i /home/pi/handsystem/linuxcan/pcican/../common/Module.symvers   -T -
