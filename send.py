from canlib import canlib, Frame
import time
import sys
import select

ch = canlib.openChannel(
	channel=0,
	flags=canlib.Open.EXCLUSIVE,
	bitrate=canlib.canBITRATE_250K
)

# Set the CAN bus driver type to NORMAL.
ch.setBusOutputControl(canlib.Driver.NORMAL)

# Activate the CAN chip.
ch.busOn()
 
# Transmit a message with (11-bit) CAN id , length 8 and content
ask_UBoot = Frame(id_=1, data=[0,1,1,1,0,0,0,0] , dlc=8)
ask_STMBoot = Frame(id_=1, data=[0,0,0,0,0,0,0,2] , dlc=8)
# set filter (id = 2, 3 can pass, 4 can't)
ch.canAccept(2,canlib.AcceptFilterFlag.SET_CODE_STD) # code = 0010 
ch.canAccept(6,canlib.AcceptFilterFlag.SET_MASK_STD) # mask = 0110

while True:
	ready, _, _ = select.select([sys.stdin],[],[],0.1)
	if ready:
		user_input = input()
		if user_input == 'a':
			ch.write(ask_UBoot)
      #ch.write(ask_STMBoot)
			time.sleep(0.1)
	else:
		try:
  			msg = ch.read()
			print(msg)
        	if msg.id=3 and msg.data=[0,2,3,1,0,0,0,1]:
			print("goto init_status")
			time.sleep(0.1) 
		except canlib.CanNoMsg: #it will stuck at ch.read() if not add this 
			continue
		except KeyboardInterrupt:
			pass

ch.busOff()
ch.close()
