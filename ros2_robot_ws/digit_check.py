from digit_interface import Digit

def check_camera():
  d = Digit("D20992") # Unique serial number
  d.connect()
  d.show_view()
  d.disconnect()
  
  
if __name__ == "__main__":
  check_camera()