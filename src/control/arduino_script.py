import serial

thruster_number = 1
delay_value = 1480

portName = serial.Serial("/dev/cu.usbmodem1101")
command = f"{thruster_number} {delay_value}"
portName.write(command.encode()
