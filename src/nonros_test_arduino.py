import serial

vals = ['p100','p010','n132','n001']

arduino = serial.Serial('/dev/ttyACM1', 9600, timeout=0.1)

while True:
    for i in vals:
        arduino.write(i)
        data = arduino.readline()
        if data:
            print data
