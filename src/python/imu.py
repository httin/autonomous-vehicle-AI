import serial
import time
 
ser_imu = serial.Serial('/dev/ttyUSB2', 
                    baudrate = 921600, 
                    parity = serial.PARITY_NONE,    
                    stopbits = serial.STOPBITS_ONE, 
                    bytesize = serial.EIGHTBITS)
ser_stm = serial.Serial('/dev/ttyUSB1', 
                    baudrate = 115200, 
                    parity = serial.PARITY_NONE,    
                    stopbits = serial.STOPBITS_ONE, 
                    bytesize = serial.EIGHTBITS)

# data_config = ser_stm.readline()
# ser_imu.write(data_config)

while True:
    data_imu = ser_imu.readline()      
    data_imu = '\n' + data_imu
    #print data_imu
    ser_stm.write(data_imu)    

ser.close()
