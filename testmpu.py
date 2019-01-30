from machine import I2C, Pin
import q4
import time,math
import mpu
import mpu6050
i2c = I2C(scl=Pin(5), sda=Pin(4))
acc = mpu6050.accel(i2c)
acc.error_gy()
#ay=acc.get_values()
#mpu.one_filter(ay["AcX"],ay["AcY"],ay["AcZ"],ay["GyX"],ay["GyY"],ay["GyZ"])
i=0
gyx=0
gyy=0
r=[]
q=[]
while 1:
	ay=acc.get_values()
	i=i+1
	if i==124:
		#print(r)
		print(q)
		i=0
		#print(ay["AcX"],ay["AcY"],ay["AcZ"],ay["GyX"],ay["GyY"],ay["GyZ"])
	#r=mpu.one_filter(ay["AcX"],ay["AcY"],ay["AcZ"],ay["GyX"],ay["GyY"],ay["GyZ"])
	q=q4.IMUupdate(ay["GyX"]/65.5*0.0174533,ay["GyY"]/65.5*0.0174533,ay["GyZ"]/65.5*0.0174533,ay["AcX"]/8192,ay["AcY"]/8192,ay["AcZ"]/8192)
	time.sleep(0.0035)
