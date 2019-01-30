import machine
from time import sleep,sleep_ms
class accel():
    error=[0,0,0]
    def __init__(self, i2c, addr=0x68):
        self.iic = i2c
        self.addr = addr
        self.iic.start()
        sleep_ms(1)
        self.iic.writeto(self.addr, bytearray([107, 0]))
        sleep_ms(1)
        self.iic.writeto_mem(self.addr,0x19,b'\x07') #gyro 125hz
        sleep_ms(1)
        self.iic.writeto_mem(self.addr,0x1a,b'\x04')  #low filter 21hz
        sleep_ms(1)
        self.iic.writeto_mem(self.addr,0x1b,b'\x08') #gryo 500/s 65.5lsb/g
        sleep_ms(1)
        self.iic.writeto_mem(self.addr,0x1c,b'\x08') #acceler 4g ,8192lsb.g
        sleep_ms(1)
        self.iic.stop()

    def get_raw_values(self):
        self.iic.start()
        a = self.iic.readfrom_mem(self.addr, 0x3B, 14)
        self.iic.stop()
        return a

    def get_ints(self):
        b = self.get_raw_values()
        c = []
        for i in b:
            c.append(i)
        return c

    def bytes_toint(self, firstbyte, secondbyte):
        if not firstbyte & 0x80:
            return firstbyte << 8 | secondbyte
        return - (((firstbyte ^ 255) << 8) | (secondbyte ^ 255) + 1)

    def error_gy(self):
        sleep(3)
        global error
        error=[0,0,0]
        vals = {}
        for i in range(0,10):
	            raw_ints = self.get_raw_values()
	            vals["GyX"] = self.bytes_toint(raw_ints[8], raw_ints[9])
	            vals["GyY"] = self.bytes_toint(raw_ints[10], raw_ints[11])
	            vals["GyZ"] = self.bytes_toint(raw_ints[12], raw_ints[13])
	            error[0]= error[0]+vals["GyX"]
	            error[1]= error[1]+vals["GyY"]
	            error[2]= error[2]+vals["GyZ"]
	            sleep_ms(8)
        error[1]=error[1]/10.0
        error[2]=error[2]/10.0
        error[0]=error[0]/10.0
		
    def get_values(self):
        global error
        vals = {}
        raw_ints = self.get_raw_values()
        vals["AcX"] = self.bytes_toint(raw_ints[0], raw_ints[1])
        vals["AcY"] = self.bytes_toint(raw_ints[2], raw_ints[3])
        vals["AcZ"] = self.bytes_toint(raw_ints[4], raw_ints[5])
        vals["Tmp"] = self.bytes_toint(raw_ints[6], raw_ints[7]) / 340.00 + 36.53
        vals["GyX"] = self.bytes_toint(raw_ints[8], raw_ints[9])-error[0]
        vals["GyY"] = self.bytes_toint(raw_ints[10], raw_ints[11])-error[1]
        vals["GyZ"] = self.bytes_toint(raw_ints[12], raw_ints[13])-error[2]
        #vals["GyZ1"] = self.bytes_toint(raw_ints[12], raw_ints[13])
        return vals  # returned in range of Int16
        # -32768 to 32767

