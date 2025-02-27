from loguru import logger
import numpy as np
import struct
import plotly.graph_objects as go
import time
import copy

config = {
    "REG_SYSTEM_BASE"       : 0x44A0_0000,
    "RGB_0_BASE"            : 0xC000_0000,
    "RGB_1_BASE"            : 0xC200_0000,
    "SPI_0_BASE"            : 0xC400_0000,
    "SPI_1_BASE"            : 0xC600_0000,
    "AXI_GPIO_BASE"         : 0x4000_0000,
    "IIC_0_BASE"            : 0x4080_0000,
    "IIC_1_BASE"            : 0x4081_0000
}


class ToolClass():
    def __init__(self, dev_instance):
        self.dev = dev_instance
        self.conf = config
        self.read = dev_instance.read
        self.write = dev_instance.write
        self.wait = dev_instance.wait
        self.breath_light()
### GPIO begin
    def gpio_init_w_pin(self, device, w_pin): #o.w. read pin
        if device=='gpio_3v3':
            axi_addr = self.conf["AXI_GPIO_BASE"] + 4*1
        else:
            print("no device:", device)
            return -1

        w_data = 0
        for pin in w_pin:
            w_data = w_data | (1<<pin)
        w_data = ~w_data
        self.write(axi_addr,[w_data])

    def gpio_write(self, device, w_pin_data):
        if device=='gpio_3v3':
            axi_addr = self.conf["AXI_GPIO_BASE"] + 4*0
        else:
            print("no device:", device)
            return -1

        w_data = self.read(axi_addr, 1)[0]
        for pin in w_pin_data:
            if w_pin_data[pin]==0:
                w_data &= ~(1<<pin)
            elif w_pin_data[pin]==1:
                w_data |= (1<<pin)
            else:
                print(f'illegel value {w_pin_data[pin]} on pin {pin}')
                return -1
            
        self.write(axi_addr,[w_data])

    def gpio_read(self, device): # return r_data
        if device=='gpio_3v3':
            axi_addr = self.conf["AXI_GPIO_BASE"] + 4*0
        else:
            print("no device:", device)
            return -1

        r_data = self.read(axi_addr,1)[0]
        print(f'{r_data:022b}')
        return r_data
### GPIO end

### I2C begin
    def i2c_read(self, device, slv_addr, reg_ptr, byte_len): # if reg_ptr is not used, fill in []
        if device=='iic_0':
            axi_addr = self.conf["IIC_0_BASE"]
        elif device=='iic_1':
            axi_addr = self.conf["IIC_1_BASE"]
        else:
            print("no device:", device)
            return -1

        self.write(axi_addr+0x120,[0xF])
        self.write(axi_addr+0x100,[0x2])
        self.write(axi_addr+0x100,[0x0])
        if len(reg_ptr)>0:
            self.write(axi_addr+0x108,[0x100 | slv_addr<<1])
        for i in range(len(reg_ptr)):
            self.write(axi_addr+0x108,[reg_ptr[i]])
        self.write(axi_addr+0x108,[0x101 | slv_addr<<1])
        self.write(axi_addr+0x108,[0x200 | byte_len])
        self.write(axi_addr+0x100,[0x1])

        read_data = [0]*byte_len
        for i in range(byte_len): # read bytes
            self.wait(addr    =axi_addr+0x104,
                mask    =0x00000040,
                expected=0x00000000,
                timeout =1000000); # uSec
            read_data[i] = self.read(axi_addr+0x10C,1)[0]

        self.wait(addr    =axi_addr+0x104,
            mask    =0x00000004,
            expected=0x00000000,
            timeout =1000000); # uSec 
        return read_data
        
    def i2c_write(self, device, slv_addr, reg_ptr, write_data): # if reg_ptr is not used, fill in []
        if device=='iic_0':
            axi_addr = self.conf["IIC_0_BASE"]
        elif device=='iic_1':
            axi_addr = self.conf["IIC_1_BASE"]
        else:
            print("no device:", device)
            return -1

        self.write(axi_addr+0x120,[0xF])
        self.write(axi_addr+0x100,[0x2])
        self.write(axi_addr+0x100,[0x0])
        self.write(axi_addr+0x108,[0x100 | slv_addr<<1])
        for i in range(len(reg_ptr)):
            self.write(axi_addr+0x108,[reg_ptr[i]])
        for i in range(len(write_data)-1):
            self.write(axi_addr+0x108,[write_data[i]])
        self.write(axi_addr+0x108,[0x200 | write_data[len(write_data)-1]])
        self.write(axi_addr+0x100,[0x1])

        self.wait(addr    =axi_addr+0x104,
            mask    =0x00000084,
            expected=0x00000080,
            timeout =1000000); # uSec
### I2C end

### SPI begin
    def SPI_init(self, device, SPI_CSB_IDLE_STATE, SPI_CSB_MASK, SPI_CPOL, SPI_CPHA, SPI_SDIO_mode):
        if device=='SPI_0':
            SPI_bit_shift = 0
        elif device=='SPI_1':
            SPI_bit_shift = 8
        else:
            print("no device:", device)
            return -1
        SPI_REG_BASE_ADDR = self.conf["REG_SYSTEM_BASE"]

        SPI_CONFIG = ((0b11111000<<16) | (SPI_CSB_IDLE_STATE<<7) | (SPI_CSB_MASK<<6) | (SPI_CPOL<<5) | (SPI_CPHA<<4) | (SPI_SDIO_mode<<3)) << SPI_bit_shift
        self.write(SPI_REG_BASE_ADDR,[SPI_CONFIG])


    def SPI_program(self, device, w_data, r_bytes):
        SPI_start_bit = 0
        SPI_REG_BASE_ADDR = self.conf["REG_SYSTEM_BASE"]
        for idx, d in enumerate(device):
            if d=='SPI_0':
                SPI_start_bit += 1
                SPI_BRAM_BASE_ADDR = self.conf["SPI_0_BASE"]
                SPI_LEN_BASE_ADDR = self.conf["REG_SYSTEM_BASE"] + 4*1

            elif d=='SPI_1':
                SPI_start_bit += (1<<8)
                SPI_BRAM_BASE_ADDR = self.conf["SPI_1_BASE"]
                SPI_LEN_BASE_ADDR = self.conf["REG_SYSTEM_BASE"] + 4*2
            else:
                print("no device:", device)
                return -1

            SPI_wr_bytes = np.size(w_data[idx])//4
            SPI_rd_bytes = r_bytes[idx]
            self.write(SPI_LEN_BASE_ADDR, [(SPI_wr_bytes << 16) | (SPI_rd_bytes)]) #[31:16] SPI_wr_bytes, [15:0] SPI_rd_bytes 

            bram_data = np.array(w_data[idx], dtype="uint8")
            bram_data = bram_data.T
            bram_data = np.reshape(bram_data, np.size(bram_data))
            bram_data = bram_data.view('uint32')
            self.write(SPI_BRAM_BASE_ADDR, bram_data)

        
        self.write(SPI_REG_BASE_ADDR,[(1<<16) | (1<<24) | SPI_start_bit]) # start send SPI
        self.wait(addr    =SPI_REG_BASE_ADDR,
            mask     =(SPI_start_bit<<2),
            expected =(SPI_start_bit<<2),
            timeout  =1000000); # uSec
        self.write(SPI_REG_BASE_ADDR,[(1<<18) | (1<<26)]) # axi_32_sys_control_reg clear flag

        r_data = []
        for idx, d in enumerate(device):
            if d=='SPI_0':
                SPI_BRAM_BASE_ADDR = self.conf["SPI_0_BASE"]

            elif d=='SPI_1':
                SPI_BRAM_BASE_ADDR = self.conf["SPI_1_BASE"]

            SPI_wr_bytes = np.size(w_data[idx])//4
            SPI_rd_bytes = r_bytes[idx]

            bram_data = self.read(SPI_BRAM_BASE_ADDR+SPI_wr_bytes*4, SPI_rd_bytes)
            bram_data = np.array(bram_data, dtype="uint32")
            bram_data = bram_data.view('uint8')
            bram_data = np.reshape(bram_data, (SPI_rd_bytes, 4))
            r_data.append(bram_data)
            # print(r_data)

        return r_data


### SPI end


    def parse_usr_access_timestamp(self):
        from datetime import datetime
        data = self.read(self.conf["REG_SYSTEM_BASE"]+4*4, 1)[0]
        day = (data >> (32-5)) & 0b11111
        month = (data >> (32-9)) & 0b1111
        year = ((data >> (32-15)) & 0b111111) + 2000
        hour = (data >> (32-20)) & 0b11111
        min = (data >> (32-26)) & 0b111111
        sec = (data ) & 0b111111

        # datetime(year, month, day, hour, minute, second, microsecond)
        return datetime(year, month, day, hour, min, sec, 0)

    def edit_RGB(self, script, sel):
        bram_data = []
        for step in script:
            while step[0]>255:
                data = (255<<24) | (step[1]<<16) | (step[2]<<8) | (step[3]) 
                bram_data.append(data)
                step[0]-=255
            data = (step[0]<<24) | (step[1]<<16) | (step[2]<<8) | (step[3]) 
            bram_data.append(data)
        bram_data.append(0) # EOF
        if len(bram_data) > 2048:
            raise ValueError("BRAM Overflow!")
        if sel==0:
            self.write(self.conf["RGB_0_BASE"], bram_data)
        elif sel==1:
            self.write(self.conf["RGB_1_BASE"], bram_data)

    def on_off_RGB(self, sel):
        data = 0
        for s in sel:
            data += 1 << s
        self.write(self.conf["REG_SYSTEM_BASE"]+4*3,[data])

    
    def breath_light(self):
        import numpy as np
        self.on_off_RGB([]) # turn off 
        # R_set = 46
        # G_set = 63
        # B_set = 143

        R_set = 21
        G_set = 38
        B_set = 255

        script = []
        N_point = 1500
        for idx in range(N_point):
            deg = idx/N_point*180
            t = 3
            R = int(np.sin(np.deg2rad(deg))*R_set)
            G = int(np.sin(np.deg2rad(deg))*G_set)
            B = int(np.sin(np.deg2rad(deg))*B_set)
            script.append([t,R,G,B])
        # print(script)
        self.edit_RGB(script, 0)

        R_set = 21
        G_set = 38
        B_set = 255

        script = []
        N_point = 1500
        for idx in range(N_point):
            deg = idx/N_point*180+90
            if deg>180:
                deg-=180
            t = 3
            R = int(np.sin(np.deg2rad(deg))*R_set)
            G = int(np.sin(np.deg2rad(deg))*G_set)
            B = int(np.sin(np.deg2rad(deg))*B_set)
            script.append([t,R,G,B])
        # print(script)
        self.edit_RGB(script, 1)

        self.on_off_RGB([0, 1]) # turn on RGB_0, RGB_1

