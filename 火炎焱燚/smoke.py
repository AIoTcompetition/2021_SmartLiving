from .my_iic import My_Pmod_IIC
from struct import unpack


class Smoke:
    def __init__(self, mb_info):
        self.iic = My_Pmod_IIC(mb_info, 2, 3, 0x64)
        self.iic.write_reg(0x4B, b'\x26\x92')
        self.iic.write_reg(0x10, b'\x00\x01')
        self.iic.write_reg(0x11, b'\x30\xA9')
        self.iic.write_reg(0x12, b'\x02\x00')
        self.iic.write_reg(0x14, b'\x01\x1D')
        self.iic.write_reg(0x15, b'\x00\x00')
        self.iic.write_reg(0x17, b'\x00\x09')
        self.iic.write_reg(0x18, b'\x00\x00')
        self.iic.write_reg(0x19, b'\x3F\xFF')
        self.iic.write_reg(0x1A, b'\x3F\xFF')
        self.iic.write_reg(0x1B, b'\x3F\xFF')
        self.iic.write_reg(0x1D, b'\x00\x09')
        self.iic.write_reg(0x1E, b'\x00\x00')
        self.iic.write_reg(0x1F, b'\x3F\xFF')
        self.iic.write_reg(0x20, b'\x3F\xFF')
        self.iic.write_reg(0x21, b'\x3F\xFF')
        self.iic.write_reg(0x22, b'\x35\x39')
        self.iic.write_reg(0x23, b'\x35\x36')
        self.iic.write_reg(0x24, b'\x15\x30')
        self.iic.write_reg(0x25, b'\x63\x0C')
        self.iic.write_reg(0x30, b'\x03\x20')
        self.iic.write_reg(0x31, b'\x04\x0E')
        self.iic.write_reg(0x35, b'\x03\x20')
        self.iic.write_reg(0x36, b'\x04\x0E')
        self.iic.write_reg(0x39, b'\x22\xF0')
        self.iic.write_reg(0x3B, b'\x22\xF0')
        self.iic.write_reg(0x3C, b'\x31\xC6')
        self.iic.write_reg(0x42, b'\x1C\x34')
        self.iic.write_reg(0x43, b'\xAD\xA5')
        self.iic.write_reg(0x44, b'\x1C\x34')
        self.iic.write_reg(0x45, b'\xAD\xA5')
        self.iic.write_reg(0x58, b'\x05\x44')
        self.iic.write_reg(0x54, b'\x0A\xA0')
        self.iic.write_reg(0x10, b'\x00\x02')

    def read(self):
        buffer = unpack('>HHHH', self.iic.read_reg(0x60, 8))
        return (buffer[1] << 16) + buffer[0], (buffer[3] << 16) + buffer[2]
