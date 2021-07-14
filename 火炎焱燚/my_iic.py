from pynq.lib.pmod.pmod_iic import Pmod_IIC


class My_Pmod_IIC(Pmod_IIC):
    def __init__(self, mb_info, scl_pin, sda_pin, iic_addr):
        super().__init__(mb_info, scl_pin, sda_pin, iic_addr)

    def write_reg(self, addr, data):
        self.send([addr] + list(data))

    def read_reg(self, addr, num):
        self._iic_enable()
        self.write_cmd(self.dtr_addr, 0x100 | (self.iic_addr << 1))
        self.write_cmd(self.dtr_addr, addr)
        return bytes(self.receive(num))
