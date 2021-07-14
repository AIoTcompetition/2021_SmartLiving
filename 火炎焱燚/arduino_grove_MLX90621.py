#   Copyright (c) 2016, Xilinx, Inc.
#   All rights reserved.
#
#   Redistribution and use in source and binary forms, with or without
#   modification, are permitted provided that the following conditions are met:
#
#   1.  Redistributions of source code must retain the above copyright notice,
#       this list of conditions and the following disclaimer.
#
#   2.  Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#
#   3.  Neither the name of the copyright holder nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
#   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
#   AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
#   THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
#   PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
#   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
#   EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
#   PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
#   OR BUSINESS INTERRUPTION). HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
#   WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
#   OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
#   ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


import math
from . import Arduino
from . import ARDUINO_GROVE_I2C
from time import sleep
import numpy as np

# MLX90621 command
ARDUINO_GROVE_MLX90621_PROGRAM = "arduino_grove_MLX90621.bin"
CONFIG_IOP_SWITCH = 0x1
MLX90621_GET_EEPROM = 0x03
MLX90621_GET_PTAT = 0x05
MLX90621_GET_CPIX = 0x07
MLX90621_GET_CONFIG = 0x09
MLX90621_SET_CONFIG = 0x0B
MLX90621_GET_TRIMMING_VALUE = 0x0D
MLX90621_SET_TRIMMING_VALUE = 0x0F
MLX90621_GET_IR = 0x11
RESET = 0xF

# eeprom address map
CAL_ACOMMON = 0xD0
CAL_ACP = 0xD3
CAL_BCP = 0xD5
CAL_alphaCP = 0xD6
CAL_TGC = 0xD8
CAL_AI_SCALE = 0xD9
CAL_BI_SCALE = 0xD9

KT_SCALE = 0xD2
VTH = 0xDA
KT1 = 0xDC
KT2 = 0xDE

CAL_A0 = 0xE0
CAL_A0_SCALE = 0xE2
CAL_DELTA_A_SCALE = 0xE3
CAL_EMIS = 0xE4

RATE_BYTE = {
    0.5: 0xF,
    1: 0xE,
    2: 0xD,
    4: 0xC,
    8: 0xB,
    16: 0xA,
    32: 0x9,
    64: 0x8,
    128: 0x7,
    256: 0x6,
    512: 0x5
}


def _reg2float(reg):
    if reg == 0:
        return 0.0
    sign = (reg & 0x80000000) >> 31 & 0x01
    exp = ((reg & 0x7f800000) >> 23) - 127
    if exp == 0:
        man = (reg & 0x007fffff) / pow(2, 23)
    else:
        man = 1 + (reg & 0x007fffff) / pow(2, 23)
    result = pow(2, exp) * man * ((sign * -2) + 1)
    return float("{0:.2f}".format(result))


def _reg2int(reg):
    result = -(reg >> 31 & 0x1) * (1 << 31)
    for i in range(31):
        result += (reg >> i & 0x1) * (1 << i)
    return result


def byte_merge(byte1, byte2) -> int:
    return (byte1 << 8) + byte2


def signed(num, base=16) -> int:
    return num - (1 << base) if num >= 1 << (base - 1) else num


class Grove_MLX90621(object):

    def __init__(self, mb_info, gr_pin, detect_rate):

        if gr_pin not in [ARDUINO_GROVE_I2C]:
            raise ValueError("Group number can only be I2C.")

        if detect_rate not in RATE_BYTE.keys():
            raise ValueError("Rate is not valid.")
        sleep(0.005)
        self.microblaze = Arduino(mb_info, ARDUINO_GROVE_MLX90621_PROGRAM)
        self.reset()

        self.get_eeprom()
        self.set_config(0x4630 + RATE_BYTE[detect_rate])
        self.set_trimming_value(self.eeprom[0xF7])

    def eeprom_get_uint(self, addr):
        return byte_merge(self.eeprom[addr + 1], self.eeprom[addr])

    def eeprom_get_int(self, addr):
        return signed(byte_merge(self.eeprom[addr + 1], self.eeprom[addr]))

    def get_temps(self):

        if not (self.get_config() & 0x0400) >> 10:
            self.get_eeprom()
            self.set_trimming_value(self.eeprom[0xF7])
            self.set_config()

        ptat = self.get_ptat()
        IR_data = np.array(self.get_IR())

    
        k_t1_scale = (self.eeprom[KT_SCALE] & 0xF0) >> 4
        k_t2_scale = (self.eeprom[KT_SCALE] & 0x0F) + 10

        v_th = self.eeprom_get_int(VTH)
        v_th = v_th / (1 << (3 - self.resolution))

        k_t1 = self.eeprom_get_int(KT1)
        k_t1 = k_t1 / (1 << (3 - self.resolution + k_t1_scale))

        k_t2 = self.eeprom_get_int(KT2)
        k_t2 = k_t2 / (1 << (3 - self.resolution + k_t2_scale))
        Tambient = ((k_t1 ** 2 - ((v_th - ptat) * k_t2 * 4))
                    ** 0.5 - k_t1) / (k_t2 * 2) + 25.0

        cpix = self.get_cpix()

    
        emissivity = self.eeprom_get_uint(CAL_EMIS) / 32768.0
        a_common = self.eeprom_get_int(CAL_ACOMMON)

        alpha_cp = self.eeprom_get_uint(
            CAL_alphaCP) / (1 << (3 - self.resolution + CAL_A0_SCALE))

        a_i_scale = (self.eeprom[CAL_AI_SCALE] & 0xF0) >> 4
        b_i_scale = self.eeprom[CAL_BI_SCALE] & 0x0F

        a_cp = self.eeprom_get_int(CAL_ACP) / (1 << (3 - self.resolution))
        b_cp = signed(self.eeprom[CAL_BCP], 8) / \
            (1 << (3 - self.resolution + b_i_scale))

        tgc = signed(self.eeprom[CAL_TGC], 8) / 32

        v_cp_off_comp = cpix - (a_cp + b_cp * (Tambient - 25.0))

        a_ij = (
            a_common + (np.array(self.eeprom[:0x40]) << a_i_scale)) / (1 << 3 - self.resolution)
        b_ij = np.vectorize(lambda v: signed(v, 8))(
            np.array(self.eeprom[0x40: 0x80]))
        b_ij = b_ij / (1 << 3 - self.resolution + b_i_scale)

        v_ir_tgc_comp = IR_data - \
            (a_ij + b_ij * (Tambient - 25.0)) - tgc * v_cp_off_comp
        alpha_ij = self.eeprom_get_uint(
            CAL_A0) / (1 << self.eeprom[CAL_A0_SCALE])
        alpha_ij += np.array(self.eeprom[0x80:0xC0]) / \
            (1 << self.eeprom[CAL_DELTA_A_SCALE])
        alpha_ij = alpha_ij / (1 << 3 - self.resolution)
        v_ir_norm = v_ir_tgc_comp / (alpha_ij - tgc * alpha_cp)
        v_ir_comp = v_ir_norm / emissivity

        return (np.exp(np.log(v_ir_comp + (Tambient + 273.15) ** 4) / 4) - 273.15).reshape(16, 4).transpose()

    def get_eeprom(self):
        self.microblaze.write_blocking_command(MLX90621_GET_EEPROM)
        self.eeprom = [_reg2int(i)
                       for i in self.microblaze.read_mailbox(0, 256)]

    def get_ptat(self):
        self.microblaze.write_blocking_command(MLX90621_GET_PTAT)
        return _reg2int(self.microblaze.read_mailbox(0, 1))

    def get_cpix(self):
        self.microblaze.write_blocking_command(MLX90621_GET_CPIX)
        return _reg2int(self.microblaze.read_mailbox(0, 1))

    def get_config(self):
        self.microblaze.write_blocking_command(MLX90621_GET_CONFIG)
        self.config = _reg2int(self.microblaze.read_mailbox(0, 1))
        return self.config

    def set_config(self, config=None):
        if config is not None:
            self.config = config
        self.microblaze.write_mailbox(0, [self.config])
        self.microblaze.write_blocking_command(MLX90621_SET_CONFIG)

        self.resolution = (self.get_config() & 0x30) >> 4

    def get_trimming_value(self):
        self.microblaze.write_blocking_command(MLX90621_GET_TRIMMING_VALUE)
        self.trimming_value = self.microblaze.read_mailbox(0, 1)
        return self.trimming_value

    def set_trimming_value(self, trimming_value):
        self.trimming_value = trimming_value
        self.microblaze.write_mailbox(0, [trimming_value])
        self.microblaze.write_blocking_command(MLX90621_SET_TRIMMING_VALUE)

    def get_IR(self):
        self.microblaze.write_blocking_command(MLX90621_GET_IR)
        return [_reg2int(i) for i in self.microblaze.read_mailbox(0, 0x40)]

    def reset(self):
        self.microblaze.write_blocking_command(RESET)
