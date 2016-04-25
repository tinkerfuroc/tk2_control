import mmap
from utilities import *
import rospy

INPUT_FREQ = 12500000

OFFSET_VEL = 0
OFFSET_CNT = 4
OFFSET_CTRL = 8
OFFSET_FEEDBACK = 12

CTRL_DIR = 0x01
CTRL_ENABLE = 0x01 << 1

class Motor:
    def __init__(self, index):
        self.index = index
        self.io_file = open('/dev/uio%d' % index, 'rb+');
        self.reg_map = mmap.mmap(self.io_file.fileno(), 16);
        self.ctrl_reg = CTRL_DIR & ~CTRL_ENABLE 
        set_uint(self.reg_map, OFFSET_CTRL, self.ctrl_reg)

    def set_vel(self, vel):
        if(abs(vel) < 1):
            self.set_count(0)
        else:
            if(vel > 0):
                self.ctrl_reg |= CTRL_DIR
            else:
                self.ctrl_reg &= (~CTRL_DIR)
            set_uint(self.reg_map, OFFSET_CTRL, self.ctrl_reg)
            set_uint(self.reg_map, OFFSET_VEL, int(abs(INPUT_FREQ/vel)))

    def set_count(self, cnt):
        cnt *= 2
        set_uint(self.reg_map, OFFSET_CNT, int(cnt))

    def get_feedback(self):
        return get_int(self.reg_map, OFFSET_FEEDBACK)

    def enable(self):
        self.ctrl_reg &= (~CTRL_ENABLE)
        set_uint(self.reg_map, OFFSET_CTRL, self.ctrl_reg)

    def disable(self):
        self.ctrl_reg |= CTRL_ENABLE
        set_uint(self.reg_map, OFFSET_CTRL, self.ctrl_reg)

    def close(self):
        self.reg_map.close()
        self.io_file.close()


