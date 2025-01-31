#!/usr/bin/env python

from .stservo_def import *
from .protocol_packet_handler import *
from .group_sync_read import *
from .group_sync_write import *

#波特率定义
# Baud rate definitions
STS_1M = 0
STS_0_5M = 1
STS_250K = 2
STS_128K = 3
STS_115200 = 4
STS_76800 = 5
STS_57600 = 6
STS_38400 = 7

#内存表定义
#-------EPROM(只读)--------
# Memory Table Definition
# -------EPROM (Read-Only) --------
STS_MODEL_L = 3
STS_MODEL_H = 4

#-------EPROM(读写)--------
# -------EPROM (Read/Write) --------
STS_ID = 5
STS_BAUD_RATE = 6
STS_MIN_ANGLE_LIMIT_L = 9
STS_MIN_ANGLE_LIMIT_H = 10
STS_MAX_ANGLE_LIMIT_L = 11
STS_MAX_ANGLE_LIMIT_H = 12
STS_CW_DEAD = 26
STS_CCW_DEAD = 27
STS_OFS_L = 31
STS_OFS_H = 32
STS_MODE = 33

#-------SRAM(读写)--------
# -------SRAM (Read/Write) --------
STS_TORQUE_ENABLE = 40
STS_ACC = 41
STS_GOAL_POSITION_L = 42
STS_GOAL_POSITION_H = 43
STS_GOAL_TIME_L = 44
STS_GOAL_TIME_H = 45
STS_GOAL_SPEED_L = 46
STS_GOAL_SPEED_H = 47
STS_LOCK = 55

#-------SRAM(只读)--------
# -------SRAM (Read-Only) --------
STS_PRESENT_POSITION_L = 56
STS_PRESENT_POSITION_H = 57
STS_PRESENT_SPEED_L = 58
STS_PRESENT_SPEED_H = 59
STS_PRESENT_LOAD_L = 60
STS_PRESENT_LOAD_H = 61
STS_PRESENT_VOLTAGE = 62
STS_PRESENT_TEMPERATURE = 63
STS_MOVING = 66
STS_PRESENT_CURRENT_L = 69
STS_PRESENT_CURRENT_H = 70

class sts(protocol_packet_handler):
    def __init__(self, portHandler):
        protocol_packet_handler.__init__(self, portHandler, 0)
        self.groupSyncWrite = GroupSyncWrite(self, STS_ACC, 7)

    def WritePosEx(self, sts_id, position, speed, acc):
        txpacket = [acc, self.sts_lobyte(position), self.sts_hibyte(position), 0, 0, self.sts_lobyte(speed), self.sts_hibyte(speed)]
        return self.writeTxRx(sts_id, STS_ACC, len(txpacket), txpacket)

    def ReadPos(self, sts_id):
        sts_present_position, sts_comm_result, sts_error = self.read2ByteTxRx(sts_id, STS_PRESENT_POSITION_L)
        return self.sts_tohost(sts_present_position, 15), sts_comm_result, sts_error

    def ReadSpeed(self, sts_id):
        sts_present_speed, sts_comm_result, sts_error = self.read2ByteTxRx(sts_id, STS_PRESENT_SPEED_L)
        return self.sts_tohost(sts_present_speed, 15), sts_comm_result, sts_error

    def ReadPosSpeed(self, sts_id):
        sts_present_position_speed, sts_comm_result, sts_error = self.read4ByteTxRx(sts_id, STS_PRESENT_POSITION_L)
        sts_present_position = self.sts_loword(sts_present_position_speed)
        sts_present_speed = self.sts_hiword(sts_present_position_speed)
        return self.sts_tohost(sts_present_position, 15), self.sts_tohost(sts_present_speed, 15), sts_comm_result, sts_error

    def ReadVoltage(self, sts_id):
        voltage, sts_comm_result, sts_error = self.read1ByteTxRx(sts_id, STS_PRESENT_VOLTAGE)
        return voltage, sts_comm_result, sts_error

    def ReadTemperature(self, sts_id):
        temperature, sts_comm_result, sts_error = self.read1ByteTxRx(sts_id, STS_PRESENT_TEMPERATURE)
        return temperature, sts_comm_result, sts_error

    def ReadCurrent(self, sts_id):
        current_l, sts_comm_result_l, sts_error_l = self.read1ByteTxRx(sts_id, STS_PRESENT_CURRENT_L)
        current_h, sts_comm_result_h, sts_error_h = self.read1ByteTxRx(sts_id, STS_PRESENT_CURRENT_H)
        if sts_comm_result_l == COMM_SUCCESS and sts_comm_result_h == COMM_SUCCESS:
            current = (current_h << 8) | current_l
            return current, COMM_SUCCESS, 0
        else:
            return None, sts_comm_result_l, sts_error_l

    def ReadLoad(self, sts_id):
        load_l, sts_comm_result_l, sts_error_l = self.read1ByteTxRx(sts_id, STS_PRESENT_LOAD_L)
        load_h, sts_comm_result_h, sts_error_h = self.read1ByteTxRx(sts_id, STS_PRESENT_LOAD_H)
        if sts_comm_result_l == COMM_SUCCESS and sts_comm_result_h == COMM_SUCCESS:
            load = (load_h << 8) | load_l
            return load, COMM_SUCCESS, 0
        else:
            return None, sts_comm_result_l, sts_error_l


    def ReadMoving(self, sts_id):
        moving, sts_comm_result, sts_error = self.read1ByteTxRx(sts_id, STS_MOVING)
        return moving, sts_comm_result, sts_error

    def SyncWritePosEx(self, sts_id, position, speed, acc):
        txpacket = [acc, self.sts_lobyte(position), self.sts_hibyte(position), 0, 0, self.sts_lobyte(speed), self.sts_hibyte(speed)]
        return self.groupSyncWrite.addParam(sts_id, txpacket)

    def RegWritePosEx(self, sts_id, position, speed, acc):
        txpacket = [acc, self.sts_lobyte(position), self.sts_hibyte(position), 0, 0, self.sts_lobyte(speed), self.sts_hibyte(speed)]
        return self.regWriteTxRx(sts_id, STS_ACC, len(txpacket), txpacket)

    def RegAction(self):
        return self.action(BROADCAST_ID)

    def WriteID(self, current_id, new_id):
        """ Changes the servo's ID. Must be done while the servo is the only one connected. """
        return self.write1ByteTxRx(current_id, STS_ID, new_id)


    def WheelMode(self, sts_id):
        return self.write1ByteTxRx(sts_id, STS_MODE, 1)

    def WriteSpec(self, sts_id, speed, acc):
        speed = self.sts_toscs(speed, 15)
        txpacket = [acc, 0, 0, 0, 0, self.sts_lobyte(speed), self.sts_hibyte(speed)]
        return self.writeTxRx(sts_id, STS_ACC, len(txpacket), txpacket)

    def LockEprom(self, sts_id):
        return self.write1ByteTxRx(sts_id, STS_LOCK, 1)

    def unLockEprom(self, sts_id):
        return self.write1ByteTxRx(sts_id, STS_LOCK, 0)

