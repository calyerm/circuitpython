# module   : roomba_ctrl.py
# Hardware : Adafruit Grand Central
#            Serial 1 , HW Uart : TX GPIO #1 , RX GPIO #0
# Roomba   : Model 805
# Python   : circuitpython 5.0.0
# Date     : 03/10/2020
# Author   : mcalyer
# Description:
# 1. Sends commands to Roomba
# 2. Reads sensor data
#
# Comments:
# 1. Specification: 10/2016 iRobot Create2 OI , Roomba 600


import board
import busio
import digitalio
import time
import struct


############### Commands ####################

CMD_RESET        = 7
CMD_OI_START     = 128
CMD_OI_STOP      = 173
CMD_POWER_DOWN   = 133
CMD_MODE_FULL    = 132
CMD_MODE_SAFE    = 131
CMD_SONG         = 140
CMD_SONG_PLAY    = 141
CMD_LEDS         = 139
CMD_LED_SEGS     = 163
CMD_LED_DIGIT    = 164
CMD_SENSOR       = 142
CMD_SENSOR_QUERY = 149
CMD_SEEK_DOCK    = 143
CMD_DRIVE        = 137
CMD_DRIVE_DIRECT = 145
CMD_DRIVE_PWM    = 146
CMD_MOTOR_POWER  = 138
CMD_SONG         = 140
CMD_SONG_PLAY    = 141
CMD_DELAY        = 999


ROOMBA_BEEP = [CMD_SONG,0,1,64,16,CMD_SONG_PLAY,0]


unpack_byte           = lambda x : struct.unpack('b',x)[0]   # 1 signed byte
unpack_unsigned_byte  = lambda x : struct.unpack('B',x)[0]   # 1 unsigned byte
unpack_short          = lambda x : struct.unpack('>h',x)[0]  # 2 signed bytes (short)
unpack_unsigned_short = lambda x : struct.unpack('>H',x)[0]  # 2 unsigned bytes (ushort)


def twos_Complement_2_Byte(val):
    if val < 0 :
        val = 0x10000 + val
    else:
        if val & 0x8000 != 0:
            val = val - 0x8000
    return ((val >> 8) & 0xFF) , val & 0xFF

class Roomba():
    def __init__(self):
        self.uart = None
        self.sensors_scan_nb = 0
        self.rx_error = None

    def connect(self):
        self.uart = None
        try:
            uart = busio.UART(board.TX, board.RX, baudrate=115200)
            self.uart = uart
        except:
            return 1
        return 0

    def disconnect(self):
        if self.uart is not None:
            self.uart.deinit()

    def flush_uart(self):
        return self.uart.reset_input_buffer()

    def send_cmd(self, cmd):
        return self.uart.write(bytes(cmd))

    def send_cmd_list(self,cmds,delay = .001):
        for cmd in cmds:
            if CMD_DELAY == cmd[0]:
                time.sleep(cmd[1])
                continue
            time.sleep(delay)
            nb = self.send_cmd(cmd)
            if nb is None : return None
        return nb

    def rxbytes(self, nb, delay = .001):
        self.error = None
        if delay != 0 : time.sleep(delay)
        wb = self.uart.in_waiting
        if nb == '?' : nb = wb
        if wb == 0 :
            self.rx_error = 'RX_Z'
            return None
        if nb != wb :
            self.flush_uart()
            self.error = 'RX_NE'
            return None
        return self.uart.read(nb)

    def start(self):      
        cmd_start_list = [[CMD_RESET], [CMD_DELAY,10] , [CMD_OI_START],[CMD_DELAY,.3], [CMD_MODE_FULL], [CMD_DELAY,.3]]       
        self.send_cmd_list(cmd_start_list, delay = .01)

    def stop(self):
        cmd_stop_list  = [[CMD_OI_STOP], [CMD_POWER_DOWN]]       
        self.send_cmd_list(cmd_stop_list, delay = 0)

    def sensor(self, packet_id,nbytes,delay):
        nb = self.send_cmd([CMD_SENSOR,packet_id])
        if nb is None : return None
        return self.rxbytes(nbytes, delay)

    def sensors_scan(self, sens):
        np = len(sens)
        packet = [CMD_SENSOR_QUERY, np]
        i = 0
        nb = 0
        for s in sens:
            packet.append(s.id)
            s.index = i
            nb = nb + s.nbytes
            i = i + s.nbytes
        self.sensors_scan_nb = nb
        return self.send_cmd(packet)

    def sensors_aquire(self, sens):
        data = self.rxbytes(self.sensors_scan_nb,0)
        if data is None :
            return 1
        for s in sens:
            try:
                s.value = s.unpack(data[s.index : s.index + s.nbytes])
            except:
                return  1
        return 0

    def song_op(self, s):
        return self.send_cmd(s)

    def LEDS(self, leds, color = 0xff , intensity = 0xFF):
        return self.send_cmd([CMD_LEDS, leds, color, intensity])

    def LED_Display(self,d3,d2,d1,d0):
        return self.send_cmd([CMD_LED_DIGIT, d3, d2, d1 , d0])

    def LED_Display_Segs(self, d3, d2, d1, d0):
        return self.send_cmd([CMD_LED_SEGS, d3, d2, d1, d0])

    def Drive_All(self, op, a, b):
            a_high_byte , a_low_byte = twos_Complement_2_Byte(a)
            b_high_byte , b_low_byte = twos_Complement_2_Byte(b)
            return self.send_cmd([op, a_high_byte, a_low_byte, b_high_byte, b_low_byte])

    def Drive(self, vel, rad):
        return self.Drive_All(CMD_DRIVE, vel, rad)

    def Direct_Drive(self, vel_right, vel_left):
        return self.Drive_All(CMD_DRIVE_DIRECT, vel_right, vel_left)

    def Drive_PWM(self, vel_right_pwm, vel_left_pwm):
        return self.Drive_All(CMD_DRIVE_PWM, vel_right_pwm, vel_left_pwm)


roomba = Roomba()

############# Sensors ########################

class Sensor():
    def __init__(self,id,nbytes,unpack, decode = None):
        self.sensor       = roomba.sensor
        self.id           = id
        self.nbytes       = nbytes
        self.unpack       = unpack
        self.data_decode  = decode
        self.value        = 0
        self.index        = 0

    def read(self, delay = .020):
        data = self.sensor(self.id,self.nbytes,delay)
        if data is None : return None
        try:
            self.value = self.unpack(data)
            return self.value
        except:
            return None

    def decode(self):
        if self.data_decode is None : return self.value
        return self.data_decode(self.value)


# OI Modes
OI_MODE_FULL    = 3
OI_MODE_SAFE    = 2
OI_MODE_PASSIVE = 1
sens_oi_mode           = Sensor(35 , 1, unpack_unsigned_byte)

# Buttons
sens_buttons           = Sensor(18 , 1, unpack_unsigned_byte)

# bump_wheel_decode: any_bump , bump left , bump right , any_wheel_drop, wheel drop left , wheel drop right
bump_wheel_decode = lambda x : (x&0x03, x&0x02, x&0x01, x&0x0C, x&0x08, x&0x04)
sens_bump_wheel_drop   = Sensor(7  , 1, unpack_unsigned_byte, bump_wheel_decode)



sens_cliff_left        = Sensor(9  , 1, unpack_unsigned_byte)
sens_cliff_right       = Sensor(12 , 1, unpack_unsigned_byte)
sens_cliff_front_left  = Sensor(10 , 1, unpack_unsigned_byte)
sens_cliff_front_right = Sensor(11 , 1, unpack_unsigned_byte)
sens_left_wheel_count  = Sensor(43 , 2, unpack_short)
sens_right_wheel_count = Sensor(44 , 2, unpack_short)
sens_over_currents     = Sensor(14 , 1, unpack_unsigned_byte)

sens_bat_volts         = Sensor(22 , 2, unpack_unsigned_short)
sens_bat_current       = Sensor(23 , 2, unpack_short)

sens_base_update = [sens_buttons,
                    sens_bump_wheel_drop,
                    sens_cliff_left,
                    sens_cliff_front_left,
                    sens_cliff_right,
                    sens_cliff_front_right,
                    sens_left_wheel_count,
                    sens_right_wheel_count]


sens_bat_update = [sens_bat_current,
                   sens_bat_volts]






