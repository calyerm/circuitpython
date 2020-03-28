# module   : roomba_live.py
# Hardware : Adafruit Grand Central
#            Serial 1 , HW Uart : TX GPIO #1 , RX GPIO #0
# Roomba   : Model 805
# Python   : circuitpython 5.0.0
# Date     : 03/28/2020
# Author   : mcalyer
# Description:
# 1. Sends commands to Roomba
# 2. Reads sensor data
#
# Comments:
# 1. Specification: 10/2016 iRobot Create2 OI , Roomba 600


import board
import digitalio
import time
from   roomba_ctrl import *
import neopixel
from roomba_effects import *


############# Board NEO Pixel ########################################

neo_leds = neopixel.NeoPixel(board.NEOPIXEL, 1, brightness=0.3, auto_write=False)
NEO_OFF    = (0,   0  ,  0)
NEO_RED    = (255, 0  , 0)
NEO_YELLOW = (255, 150, 0)
NEO_GREEN  = (0  , 255, 0)
NEO_BLUE   = (0  , 0  , 255)
NEO_PURPLE = (180, 0  , 255)
#NEO_CYAN   = (0  , 255, 255)
def status_led(color,neo = neo_leds):
    neo.fill(color)
    neo.show()
def status_led_flash(color, rate):
    status_led(color)
    time.sleep(rate)
    status_led(NEO_OFF)
    time.sleep(rate)

############### Roomba LED Digits ################################

def LED_Display_Int(n):
    s = str(n)
    if n > 9999 or n < 0 : s = '----'
    s = (4 - len(s)) * '0' + s
    return roomba.LED_Display( ord(s[0]), ord(s[1]),  ord(s[2]),  ord(s[3]))

def LED_Display_Float(n):
    s = str(n)
    if n > 99.9 or n < .001 : s = '----'
    if s[3] == '.' : s = s[0:3]
    s = (4 - len(s)) * '0' + s
    return roomba.LED_Display( ord(s[0]), ord(s[1]),  ord(s[2]),  ord(s[3]))

############### Roomba Lives ######################################

# Roomba 
# Push clean button first !


def roomba_setup():
    status_led(NEO_GREEN)
    # Serial port , if failure , indicate error , loop , wait for user reset
    roomba.disconnect()
    if roomba.connect():
        # Serial port error
        while True:
            status_led_flash(NEO_RED, .5)

    # Send Commands RESET, START, MODE_FULL
    # Should hear tone sequence during reset
    # Should hear start beep
    roomba.start()

    # Flush UART , reset process sends out Roomba system info serial rx
    roomba.flush_uart()

    time.sleep(1)

    # Read Mode
    # Check serial rx , read from sensor
    m = sens_oi_mode.read()
    if m != OI_MODE_FULL:
        # Error can not read sensors
        roomba_exit()
        while True:
            status_led_flash(NEO_YELLOW, .5)

    # First sensor scan
    #roomba.sensors_scan(sens_bat_update)
    #time.sleep(1)

def roomba_exit():
    roomba.stop()
    time.sleep(.1)
    roomba.disconnect()

def roomba_loop():
    # Behavior Loop
    LOOP_TIME    = .1 # seconds
    loop_counter      = 0
    loop_counter_10   = 0

    sens_ok_flag = False
    sens_update  = sens_base_update + sens_bat_update   

    try:
        while(True):
            response_timer = time.monotonic()

            # Sensors
            sens_ok_flag = True
            roomba.sensors_scan(sens_update)
            time.sleep(.020)
            if roomba.sensors_aquire(sens_update) :  sens_ok_flag = False

            # Prime Directive
          
            # Periodics
            loop_counter += 1
            if 0 == loop_counter % 10 : loop_counter_10   += 1
            # run indicator
            status_led(NEO_OFF if loop_counter_10 & 0x01 else NEO_PURPLE)
            sens_update = sens_base_update + sens_bat_update if 0 == loop_counter % 1000 else sens_base_update

            # Response time
            tr = LOOP_TIME - (time.monotonic() - response_timer)
            if tr > 0 : time.sleep(tr)

            # End of Loop
    except:
        # Loop Error
        roomba_exit()
        while True:
            status_led_flash(NEO_RED, .5)


def run():
    roomba_setup()
    roomba_loop()

