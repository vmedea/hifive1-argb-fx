'''
Set caseleds led strip through 'f' (frame) command.
A hifive1 running hifive1-argb-fxl must be connected through USB.
'''
import math
import serial

from pyutil.delayedkeyboardinterrupt import DelayedKeyboardInterrupt

UART='/dev/serial/by-id/usb-FTDI_Dual_RS232-HS-if01-port0'
baudrate = 115200

def _toint(p):
    return math.floor(max(min(p, 255.0), 0.0))

class SimplexMode:
    FULLRGB = 0
    COLOR1 = 1
    COLOR2 = 2
    COLORN = 3

class Leds:
    count = 16

    def __init__(self):
        self.ser = serial.Serial(port=UART, baudrate=baudrate)

    def set(self, colors):
        assert(len(colors) == self.count)
        with DelayedKeyboardInterrupt(): # try hard to avoid sending a partial packet, while still honoring Ctrl-C
            self.ser.write(b'f')
            self.ser.read(1) # wait for confirmation of 'f' before sending the rest of the data
            cmd = b''
            for (r,g,b) in colors:
                cmd += bytes([_toint(r), _toint(g), _toint(b)])
            self.ser.write(cmd)

    def simplex(self, mode, colors=None):
        if colors is None:
            colors = []
        if len(colors) < self.count: # pad to num leds
            colors = colors[:] + [(0, 0, 0)] * (self.count - len(colors))
        with DelayedKeyboardInterrupt(): # try hard to avoid sending a partial packet, while still honoring Ctrl-C
            self.ser.write(b's')
            self.ser.read(1) # wait for confirmation before sending the rest of the data
            cmd = bytes([mode])
            for (r,g,b) in colors:
                cmd += bytes([_toint(r), _toint(g), _toint(b)])
            self.ser.write(cmd)
