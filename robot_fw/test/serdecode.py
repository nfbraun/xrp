import serial

DATA_PREFIX = 0x40
SHORT_FRAME_PREFIX = 0x50
LONG_FRAME_PREFIX = 0x60

_ser = serial.Serial('/dev/ttyUSB0', 38400, timeout=0.1, xonxoff=0, rtscts=0)
_resync = False

class SerDecodeError(Exception):
    pass

def _read(n):
    global _ser
    s = _ser.read(n)
    if len(s) != n:
        raise SerDecodeError("Serial timeout")
    
    return s

def resync():
    global _resync
    _resync = True

def read_frame():
    global _ser, _resync

    if _resync:
        _ser.flushInput()
        s = _read(1)
        while not(ord(s) & 0xF0 in (SHORT_FRAME_PREFIX, LONG_FRAME_PREFIX)):
            s = _read(1)
    else:
        s = _read(1)
    
    if not(ord(s) & 0xF0 in (SHORT_FRAME_PREFIX, LONG_FRAME_PREFIX)):
        raise SerDecodeError("Garbage at beginning of frame: %02X" % ord(s))
    
    _resync = False
    
    if ord(s) & 0xF0 == SHORT_FRAME_PREFIX:
        l = ord(s) & 0x0F
    else:
        s = _read(1)
        l = ord(s)
    
    rdata = _read(2*l)
    data = str()
    
    for i in range(0, 2*l, 2):
        data += chr(ord(rdata[i]) & 0x0F | (ord(rdata[i+1]) & 0x0F) << 4)
        
    return data
