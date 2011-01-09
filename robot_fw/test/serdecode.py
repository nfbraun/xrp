import serial

ESCAPE = 0xBA
ESCAPE_CHAR = 0x40
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
        while True:
            s = _read(1)
            while not(ord(s) == ESCAPE):
                s = _read(1)
            s = _read(1)
            if ord(s) != ESCAPE_CHAR:
                break
    else:
        s = _read(1)
        if ord(s) != ESCAPE:
            raise SerDecodeError("Garbage at beginning of frame: %02X" % ord(s))
        s = _read(1)
    
    if not(ord(s) & 0xF0 in (SHORT_FRAME_PREFIX, LONG_FRAME_PREFIX)):
        raise SerDecodeError("Garbage at beginning of frame: %02X" % ord(s))
    
    _resync = False
    
    if ord(s) & 0xF0 == SHORT_FRAME_PREFIX:
        l = ord(s) & 0x0F
    else:
        s = _read(1)
        l = ord(s)
    
    data = str()
    
    for i in range(0, l):
        s = _read(1)
        if ord(s) == ESCAPE:
            s = _read(1)
            if ord(s) == ESCAPE_CHAR:
                data += chr(ESCAPE)
            else:
                raise SerDecodeError("Unexpected escape sequence: %02X" % ord(s))
        else:
            data += s
        
    return data
