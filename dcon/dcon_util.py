import serial
from argparse import ArgumentParser
import time

parser = ArgumentParser()
parser.add_argument("command", metavar='CMD', type=str);
parser.add_argument("-d", "--device", dest="device",
                  help="Serial device", default="/dev/ttyACM0")
args = parser.parse_args();
#dev_err = int(options.dev_err)


ser = serial.Serial(None, 9600, timeout=0.5);
ser.port = args.device

while True:
    try:
        ser.open()
    except IOError as e:
        if e.errno != 16:
            raise
        time.sleep(2)
        print(options.device, "is busy, retrying")
        continue
    break
print(args)
cmd = args.command.encode("ASCII")

sum = 0

for c in cmd:
    sum += c
cmd += b'%02X\r' % (sum & 0xff)
print(cmd)
ser.write(cmd)

reply = b''
while True:
    r = ser.read()
    if r == b'': break
    reply += r
print(reply)
sum = 0

for b in reply[:-3]:
    sum += b
cs = int(reply[-3:-1],16)
if (sum & 0xff) != cs: print("CHhecksum error %02x != %02x"%(sum & 0xff, cs))
print(reply[:-3])
