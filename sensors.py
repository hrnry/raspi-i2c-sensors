#!/usr/bin/env python3

import argparse
import atexit
import datetime
import os
import posix
import signal
import sys
import time
from ctypes import c_uint32, c_uint8, c_uint16, c_char, POINTER, Structure, Array, Union, create_string_buffer, string_at
from fcntl import ioctl
from wsgiref.simple_server import make_server, WSGIRequestHandler

LISTEN_PORT = 8080
PIDFILE = '/tmp/sensors_py.pid'

# uapi/linux/i2c-dev.h
I2C_SLAVE = 0x0703
I2C_SMBUS = 0x0720

# uapi/linux/i2c.h
I2C_SMBUS_WRITE = 0
I2C_SMBUS_READ = 1
I2C_SMBUS_BYTE_DATA = 2
I2C_SMBUS_I2C_BLOCK_DATA = 8
I2C_SMBUS_BLOCK_MAX = 32


# https://github.com/kplindegaard/smbus2/blob/master/smbus2/smbus2.py
class i2c_smbus_data(Array):
  _length_ = I2C_SMBUS_BLOCK_MAX + 2
  _type_ = c_uint8

class union_i2c_smbus_data(Union):
  _fields_ = [
    ('byte', c_uint8),
    ('word', c_uint16),
    ('block', i2c_smbus_data)
  ]

union_pointer_type = POINTER(union_i2c_smbus_data)

class SMBus:
  class i2c_smbus_ioctl_data(Structure):
    _fields_ = [
      ('read_write', c_uint8),
      ('command', c_uint8),
      ('size', c_uint32),
      ('data', union_pointer_type)
    ]
    __slots__ = [name for name, type in _fields_]

    @staticmethod
    def create(read_write=I2C_SMBUS_READ, command=0, size=I2C_SMBUS_BYTE_DATA):
      u = union_i2c_smbus_data()
      return SMBus.i2c_smbus_ioctl_data(read_write=read_write, command=command, size=size, data=union_pointer_type(u))

  def __init__(self, i2cbus = 1):
    self._i2cbus = i2cbus
    self.fd = posix.open('/dev/i2c-%d' % self._i2cbus, posix.O_RDWR)

  def close(self):
    if self.fd:
      posix.close(self.fd)
      self.fd = None

  def read_i2c_block_data(self, i2c_addr, register, length):
    ioctl(self.fd, I2C_SLAVE, i2c_addr)
    msg = SMBus.i2c_smbus_ioctl_data.create(read_write=I2C_SMBUS_READ, command=register, size=I2C_SMBUS_I2C_BLOCK_DATA)
    msg.data.contents.byte = length
    ioctl(self.fd, I2C_SMBUS, msg)
    return msg.data.contents.block[1:length + 1]

  def write_byte_data(self, i2c_addr, register, value):
    ioctl(self.fd, I2C_SLAVE, i2c_addr)
    msg = SMBus.i2c_smbus_ioctl_data.create(read_write=I2C_SMBUS_WRITE, command=register, size=I2C_SMBUS_BYTE_DATA)
    msg.data.contents.byte = value
    ioctl(self.fd, I2C_SMBUS, msg)


# AM2320 - Digital Temperature and Humidity Sensor
#   -40 ~ +80℃ ±0.5℃
#   0 ~ 99.9%RH ±3%RH(@25℃)
# http://akizukidenshi.com/catalog/g/gM-08663/
# http://akizukidenshi.com/download/ds/aosong/AM2320.pdf
# https://github.com/Gozem/am2320/blob/master/am2320.py
# https://github.com/adafruit/Adafruit_CircuitPython_AM2320
class AM2320:
  I2C_ADDR = 0x5c

  def __init__(self, i2cbus = 1):
    self._i2cbus = i2cbus

  @staticmethod
  def _calc_crc16(data):
    crc = 0xFFFF
    for x in data:
      crc = crc ^ x
      for _ in range(0, 8):
        if (crc & 0x0001) == 0x0001:
          crc >>= 1
          crc ^= 0xA001
        else:
          crc >>= 1
    return crc

  @staticmethod
  def _combine_bytes(msb, lsb):
    return msb << 8 | lsb

  def read_sensor(self):
    fd = posix.open('/dev/i2c-%d' % self._i2cbus, posix.O_RDWR)
    ioctl(fd, I2C_SLAVE, self.I2C_ADDR)
    try:
      posix.write(fd, b'\0x00')  # wake up AM2320
    except:
      pass
    time.sleep(0.01)
    posix.write(fd, b'\x03\x00\x04') # x03(function code) x00(starting address) x04(register length)
    time.sleep(0.002)
    data = bytearray(posix.read(fd, 8))
    posix.close(fd)

    if data[0] != 0x03 or data[1] != 0x04:  # Modbus function code (0x03), number of registers to read (0x04)
      raise Exception('I2C read failure')
    if self._calc_crc16(data[0:6]) != self._combine_bytes(data[7], data[6]):
      raise Exception('CRC failure')

    humi = self._combine_bytes(data[2], data[3]) / 10.0
    temp = self._combine_bytes(data[4], data[5])
    if temp & 0x8000:
      temp = -(temp & 0x7FFF)
    temp /= 10.0
    return (temp, humi)


# TSL2572 - Ambient Light Sensor
#   60000 Lx
# http://akizukidenshi.com/catalog/g/gK-15536/
# http://akizukidenshi.com/download/ds/ams/tsl2572.pdf
# http://akizukidenshi.com/download/ds/akizuki/AE_TSL25721_demo.zip
# https://learn.tinycircuits.com/Wirelings/Ambient-Light_Wireling_Tutorial/
class TSL2572:
  I2C_ADDR = 0x39
  atime = 0xED  # 0xED:1.87ms   #0xC0
  gain = 1  # 1, 8, 16, 120

  COMMAND  = 0x80
  TYPE_REP = 0x00
  TYPE_INC = 0x20
  ALSIFC   = 0x66
  RESISTER = {
    'ENABLE': 0x00, 'ATIME': 0x01, 'WTIME': 0x03,
    'AILTL': 0x04, 'AILTH': 0x05, 'AIHTL': 0x06, 'AIHTH': 0x07,
    'PRES': 0x0C, 'CONFIG': 0x0D, 'CONTROL': 0x0F,
    'ID': 0x12, 'STATUS': 0x13,
    'C0DATA': 0x14, 'C0DATAH': 0x15, 'C1DATA': 0x16, 'C1DATAH': 0x17
  }
  ENABLE = {'PON': 0x01, 'AEN': 0x02, 'WEN': 0x80, 'AIEN': 0x10, 'SAI': 0x40}
  ATIME = {1: 0xFF, 10: 0xF6, 37: 0xDB, 64: 0xC0, 256: 0x00}  # ALS Time
  # ATIME = 256 - Integration Time / 2.73 ms
  # Integration Time = 2.73 ms * (256 - ATIME)
  WTIME = {1: 0xFF, 74: 0xB6, 256: 0x00}  # Wait Time
  AGAIN = {1: 0x00, 8: 0x01, 16: 0x10, 120: 0x11}

  def __init__(self, i2cbus = 1):
    self._i2cbus = i2cbus

  def __enter__(self):
    self._i2c = SMBus(self._i2cbus)
    if(self._i2c.read_i2c_block_data(self.I2C_ADDR, self.COMMAND | self.TYPE_INC | self.RESISTER['ID'], 1) != [0x34]):  # 0x34: TSL25721, TSL25725
      self._i2c.close()
      raise Exception('I2C read failure')
    if(self.gain == 1 or self.gain == 8):
      self._i2c.write_byte_data(self.I2C_ADDR, self.COMMAND | self.TYPE_INC | self.RESISTER['CONFIG'], 0x04)  # scale gain by 0.16
    else:
      self._i2c.write_byte_data(self.I2C_ADDR, self.COMMAND | self.TYPE_INC | self.RESISTER['CONFIG'], 0x00)
    self._i2c.write_byte_data(self.I2C_ADDR, self.COMMAND | self.TYPE_INC | self.RESISTER['CONTROL'], self.AGAIN[self.gain])
    self._i2c.write_byte_data(self.I2C_ADDR, self.COMMAND | self.TYPE_INC | self.RESISTER['ATIME'], self.atime)
    self._i2c.write_byte_data(self.I2C_ADDR, self.COMMAND | self.TYPE_INC | self.RESISTER['ENABLE'], self.ENABLE['AEN'] | self.ENABLE['PON'])
    time.sleep(0.2)  # センサーの起動待ち
    return self

  def __exit__(self, exc_type, exc_value, exc_tb):
    self._i2c.close()
    # TODO センサーをスリープさせる

  def read_sensor(self):
    dat = self._i2c.read_i2c_block_data(self.I2C_ADDR, self.COMMAND | self.TYPE_INC | self.RESISTER['C0DATA'], 4)
    (adc0, adc1) = ((dat[1] << 8) | dat[0], (dat[3] << 8) | dat[2])
    cpl = (2.73 * (256 - self.atime)) * self.gain / 60.0
    if(self.gain == 1 or self.gain == 8):
      cpl = cpl / 6.0
    lux1 = ((adc0 * 1.00) - (adc1 * 1.87)) / cpl
    lux2 = ((adc0 * 0.63) - (adc1 * 1.00)) / cpl
    time.sleep(0.2)
    return max(lux1, lux2, 0)


# https://gist.github.com/josephernest/77fdb0012b72ebdf4c9d19d6256a1119
class Daemon:
  def __init__(self, pidfile='_.pid', stdin='/dev/null', stdout='/dev/null', stderr='/dev/null'):
    self.stdin = stdin
    self.stdout = stdout
    self.stderr = stderr
    self.pidfile = pidfile

  def daemonize(self):
    try:
      pid = os.fork()
      if pid > 0:
        sys.exit(0)
    except OSError as e:
      sys.stderr.write('fork#1 failed: %d (%s)%s' % (e.errno, e.strerror, os.linesep))
      sys.exit(1)
    os.setsid()
    os.umask(0)
    try:
      pid = os.fork()
      if pid > 0:
        sys.exit(0)
    except OSError as e:
      sys.stderr.write('fork#2 failed: %d (%s)%s' % (e.errno, e.strerror, os.linesep))
      sys.exit(1)
    sys.stdout.flush()
    sys.stderr.flush()
    stdin = open(os.devnull, 'r')
    stdout = open(os.devnull, 'a+')
    stderr = open(os.devnull, 'a+', 0)
    os.dup2(stdin.fileno(), sys.stdin.fileno())
    os.dup2(stdout.fileno(), sys.stdout.fileno())
    os.dup2(stderr.fileno(), sys.stderr.fileno())
    atexit.register(self._onstop)
    signal.signal(signal.SIGTERM, lambda signum, stack_frame: exit())
    pid = str(os.getpid())
    open(self.pidfile, 'w+').write('%s\n' % pid)

  def _onstop(self):
    self.quit()
    os.remove(self.pidfile)

  def start(self):
    try:
      pf = open(self.pidfile, 'r')
      pid = int(pf.read().strip())
      pf.close()
    except IOError:
      pid = None
    if pid:
      sys.stderr.write('pidfile %s already exist%s' % (self.pidfile, os.linesep))
      sys.exit(1)
    self.daemonize()
    self.run()

  def stop(self):
    try:
      pf = open(self.pidfile, 'r')
      pid = int(pf.read().strip())
      pf.close()
    except IOError:
      pid = None
    if not pid:
      sys.stderr.write('pidfile %s does not exist%s' % (self.pidfile, os.linesep))
      return
    try:
      os.kill(pid, signal.SIGTERM)
    except OSError as err:
      err = str(err)
      if err.find('No such process') > 0:
        if os.path.exists(self.pidfile):
          os.remove(self.pidfile)
      else:
        print(str(err))
        sys.exit(1)

  def restart(self):
    self.stop()
    # FIXME: add wait? sleep? wait os.remove() to complete
    self.start()

  def run(self):
    pass  # override this method

  def quit(self):
    pass  # override this method


def get_sensors():
  dt_now = datetime.datetime.now()
  dt_date = dt_now.strftime('%Y%m%d')
  dt_time = dt_now.strftime('%H%M%S')

  am2320 = AM2320(1)
  am2320.read_sensor()  # スリープから復帰させる、データは過去の値なので捨てる
  time.sleep(2)  # 復帰待ち
  (t, h) = am2320.read_sensor()

  lx = -1
  with TSL2572(1) as tsl25721:
    lx = tsl25721.read_sensor()

  return {'date': dt_date, 'time': dt_time, 'temp': t, 'humi': h, 'lux': lx}

def print_sensors():
  v = get_sensors()
  print(f'Sensors [{v["date"]} {v["time"]}]')
  print(f'  AM2320:   {v["temp"]:.1f}C  {v["humi"]:.1f}%RH')
  print(f'  TSL25721: {v["lux"]:.1f} Lx')

def api_server(environ, start_response):
  v = get_sensors()
  status = '200 OK'
  headers = [('Content-type', 'text/plain; charset=utf-8')]
  start_response(status, headers)
  # Date(Ymd),Time(HMS),Temperature(C),Humidity(%RH),Light(Lx)
  #   res=$(curl -sS http://[SERVER_ADDRESS]:[LISTEN_PORT])
  #   list=(${res//,/ })
  return [(f'{v["date"]},{v["time"]},{v["temp"]:.1f},{v["humi"]:.1f},{v["lux"]:.1f}').encode('utf-8')]


if __name__ == '__main__':
  # disable wsgiref.simple_server output
  class SilentWSGIRequestHandler(WSGIRequestHandler):
    def log_message(self, format, *args):
      pass

  # daemonize
  class sensorDaemon(Daemon):
    def run(self):
      self.httpd = make_server('', LISTEN_PORT, api_server, handler_class=SilentWSGIRequestHandler)
      print(f'Serving on port {LISTEN_PORT}...')
      try:
        self.httpd.serve_forever()
      except KeyboardInterrupt:
        pass
      finally:
        self.httpd.shutdown()
    def quit(self):
      self.httpd.shutdown()
  daemon = sensorDaemon(pidfile=PIDFILE)

  parser = argparse.ArgumentParser(description='read i2c sensors')
  parser.add_argument('-s', '--server', action='store_true', help='Start server (not daemon)')
  parser.add_argument('--start', action='store_true', help='Start server daemon')
  parser.add_argument('--stop', action='store_true', help='Stop server daemon')
  parser.add_argument('--restart', action='store_true', help='Restart server daemon')
  args = parser.parse_args()

  if(args.server):
    httpd = make_server('', LISTEN_PORT, api_server, handler_class=SilentWSGIRequestHandler)
    print(f'Serving on port {LISTEN_PORT}...')
    try:
      httpd.serve_forever()
    except KeyboardInterrupt:
      pass
    finally:
      httpd.shutdown()
      print(f'Server shutdown')
  elif(args.start):
    print(f'Daemon start')
    daemon.start()
  elif(args.stop):
    print(f'Daemon stop')
    daemon.stop()
  elif(args.restart):
    print(f'Daemon restart')
    daemon.restart()
  else:
    print_sensors()
