# Minimal Arducam OV2640 driver for STELLA-Q2 / SparkFun Thing Plus RP2040
# Purpose-built for the tested 320x240 JPEG configuration.
# Removes OV5642 support, alternate resolutions, image effects, and large list tables.
# Raw bytes literals avoid the large temporary list expression that can exhaust CircuitPython pystack.

import board
import busio
import bitbangio
import digitalio
import time as utime

OV2640 = 0
JPEG = 1
OV2640_320x240 = 2
MAX_FIFO_SIZE = 0x7FFFFF
ARDUCHIP_TRIG = 0x41
CAP_DONE_MASK = 0x08

# Register tables are packed as bytes: addr, value, addr, value, ...
# They are copied exactly from the user's known-working OV2640_reg.py tables.
_OV2640_JPEG_INIT = (
    b"\xff\x00\x2c\xff\x2e\xdf\xff\x01\x3c\x32\x11\x00\x09\x02\x04\x28\x13\xe5\x14\x48\x2c\x0c\x33\x78\x3a\x33\x3b\xfb\x3e\x00\x43\x11"
    b"\x16\x10\x39\x92\x35\xda\x22\x1a\x37\xc3\x23\x00\x34\xc0\x36\x1a\x06\x88\x07\xc0\x0d\x87\x0e\x41\x4c\x00\x48\x00\x5b\x00\x42\x03"
    b"\x4a\x81\x21\x99\x24\x40\x25\x38\x26\x82\x5c\x00\x63\x00\x61\x70\x62\x80\x7c\x05\x20\x80\x28\x30\x6c\x00\x6d\x80\x6e\x00\x70\x02"
    b"\x71\x94\x73\xc1\x12\x40\x17\x11\x18\x43\x19\x00\x1a\x4b\x32\x09\x37\xc0\x4f\x60\x50\xa8\x6d\x00\x3d\x38\x46\x3f\x4f\x60\x0c\x3c"
    b"\xff\x00\xe5\x7f\xf9\xc0\x41\x24\xe0\x14\x76\xff\x33\xa0\x42\x20\x43\x18\x4c\x00\x87\xd5\x88\x3f\xd7\x03\xd9\x10\xd3\x82\xc8\x08"
    b"\xc9\x80\x7c\x00\x7d\x00\x7c\x03\x7d\x48\x7d\x48\x7c\x08\x7d\x20\x7d\x10\x7d\x0e\x90\x00\x91\x0e\x91\x1a\x91\x31\x91\x5a\x91\x69"
    b"\x91\x75\x91\x7e\x91\x88\x91\x8f\x91\x96\x91\xa3\x91\xaf\x91\xc4\x91\xd7\x91\xe8\x91\x20\x92\x00\x93\x06\x93\xe3\x93\x05\x93\x05"
    b"\x93\x00\x93\x04\x93\x00\x93\x00\x93\x00\x93\x00\x93\x00\x93\x00\x93\x00\x96\x00\x97\x08\x97\x19\x97\x02\x97\x0c\x97\x24\x97\x30"
    b"\x97\x28\x97\x26\x97\x02\x97\x98\x97\x80\x97\x00\x97\x00\xc3\xed\xa4\x00\xa8\x00\xc5\x11\xc6\x51\xbf\x80\xc7\x10\xb6\x66\xb8\xa5"
    b"\xb7\x64\xb9\x7c\xb3\xaf\xb4\x97\xb5\xff\xb0\xc5\xb1\x94\xb2\x0f\xc4\x5c\xc0\x64\xc1\x4b\x8c\x00\x86\x3d\x50\x00\x51\xc8\x52\x96"
    b"\x53\x00\x54\x00\x55\x00\x5a\xc8\x5b\x96\x5c\x00\xd3\x00\xc3\xed\x7f\x00\xda\x00\xe5\x1f\xe1\x67\xe0\x00\xdd\x7f\x05\x00\x12\x40"
    b"\xd3\x04\xc0\x16\xc1\x12\x8c\x00\x86\x3d\x50\x00\x51\x2c\x52\x24\x53\x00\x54\x00\x55\x00\x5a\x2c\x5b\x24\x5c\x00"
)
_OV2640_YUV422 = (
    b"\xff\x00\x05\x00\xda\x10\xd7\x03\xdf\x00\x33\x80\x3c\x40\xe1\x77\x00\x00"
)
_OV2640_JPEG = (
    b"\xe0\x14\xe1\x77\xe5\x1f\xd7\x03\xda\x10\xe0\x00\xff\x01\x04\x08"
)
_OV2640_320x240_JPEG = (
    b"\xff\x01\x12\x40\x17\x11\x18\x43\x19\x00\x1a\x4b\x32\x09\x4f\xca\x50\xa8\x5a\x23\x6d\x00\x39\x12\x35\xda\x22\x1a\x37\xc3\x23\x00"
    b"\x34\xc0\x36\x1a\x06\x88\x07\xc0\x0d\x87\x0e\x41\x4c\x00\xff\x00\xe0\x04\xc0\x64\xc1\x4b\x86\x35\x50\x89\x51\xc8\x52\x96\x53\x00"
    b"\x54\x00\x55\x00\x57\x00\x5a\x50\x5b\x3c\x5c\x00\xe0\x00"
)


class ArducamClass:
    def __init__(self, camera_type=OV2640):
        if camera_type != OV2640:
            raise ValueError("Arducam_Q2 supports OV2640 only")

        self.CameraType = OV2640
        self.CameraMode = JPEG
        self.I2cAddress = 0x30

        # Known-working camera wiring on SparkFun Thing Plus RP2040.
        self.SPI_CS = digitalio.DigitalInOut(board.D17)
        self.SPI_CS.direction = digitalio.Direction.OUTPUT
        self.SPI_CS.value = True

        self.spi = busio.SPI(clock=board.D18, MOSI=board.D19, MISO=board.D16)
        while not self.spi.try_lock():
            pass
        self.spi.configure(baudrate=4000000, polarity=0, phase=0, bits=8)

        self.i2c = bitbangio.I2C(scl=board.D21, sda=board.D20, frequency=1000000)
        while not self.i2c.try_lock():
            pass

        # Reset ArduChip controller.
        self.Spi_write(0x07, 0x80)
        utime.sleep(0.1)
        self.Spi_write(0x07, 0x00)
        utime.sleep(0.1)

    def wrSensorReg8_8(self, addr, val):
        buf = bytearray((addr & 0xFF, val & 0xFF))
        self.i2c.writeto(self.I2cAddress, buf)

    def rdSensorReg8_8(self, addr):
        buf = bytearray((addr & 0xFF,))
        self.i2c.writeto(self.I2cAddress, buf)
        self.i2c.readfrom_into(self.I2cAddress, buf)
        return buf[0]

    def _write_table(self, table):
        # Packed pairs avoid the much larger nested-list objects in the stock library.
        i = 0
        n = len(table)
        while i < n:
            self.wrSensorReg8_8(table[i], table[i + 1])
            i += 2
            utime.sleep(0.001)

    def set_format(self, mode):
        if mode != JPEG:
            raise ValueError("Arducam_Q2 supports JPEG only")
        self.CameraMode = JPEG

    def Camera_Init(self):
        self.wrSensorReg8_8(0xFF, 0x01)
        self.wrSensorReg8_8(0x12, 0x80)
        utime.sleep(0.1)
        self._write_table(_OV2640_JPEG_INIT)
        self._write_table(_OV2640_YUV422)
        self._write_table(_OV2640_JPEG)
        self.wrSensorReg8_8(0xFF, 0x01)
        self.wrSensorReg8_8(0x15, 0x00)
        self._write_table(_OV2640_320x240_JPEG)

    def OV2640_set_JPEG_size(self, size):
        if size != OV2640_320x240:
            raise ValueError("Arducam_Q2 supports 320x240 only")
        self._write_table(_OV2640_320x240_JPEG)

    def Spi_write(self, address, value):
        buf = bytearray((address | 0x80, value & 0xFF))
        self.SPI_CS_LOW()
        self.spi.write(buf)
        self.SPI_CS_HIGH()

    def Spi_read(self, address):
        buf = bytearray((address & 0x7F,))
        self.SPI_CS_LOW()
        self.spi.write(buf)
        self.spi.readinto(buf)
        self.SPI_CS_HIGH()
        return buf

    def get_bit(self, addr, bit):
        return self.Spi_read(addr)[0] & bit

    def SPI_CS_LOW(self):
        self.SPI_CS.value = False

    def SPI_CS_HIGH(self):
        self.SPI_CS.value = True

    def set_fifo_burst(self):
        self.spi.write(bytes((0x3C,)))

    def clear_fifo_flag(self):
        self.Spi_write(0x04, 0x01)

    def flush_fifo(self):
        self.Spi_write(0x04, 0x01)

    def start_capture(self):
        self.Spi_write(0x04, 0x02)

    def read_fifo_length(self):
        len1 = self.Spi_read(0x42)[0]
        len2 = self.Spi_read(0x43)[0]
        len3 = self.Spi_read(0x44)[0] & 0x7F
        return ((len3 << 16) | (len2 << 8) | len1) & MAX_FIFO_SIZE
