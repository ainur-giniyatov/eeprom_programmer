import os
import termios
import time

from cmd import Cmd
from time import sleep

from serial import Serial
from tqdm import tqdm

# Map from the numbers to the termios constants (which are pretty much
# the same numbers).

BPS_SYMS = {
    4800: termios.B4800,
    9600: termios.B9600,
    19200: termios.B19200,
    38400: termios.B38400,
    57600: termios.B57600,
    115200: termios.B115200
}


# Indices into the termios tuple.

IFLAG = 0
OFLAG = 1
CFLAG = 2
LFLAG = 3
ISPEED = 4
OSPEED = 5
CC = 6


def bps_to_termios_sym(bps):
    return BPS_SYMS[bps]


class SerialPort(object):
    """Represents a serial port connected to an Arduino."""
    def __init__(self, serialport, bps):
        """Takes the string name of the serial port (e.g.
        "/dev/tty.usbserial","COM1") and a baud rate (bps) and connects to
        that port at that speed and 8N1. Opens the port in fully raw mode
        so you can send binary data.
        """
        self.fd = os.open(serialport, os.O_RDWR | os.O_NOCTTY | os.O_NDELAY)
        attrs = termios.tcgetattr(self.fd)
        bps_sym = bps_to_termios_sym(bps)
        # Set I/O speed.
        attrs[ISPEED] = bps_sym
        attrs[OSPEED] = bps_sym

        # 8N1
        attrs[CFLAG] &= ~termios.PARENB
        attrs[CFLAG] &= ~termios.CSTOPB
        attrs[CFLAG] &= ~termios.CSIZE
        attrs[CFLAG] |= termios.CS8
        # No flow control
        attrs[CFLAG] &= ~termios.CRTSCTS

        # Turn on READ & ignore contrll lines.
        attrs[CFLAG] |= termios.CREAD | termios.CLOCAL
        # Turn off software flow control.
        attrs[IFLAG] &= ~(termios.IXON | termios.IXOFF | termios.IXANY)

        # Make raw.
        attrs[LFLAG] &= ~(termios.ICANON |
                          termios.ECHO |
                          termios.ECHOE |
                          termios.ISIG)
        attrs[OFLAG] &= ~termios.OPOST

        # It's complicated--See
        # http://unixwiz.net/techtips/termios-vmin-vtime.html
        attrs[CC][termios.VMIN] = 0
        attrs[CC][termios.VTIME] = 20
        termios.tcsetattr(self.fd, termios.TCSANOW, attrs)

    def read_until(self, until):
        buf = b""
        done = False
        while not done:
            n = os.read(self.fd, 1)
            if n == '':
                # FIXME: Maybe worth blocking instead of busy-looping?
                time.sleep(0.01)
                continue
            buf = buf + n
            if n == until:
                done = True
        return buf

    def read(self, size=1):
        buf = b""
        n = os.read(self.fd, size)

        return n
    def write(self, str):
        os.write(self.fd, str)

    def write_byte(self, byte):
        os.write(self.fd, chr(byte))


class ProgCmd(Cmd):
    def __init__(self):
        super().__init__()
        self.__buffer = [0] * 0x10000
        self.__serial: SerialPort | None = None
        self.__rt = False

    def do_quit(self, arg):
        self.do_disconnect(None)
        return True

    def do_erase(self, arg):
        if not self.__serial:
            print('not connected')
            return

        while True:
            ans = input('14 volts provided?(yes|no):')
            if ans in ('yes', 'no'):
                break
        if ans == 'yes':
            self.__serial.write('e\r\n\0'.encode())
            sleep(0.5)
            resp = self.__serial.read_until(b'\n')
            if resp == b'ok\n':
                print('erase sequence was performed')
        else:
            print('erase cancelled')

    def do_load(self, arg):
        file_path = arg
        if not os.path.exists(file_path):
            print(f'file {file_path} does not exist')
            return

        with open(file_path, 'rb') as fp:
            data = fp.read(0x10000)
        self.__buffer = [b for b in data]
        print(f'{len(data)} bytes were read')

    def do_save(self, arg):
        file_path = arg
        if os.path.exists(file_path):
            print(f'file {file_path} already exists')
            return

        with open(file_path, 'wb') as fp:
            fp.write(bytes(self.__buffer))
        print(f'{len(self.__buffer)} bytes were written')

    def do_read(self, arg):
        if not self.__serial:
            print('not connected')
            return

        read_address = 0x0000
        read_len = 0x20
        end_address = 0x10000
        tq = tqdm(total=end_address)
        while read_address < end_address:
            # print(f'r {read_address:0=4x} {read_len:0=4x}')
            # self.__serial.reset_input_buffer()
            # self.__serial.reset_output_buffer()
            self.__serial.write(f'r {read_address:0=4x} {read_len:0=4x}\r'.encode())
            sleep(0.05)
            resp = self.__serial.read(read_len)
            sleep(0.01)
            if not resp:
                print('timeout')
                break
            # print(resp.hex(' '))
            for r in range(read_len):
                self.__buffer[read_address + r] = resp[r]
            read_address += read_len
            tq.update(read_len)

    def do_write(self, arg):
        if not self.__serial:
            print('not connected')
            return

        while True:
            ans = input('12 volts provided?(yes|no):')
            if ans in ('yes', 'no'):
                break

        if ans == 'no':
            print('write cancelled')
            return

        read_address = 0x0000
        read_len = 0x10
        end_address = 0x10000
        tq = tqdm(total=end_address)
        while read_address < end_address:
            sb = bytes(self.__buffer[read_address: read_address + read_len]).hex(' ')
            g = f'w {read_address:0=4x} {read_len:0=4x} {sb}\r'

            self.__serial.write(g.encode())

            sleep(0.05)
            resp = self.__serial.read_until(b'\n')

            if resp != b'wok\n':
                print('timeout')
                break
            read_address += read_len
            tq.update(read_len)

    def do_verify(self, arg):
        if not self.__serial:
            print('not connected')
            return

        read_address = 0x0000
        read_len = 0x20
        end_address = 0x10000
        tq = tqdm(total=end_address)
        br = False
        while read_address < end_address:
            # print(f'r {read_address:0=4x} {read_len:0=4x}')
            # self.__serial.reset_input_buffer()
            # self.__serial.reset_output_buffer()
            self.__serial.write(f'r {read_address:0=4x} {read_len:0=4x}\r'.encode())
            sleep(0.05)
            resp = self.__serial.read(read_len)
            sleep(0.01)
            if not resp:
                print('timeout')
                break
            # print(resp.hex(' '))

            for r in range(read_len):
                if self.__buffer[read_address + r] != resp[r]:
                    print(f'mismatch at {(read_address + r):0=4x}')
                    br = True
                    break
            read_address += read_len
            tq.update(read_len)
            if br:
                break
        if not br:
            print('verify ok')

    def do_view(self, arg):
        d = 0x0f
        for i in range(0x10000 // d):
            print(bytes(self.__buffer[i * d : (i + 1) * d]).hex(' '))

    def do_connect(self, arg):
        device_file = arg
        self.do_disconnect(None)

        try:
            self.__serial = SerialPort(device_file, 57600)  #
            # self.__serial = serial.serial_for_url(device_file, 57600, do_not_open=True)

        except Exception as e:
            print(e)
        sleep(2)
        self.__serial.write('?\r\n\0'.encode())
        sleep(0.01)
        resp = self.__serial.read_until(b'\n')
        if resp == b'ok\n':
            print('hardware connected')
        else:
            print('hardware not found')
            self.do_disconnect(None)

    def do_disconnect(self, arg):
        if self.__serial:
            os.close(self.__serial.fd)
            self.__serial = None


if __name__ == '__main__':
    cmd = ProgCmd()
    cmd.do_connect('/dev/ttyUSB0')
    # cmd.do_read(None)
    cmd.cmdloop()