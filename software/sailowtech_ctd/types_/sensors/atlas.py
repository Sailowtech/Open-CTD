import copy
import sys
import time
from enum import StrEnum
import fcntl

import smbus2 as smbus

from software.sailowtech_ctd.types_.sensors.generic import GenericSensor, SensorType, SensorBrand


class AtlasSensor(GenericSensor):
    class Commands(StrEnum):
        READ = "R"
        CAL = "CAL"
        SLEEP = "SLEEP"

    LONG_TIMEOUT_COMMANDS = (Commands.READ, Commands.CAL)
    SLEEP_COMMANDS = (Commands.SLEEP,)

    # the timeout needed to query readings and calibrations
    LONG_TIMEOUT = 1.5
    # timeout for regular commands
    SHORT_TIMEOUT = .3

    DEFAULT_REG = 0x00  # Maybe wrong ???

    def __init__(self, sensor_type: SensorType, name: str, address: int, moduletype="", min_delay: float = 1):
        super().__init__(SensorBrand.Atlas, sensor_type, name, address, min_delay)

        self._long_timeout = self.LONG_TIMEOUT
        self._short_timeout = self.SHORT_TIMEOUT
        self._module = moduletype

    def init(self, fr, fw):
        self.calibrate(fr, fw)

    def calibrate(self, fr, fw):
        self.query(self.Commands.CAL, fr, fw)

    def measure_value(self, fr, fw):
        print("Measuring atlas x...")
        return self.query(self.Commands.READ, fr, fw)

    # #############################################
    def set_i2c_address(self, addr, fr, fw):
        '''
        set the I2C communications to the slave specified by the address
        the commands for I2C dev using the ioctl functions are specified in
        the i2c-dev.h file from i2c-tools
        '''
        I2C_SLAVE = 0x703
        fcntl.ioctl(fr, I2C_SLAVE, addr)
        fcntl.ioctl(fw, I2C_SLAVE, addr)
        self.addr = addr

    def write(self, cmd, fw):
        '''
        appends the null character and sends the string over I2C
        '''
        cmd += "\00"
        fw.write(cmd.encode('latin-1'))

    def handle_raspi_glitch(self, response):
        '''
        Change MSB to 0 for all received characters except the first
        and get a list of characters
        NOTE: having to change the MSB to 0 is a glitch in the raspberry pi,
        and you shouldn't have to do this!
        '''
        if self.app_using_python_two():
            return list(map(lambda x: chr(ord(x) & ~0x80), list(response)))
        else:
            return list(map(lambda x: chr(x & ~0x80), list(response)))

    def app_using_python_two(self):
        return sys.version_info[0] < 3

    def get_response(self, raw_data):
        if self.app_using_python_two():
            response = [i for i in raw_data if i != '\x00']
        else:
            response = raw_data

        return response

    def response_valid(self, response):
        valid = True
        error_code = None
        if (len(response) > 0):

            if self.app_using_python_two():
                error_code = str(ord(response[0]))
            else:
                error_code = str(response[0])

            if error_code != '1':  # 1:
                valid = False

        return valid, error_code

    def get_device_info(self):
        if self.name == "":
            return self._module + " " + str(self.addr)
        else:
            return self._module + " " + str(self.addr) + " " + self.name

    def read(self, fr, num_of_bytes=31):
        '''
        reads a specified number of bytes from I2C, then parses and displays the result
        '''

        raw_data = fr.read(num_of_bytes)
        response = self.get_response(raw_data=raw_data)
        # print(response)
        is_valid, error_code = self.response_valid(response=response)

        if is_valid:
            char_list = self.handle_raspi_glitch(response[1:])
            result = "Success " + self.get_device_info() + ": " + str(''.join(char_list))
            # result = "Success: " +  str(''.join(char_list))
        else:
            result = "Error " + self.get_device_info() + ": " + error_code

        return result

    def get_command_timeout(self, command):
        timeout = None
        if command.upper().startswith(self.LONG_TIMEOUT_COMMANDS):
            timeout = self._long_timeout
        elif not command.upper().startswith(self.SLEEP_COMMANDS):
            timeout = self._short_timeout

        return timeout

    def query(self, command, fr, fw):
        '''
        write a command to the board, wait the correct timeout,
        and read the response
        '''
        self.write(command, fw)
        current_timeout = self.get_command_timeout(command=command)
        if not current_timeout:
            return "sleep mode"
        else:
            time.sleep(current_timeout)
            return self.read(fr)

    def close(self, fr, fw):
        fr.close()
        fw.close()

    def list_i2c_devices(self, fr, fw):
        '''
        save the current address so we can restore it after
        '''
        prev_addr = copy.deepcopy(self.addr)
        i2c_devices = []
        for i in range(0, 128):
            try:
                self.set_i2c_address(i, fr, fw)
                self.read(fr, 1)
                i2c_devices.append(i)
            except IOError:
                pass
        # restore the address we were using
        self.set_i2c_address(prev_addr, fr, fw)

        return i2c_devices
