import time
from enum import StrEnum

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

    def __init__(self, sensor_type: SensorType, name: str, address: int, min_delay: float = 1):
        super().__init__(SensorBrand.Atlas, sensor_type, name, address, min_delay)

    def init(self, bus: smbus.SMBus):
        self.calibrate(bus)

    def calibrate(self, bus: smbus.SMBus):
        self._query(bus, self.Commands.CAL)

    def measure_value(self, bus: smbus.SMBus):
        print("Measuring atlas x...")
        return self._query(bus, self.Commands.READ)

    ###############################################################
    # ADAPTED FROM ATLAS's CODE
    def handle_raspi_glitch(self, response):  # Maybe useless when using smbus ?
        '''
        Change MSB to 0 for all received characters except the first
        and get a list of characters
        NOTE: having to change the MSB to 0 is a glitch in the raspberry pi,
        and you shouldn't have to do this!
        '''
        return list(map(lambda x: chr(x & ~0x80), list(response)))

    def response_valid(self, response):
        valid = True
        error_code = None
        if len(response) > 0:
            error_code = str(response[0])

            if error_code != '1':  # 1:
                valid = False

        return valid, error_code

    def get_device_info(self):  # Useless ?
        return str(self.addr) + " " + self.name

    def _read(self, bus: smbus.SMBus, num_of_bytes=31):
        '''
        reads a specified number of bytes from I2C, then parses and displays the result
        '''
        # bus.write_byte(self.addr, ord(self.Commands.READ))
        response = []
        for i in range(num_of_bytes):
            response.append(bus.read_byte(self.addr))
        # response = bus.read_i2c_block_data(self.addr, self.DEFAULT_REG, num_of_bytes)
        print(response)
        is_valid, error_code = self.response_valid(response=response)

        if is_valid:
            char_list = self.handle_raspi_glitch(response[1:])
            result = "Success " + self.get_device_info() + ": " + str(''.join(char_list))
            # result = "Success: " +  str(''.join(char_list))
        else:
            result = "Error " + self.get_device_info() + ": " + error_code

        return result

    def _write(self, bus: smbus.SMBus, cmd: Commands):
        bus.write_byte(self.addr, ord(self.Commands.READ))
        # bus.write_i2c_block_data(i2c_addr=self.addr, register=self.DEFAULT_REG, data=[ord(c) for c in cmd])

    def _get_command_timeout(self, command):
        timeout = None
        if command.upper().startswith(self.LONG_TIMEOUT_COMMANDS):
            timeout = self.LONG_TIMEOUT
        elif not command.upper().startswith(self.SLEEP_COMMANDS):
            timeout = self.SHORT_TIMEOUT

        return timeout

    def _query(self, bus: smbus.SMBus, command):
        """
        write a command to the board, wait the correct timeout,
        and read the response
        """
        self._write(bus, command)
        # current_timeout = self._get_command_timeout(command=command)
        # if not current_timeout:
        #     return "sleep mode"
        # else:
        #     time.sleep(current_timeout)
        return self._read(bus=bus)

    # def list_i2c_devices(self):  # Useless ?
    #     '''
    #     save the current address so we can restore it after
    #     '''
    #     prev_addr = copy.deepcopy(self._address)
    #     i2c_devices = []
    #     for i in range(0, 128):
    #         try:
    #             self.set_i2c_address(i)
    #             self.read(1)
    #             i2c_devices.append(i)
    #         except IOError:
    #             pass
    #     # restore the address we were using
    #     self.set_i2c_address(prev_addr)
    #
    #     return i2c_devices
