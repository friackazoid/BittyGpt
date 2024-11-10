#!/usr/bin/env python3
# -*- coding: UTF-8 -*-

import serial
import serial.tools.list_ports


class Communication:
    """
    A class for handling serial communication using pySerial.
    """

    def __init__(self, port, baudrate=115200, timeout=0.5):
        """
        Initialize the serial communication.

        :param port: Serial port name or number.
        :param baudrate: Baud rate for the serial communication.
        :param timeout: Read timeout value.
        """
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.serial_engine = None
        self.is_open = False

        try:
            # Open the serial port
            self.serial_engine = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=self.timeout
            )
            self.is_open = self.serial_engine.is_open
        except Exception as e:
            print('Exception occurred while opening serial port {self.port} :', e)
            raise

    def __enter__(self):
        """
        Enter the runtime context related to this object.
        """
        if not self.is_open:
            self.open_engine()
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        """
        Exit the runtime context and close the serial port.
        """
        self.close_engine()

    def print_device_info(self):
        """
        Print basic information about the serial device.
        """
        if self.serial_engine is None:
            print('Serial port is not open.')
            return

        print(f'Device Name: {self.serial_engine.name}')
        print(f'Port: {self.serial_engine.port}')
        print(f'Baudrate: {self.serial_engine.baudrate}')
        print(f'Byte Size: {self.serial_engine.bytesize}')
        print(f'Parity: {self.serial_engine.parity}')
        print(f'Stop Bits: {self.serial_engine.stopbits}')
        print(f'Timeout: {self.serial_engine.timeout}')
        print(f'Write Timeout: {self.serial_engine.writeTimeout}')
        print(f'XON/XOFF: {self.serial_engine.xonxoff}')
        print(f'RTS/CTS: {self.serial_engine.rtscts}')
        print(f'DSR/DTR: {self.serial_engine.dsrdtr}')
        print(f'Inter-character Timeout: {self.serial_engine.interCharTimeout}')

    def open_engine(self):
        """
        Open the serial port if it's not already open.
        """
        if not self.serial_engine.is_open:
            self.serial_engine.open()
            self.is_open = True

    def close_engine(self):
        """
        Close the serial port if it's open.
        """
        if self.serial_engine.is_open:
            self.serial_engine.close()
            self.is_open = False

    @staticmethod
    def list_available_ports():
        """
        List all available serial ports.

        :return: List of tuples containing port device and name.
        """
        ports = serial.tools.list_ports.comports()
        return [(port.device, port.name) for port in ports]

    def read_size(self, size):
        """
        Read a specific number of bytes from the serial port.

        :param size: Number of bytes to read.
        :return: Bytes read from the serial port.
        """
        return self.serial_engine.read(size=size)

    def read_line(self):
        """
        Read a line from the serial port.

        :return: Line read from the serial port.
        """
        return self.serial_engine.readline()

    def send_data(self, data):
        """
        Send data over the serial port.

        :param data: Data to send (bytes).
        """
        if not self.is_open:
            print('Serial port is not open.')
            return
        self.serial_engine.write(data)

    def receive_data(self, mode=1):
        """
        Receive data from the serial port.

        :param mode: Mode of receiving data.
                     0 - Read bytes individually.
                     1 - Read all available bytes.
        """
        print('Start receiving data:')
        try:
            while True:
                if self.serial_engine.in_waiting:
                    if mode == 0:
                        for _ in range(self.serial_engine.in_waiting):
                            data = self.read_size(1)
                            print('Received ASCII data:', data)
                    elif mode == 1:
                        data = self.serial_engine.read(
                            self.serial_engine.in_waiting
                        ).decode("utf-8")
                        print('Received ASCII data:', data)
                        if not data.strip():
                            break
        except Exception as e:
            print('Error while receiving data:', e)
        print('Data reception completed.')


if __name__ == '__main__':
    available_ports = Communication.list_available_ports()
    if available_ports:
        port_name = available_ports[0][0]  # Select the first available port
        with Communication(port_name) as communication:
            print('Is port open:', communication.is_open)
            communication.print_device_info()
            communication.receive_data(mode=1)
    else:
        print('No available serial ports found.')
