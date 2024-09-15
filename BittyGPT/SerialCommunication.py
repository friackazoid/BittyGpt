#!/usr/bin/python3
# -*- coding: UTF-8 -*-

# modified from https://blog.csdn.net/u013541325/article/details/113062191

import serial  # need to install pyserial first
import serial.tools.list_ports


class Communication(object):
    """
    Python serial communication package class
    """

    def __init__(self, com, bps, timeout):
        self.port = com  # serial port number
        self.bps = bps  # baud rate
        self.timeout = timeout  # timeout
        self.main_engine = None  # global serial communication object
        self.data = None
        self.b_c_text = None

        try:
            # open the serial port and get the serial port object
            self.main_engine = serial.Serial(self.port, self.bps, timeout=self.timeout)
            # determine whether the opening is successful
            self.is_open = self.main_engine.is_open
        except Exception as e:
            print('---Exception---：', e)

    def Print_Name(self):
        """
        Print the basic information of the device
        """

        if self.main_engine is None:
            print('Serial port not open')
            return

        print(self.main_engine.name)  # device name
        print(self.main_engine.port)  # read or write port
        print(self.main_engine.baudrate)  # baud rate
        print(self.main_engine.bytesize)  # byte size
        print(self.main_engine.parity)  # parity
        print(self.main_engine.stopbits)  # stop bits
        print(self.main_engine.timeout)  # read timeout setting
        print(self.main_engine.writeTimeout)  # write timeout setting
        print(self.main_engine.xonxoff)  # software flow control setting
        print(self.main_engine.rtscts)  # hardware (RTS/CTS) flow control setting
        print(self.main_engine.dsrdtr)  # hardware (DSR/DTR) flow control setting
        print(self.main_engine.interCharTimeout)  # character interval timeout

    def Open_Engine(self):
        """
        open serial port
        """
        # If the serial port is not open, open the serial port
        if not self.main_engine.is_open:
            self.main_engine.open()
            self.is_open = True

    def Close_Engine(self):
        """
        close serial port
        """
        # print(self.main_engine.is_open)  # check if the serial port is open
        # determine whether to open
        if self.main_engine.is_open:
            self.main_engine.close()  # close serial port
            self.is_open = False

    @staticmethod
    def Print_Used_Com():
        """
        print the list of available serial ports
        """

        ports = serial.tools.list_ports.comports()
        port_list = [(port.device, port.name) for port in ports]
        return port_list

    # Receive data of specified size
    # Read size bytes from the serial port.
    # If a timeout is set it may return less characters as requested.
    # With no timeout it will block until the requested number of bytes is read.
    def Read_Size(self, size):
        """
        Receive data of specified size
        :param size:
        :return:
        """
        return self.main_engine.read(size=size)

    # Receive a line of data
    # When using readline(), you should pay attention:
    # you should specify a timeout when opening the serial port,
    # otherwise, if the serial port does not receive a new line,
    # it will wait forever.
    # If there is no timeout, readline() will report an exception.
    def Read_Line(self):
        """
        Receive a line of data
        :return:
        """
        return self.main_engine.readline()

    def Send_data(self, data):
        """
        send data
        :param data:
        """
        if not self.is_open:
            print('Serial port is not open')
            return
        self.main_engine.write(data)

    # more examples
    # self.main_engine.write(bytes(listData))  # send list data listData = [0x01, 0x02, 0xFD] or listData = [1, 2, 253]
    # self.main_engine.write(chr(0x06).encode("utf-8"))  # send a data in hexadecimal
    # print(self.main_engine.read().hex())  # read a byte in hexadecimal
    # print(self.main_engine.read())  # read a byte
    # print(self.main_engine.read(10).decode("gbk"))  # read 10 bytes
    # print(self.main_engine.readline().decode("gbk"))  # read a line
    # print(self.main_engine.readlines())  # read multiple lines, return the list, must match the timeout (timeout) use
    # print(self.main_engine.in_waiting)  # get the remaining bytes of the input buffer
    # print(self.main_engine.out_waiting)  # Get the remaining bytes of the output buffer
    # print(self.main_engine.readall())# read all characters

    # receive data
    # an integer data occupies two bytes
    # a character occupies one byte
    def Receive_data(self, way):
        """
        receive data
        :param way:
        """
        # Receiving data cyclically, this is an endless loop,
        # which can be implemented by threads
        print('Start receiving data：')
        try:
            while True:
                if self.main_engine.in_waiting:
                    if way == 0:
                        for _ in range(self.main_engine.in_waiting):
                            print('Received ASCII data:', self.read_data(1))
                    elif way == 1:
                        self.data = self.main_engine.read(self.main_engine.in_waiting).decode("utf-8")
                        print('Received ASCII data:', self.data)
                        if not self.data.strip():
                            break
        except Exception as e:
            print('Error while receiving data:', e)
        print('Data reception completed!')


if __name__ == '__main__':
    available_ports = Communication.Print_Used_Com()
    if available_ports:
        port = available_ports[0][0]  # Select the first available port
        communication = Communication(port, 115200, 0.5)
        print('Is port open:', communication.is_open)
        communication.Open_Engine()
        communication.Receive_data(1)
        communication.Close_Engine()
    else:
        print('No available serial ports found.')


# if __name__ == '__main__':
#     Communication.Print_Used_Com()
#     port = port_list_number
#     myCom = Communication(port[0], 115200, 0.5)
#     print("Ret = ", Ret)
#     myCom.Open_Engine()
#     myCom.Receive_data(1)
#     myCom.Close_Engine()
