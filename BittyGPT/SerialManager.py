#!/usr/bin/python3
# -*- coding: UTF-8 -*-

import copy
import glob
import logging
import os
import platform
import struct
import sys
import threading
import time

from SerialCommunication import Communication
import config
import tkinter as tk

# Configure logging
FORMAT = '%(asctime)-15s %(name)s - %(levelname)s - %(message)s'
logging.basicConfig(
    filename='./logfile.log', filemode='a+', level=logging.DEBUG, format=FORMAT
)
logger = logging.getLogger(__name__)
logger.info("ardSerial date: Jun. 20, 2024")


def print_header(header, value):
    print(f"{header} {value}")


def ensure_bytes(input_str, encoding='utf-8'):
    return input_str if isinstance(input_str, bytes) else input_str.encode(encoding)


class RobotController:
    def __init__(self):
        self.delay_between_slices = 0.001
        self.return_value = ''
        self.logger = logging.getLogger(self.__class__.__name__)

    def serial_write_num_to_byte(self, port, token, var=None):
        self.logger.debug(f'serial_write_num_to_byte, token={token}, var={var}')
        if var is None:
            var = []

        if token == 'K':
            period = var[0]
            skill_header = 4 if period > 0 else 7
            frame_size = (
                8 if period > 1 else 16 if period == 1 else 20
            )  # gait, posture, behavior
            angle_ratio = 1

            for row in range(abs(period)):
                angles = var[
                    skill_header
                    + row * frame_size : skill_header
                    + row * frame_size
                    + min(16, frame_size)
                ]
                if any(angle > 125 or angle < -125 for angle in angles):
                    angle_ratio = 2
                    break

            if angle_ratio == 2:
                var[3] = 2
                for row in range(abs(period)):
                    indices = range(
                        skill_header + row * frame_size,
                        skill_header
                        + row * frame_size
                        + min(16, frame_size),
                    )
                    for i in indices:
                        var[i] //= 2
                print_header('rescaled:\n', var)

            var = list(map(int, var))
            in_str = token.encode() + struct.pack('b' * len(var), *var) + b'~'

        else:
            message = list(map(int, var)) if var else []
            if token.isupper():
                if token == 'B':
                    for i in range(len(message) // 2):
                        message[i * 2 + 1] *= 8  # Adjust timing for tests
                        self.logger.debug(f"{message[i*2]},{message[i*2+1]}")
                pack_type = 'B' if token in ('W', 'C') else 'b'
                in_str = (
                    token.encode()
                    + struct.pack(f'{pack_type}' * len(message), *message)
                    + b'~'
                )
            else:
                message_str = ' '.join(str(round(elem)) for elem in var)
                in_str = token.encode() + ensure_bytes(message_str) + b'\n'

        slice_index = 0
        while len(in_str) > slice_index:
            chunk = in_str[slice_index: slice_index + 20]
            port.send_data(chunk)
            slice_index += 20
            time.sleep(self.delay_between_slices)
        self.logger.debug(f'Sent data: {in_str}')

    def serial_write_byte(self, port, var=None):
        self.logger.debug(f'serial_write_byte, var={var}')
        if var is None:
            var = []
        token = var[0][0]
        in_str = ''
        if (
            token in 'cmi but'
            and len(var) >= 2
        ):
            in_str = ' '.join(var) + '\n'
        elif token in 'LI':
            if len(var[0]) > 1:
                var.insert(1, var[0][1:])
            var[1:] = list(map(int, var[1:]))
            in_str = (
                token.encode()
                + struct.pack('b' * (len(var) - 1), *var[1:])
                + b'~'
            )
        elif token in 'wkX':
            in_str = var[0] + '\n'
        else:
            in_str = token + '\n'
        self.logger.debug(f"Sending: {in_str}")
        port.send_data(ensure_bytes(in_str))
        time.sleep(0.01)

    def print_serial_message(self, port, token, timeout=0):
        threshold = 4 if token in 'kK' else 3
        start_time = time.time()
        all_prints = ''
        while True:
            time.sleep(0.001)
            if port:
                response = port.serial_engine.readline().decode('ISO-8859-1')
                if response:
                    self.logger.debug(f"response is: {response}")
                    response_trim = response.split('\r')[0]
                    self.logger.debug(f"response_trim is: {response_trim}")
                    if response_trim.lower() == token.lower():
                        return [response, all_prints]
                    elif token == 'p' and response_trim == 'k':
                        return [response, all_prints]
                    else:
                        all_prints += response
            now = time.time()
            if (now - start_time) > threshold:
                self.logger.debug(f"Elapsed time: {threshold} seconds")
                threshold += 2
                if threshold > 5:
                    return -1
            if 0 < timeout < now - start_time:
                return -1

    def send_task(self, port_list, port, task, timeout=0):
        self.logger.debug(f'Task: {task}')
        if port:
            try:
                previous_buffer = port.serial_engine.read_all().decode('ISO-8859-1')
                if previous_buffer:
                    self.logger.debug(f"Previous buffer: {previous_buffer}")
                if len(task) == 2:
                    self.serial_write_byte(port, [task[0]])
                elif isinstance(task[1][0], int):
                    self.serial_write_num_to_byte(port, task[0], task[1])
                else:
                    self.serial_write_byte(port, task[1])
                token = task[0][0]
                if token in 'IL':
                    timeout = 1
                last_message = self.print_serial_message(port, token, timeout)
                time.sleep(task[-1])
            except Exception as e:
                self.logger.error(f"Error while sending task {task}: {e}")
                if port in port_list:
                    port_list.pop(port)
                last_message = -1
        else:
            last_message = -1
        self.return_value = last_message
        return last_message

    def send_task_parallel(self, ports, task, timeout=0):
        threads = []
        for p in ports:
            t = threading.Thread(
                target=self.send_task,
                args=(good_ports, p, task, timeout),
            )
            threads.append(t)
            t.start()
        for t in threads:
            if t.is_alive():
                t.join()
        return self.return_value

    def split_task_for_large_angles(self, task):
        token = task[0]
        queue = []
        if len(task) > 2 and token in 'LI':
            var = task[1]
            indexed_list = []
            if token == 'L':
                for i in range(4):
                    for j in range(4):
                        angle = var[4 * j + i]
                        if angle < -125 or angle > 125:
                            indexed_list += [4 * j + i, angle]
                            var[4 * j + i] = max(min(angle, 125), -125)
                if var:
                    queue.append(['L', var, task[-1]])
                if indexed_list:
                    queue[0][-1] = 0.01
                    queue.append(['i', indexed_list, task[-1]])
            elif token == 'I':
                if min(var) < -125 or max(var) > 125:
                    task[0] = 'i'
                queue.append(task)
        else:
            queue.append(task)
        return queue

    def send(self, port, task, timeout=0):
        if isinstance(port, dict):
            p = list(port.keys())
        elif isinstance(port, list):
            p = port
        queue = self.split_task_for_large_angles(task)
        for task in queue:
            if len(port) > 1:
                return_result = self.send_task_parallel(p, task, timeout)
            # elif len(port) == 1:
            #    return_result = self.send_task(p, p[0], task, timeout)
            else:
                return -1
        return return_result

    def close_serial_behavior(self, port):
        try:
            port.close_engine()
        except Exception as e:
            port.close_engine()
            raise e
        logger.info("Closed the serial port.")

    def close_all_serial(self, ports, clear_ports=True):
        if clear_ports:
            self.send(ports, ['d', 0], 1)
        for p in ports:
            t = threading.Thread(target=self.close_serial_behavior, args=(p,))
            t.start()
            t.join()
        if clear_ports:
            ports.clear()

    def keep_reading_input(self, ports):
        while True and len(ports):
            time.sleep(0.001)
            x = input()  # blocked waiting for the user's input
            if x != '':
                if x == 'q' or x == 'quit':
                    break
                else:
                    token = x[0]
                    task = x[1:].split()  # white space
                    if len(task) <= 1:
                        self.send(ports, [x, 1])
                    else:
                        self.send(ports, [token, list(map(int, task)), 1])



class PortManager:
    def __init__(self):
        self.good_ports = {}
        self.port_str_list = []
        self.initialized = False
        self.good_port_count = 0
        self.logger = logging.getLogger(self.__class__.__name__)

    @staticmethod
    def delete_duplicated_usb_serial(port_list):
        for item in port_list:
            if 'modem' in item:
                serial_number = item[item.index('modem') + 5 :]
                for name in port_list:
                    if serial_number in name and 'modem' not in name:
                        port_list.remove(name)
            elif 'serial-' in item:
                serial_number = item[item.index('serial-') + 7 :]
                for name in port_list:
                    if serial_number in name and 'wch' in name:
                        port_list.remove(name)
            elif 'cu.SLAB_USBtoUART' in item:
                port_list.remove(item)
        return port_list

    def test_port(self, port_list, serial_object, port_name):
        try:
            time.sleep(3)
            wait_time = 0
            result = serial_object.serial_engine
            if result:
                result = result.read_all().decode('ISO-8859-1')
                if result:
                    print('Waiting for the robot to boot up')
                    time.sleep(2)
                    wait_time = 3
                else:
                    wait_time = 2
                robot_controller = RobotController()
                result = robot_controller.send_task(
                    port_list, serial_object, ['?', 0], wait_time
                )
                if result != -1:
                    self.logger.debug(f"Adding port: {port_name}")
                    port_list.update({serial_object: port_name})
                    self.good_port_count += 1
                else:
                    serial_object.close_engine()
                    print(f'* Port {port_name} is not connected to a Petoi device!')
        except Exception as e:
            print(f'* Port {port_name} cannot be opened!')
            raise e

    def check_port_list(self, port_list, all_ports, need_testing=True):
        """Check the list of ports and add available ones to the port list.

        This method iterates over the provided list of serial ports (`all_ports`), attempts to create a
        `Communication` object for each port, and adds it to the `port_list` if successful.

        If `need_testing` is True, it will start a thread to test each port using the `test_port` method.
        If an exception occurs while creating the `Communication` object, it will be caught and logged,
        and the method will continue to the next port.

        Args:
            port_list (dict): A dictionary to store successfully opened ports.
            all_ports (list): A list of serial port paths to check.
            need_testing (bool, optional): Whether to test each port using `test_port`. Defaults to True.
        """
        threads = []
        for p in reversed(all_ports):
            try:
                serial_object = Communication(p, 115200, 1)
            except Exception as e:
                self.logger.warning(f'Could not open port {p}: {e}')
                continue  # Ignore the exception and proceed to the next port
            if need_testing:
                t = threading.Thread(
                    target=self.test_port,
                    args=(port_list, serial_object, p.split('/')[-1]),
                )
                threads.append(t)
                t.start()
            else:
                self.logger.debug(f'Adding port: {p}')
                port_list.update({serial_object: p.split('/')[-1]})
                self.good_port_count += 1
                self.logger.info(f'Connected to serial port: {p}')
        if need_testing:
            for t in threads:
                if t.is_alive():
                    t.join(8)

    def show_serial_ports(self, all_ports):
        """Display and log all available serial ports on the system."""
        if os.name == 'posix' and sys.platform.lower().startswith('linux'):
            extra_ports = glob.glob('/dev/ttyS*')
            for port in extra_ports:
                if port not in all_ports:
                    all_ports.append(port)
            for item in all_ports:
                if 'AMA0' in item:
                    all_ports.remove(item)
        all_ports = self.delete_duplicated_usb_serial(all_ports)
        self.logger.info('*** Available serial ports: ***')
        for index, port in enumerate(all_ports):
            self.logger.info(f'{port}')
        if platform.system() != 'Windows':
            for p in all_ports:
                if 'cu.usb' in p:
                    print(
                        '\n* Manually connect to the following port '
                        'if it fails to connect automatically\n'
                    )
                    print(p.replace('/dev/', ''), end='\n\n')

    def connect_port(self, port_list, need_testing=True, need_send_task=True, need_open_port=True):
        all_ports = [Communication.list_available_ports()[0][0]]
        
        print(f'Before {all_ports}')
        self.show_serial_ports(all_ports)
        print(f'After {all_ports}')

        if all_ports:
            self.good_port_count = 0
            if need_open_port:
                self.check_port_list(port_list, all_ports, need_testing)
        self.initialized = True
        if need_open_port:
            print(f'port_list {port_list}')
            if not port_list:
                print(
                    'No port found! Please make sure the serial port can be recognized by the computer first.'
                )
                #print('Replug mode')
                #self.replug(port_list, need_send_task, need_open_port)
            else:
                self.logger.info(f"Connected serial ports:")
                for p in port_list:
                    self.logger.info(f"{port_list[p]}")
                    self.port_str_list.append(port_list[p])
        else:
            if not all_ports or len(all_ports) > 1:
                print('Replug mode')
                self.replug(port_list, need_send_task, need_open_port)
            else:
                port_name = all_ports[0].split('/')[-1]
                self.port_str_list.insert(0, port_name)

    def replug(self, port_list, need_send_task=True, need_open_port=True):
        print('Please disconnect and reconnect the device from the COMPUTER side')
        window = tk.Tk()
        window.geometry('+800+500')
        window.title('Replug mode')

        def on_closing():
            window.destroy()
            os._exit(0)

        window.protocol('WM_DELETE_WINDOW', on_closing)

        thres = 10
        print('Counting down to manual mode:')

        def countdown(start, ap):
            cur_ports = copy.deepcopy(Communication.Print_Used_Com())
            if len(cur_ports) != len(ap):
                time.sleep(0.5)
                cur_ports = copy.deepcopy(Communication.Print_Used_Com())
                if len(cur_ports) < len(ap):
                    ap = cur_ports
                    start = time.time()
                else:
                    dif = list(set(cur_ports) - set(ap))
                    dif = self.delete_duplicated_usb_serial(dif)
                    success = False
                    for p in dif:
                        try:
                            port_name = p.split('/')[-1]
                            if need_open_port:
                                serial_object = Communication(p, 115200, 1)
                                port_list.update({serial_object: port_name})
                            self.port_str_list.append(port_name)
                            self.good_port_count += 1
                            tk.messagebox.showinfo(
                                title='Info',
                                message='New port detected: ' + port_name,
                            )
                            success = True
                        except Exception as e:
                            raise e
                            print(f"Cannot open {p}")
                    for p in ap:
                        self.port_str_list.append(p)
                    if success:
                        window.destroy()
                    else:
                        label_t.destroy()
                        label.destroy()
                        self.manual_select(port_list, window, need_send_task, need_open_port)
                    return
            if time.time() - start > thres:
                label_t.destroy()
                label.destroy()
                self.manual_select(port_list, window, need_send_task, need_open_port)
                return
            elif (time.time() - start) % 1 < 0.1:
                label['text'] = "{} s".format((thres - round(time.time() - start) // 1))
            window.after(100, lambda: countdown(start, ap))

        def b_callback():
            label_c.destroy()
            button_c.destroy()
            label_t['text'] = 'Counting down to manual mode: '
            label_t.grid(row=0, column=0)
            label.grid(row=1, column=0)
            label['text'] = "{} s".format(thres)
            countdown(time.time(), copy.deepcopy(Communication.Print_Used_Com()))

        label_c = tk.Label(window, font='sans 14 bold', justify='left')
        label_c['text'] = 'Please reconnect the device.'
        label_c.grid(row=0, column=0)
        button_c = tk.Button(window, text='Confirm', command=b_callback)
        button_c.grid(row=1, column=0, pady=10)
        label_t = tk.Label(window, font='sans 14 bold')
        label = tk.Label(window, font='sans 14 bold')
        window.focus_force()
        window.mainloop()

    def manual_select(self, port_list, window, need_send_task=True, need_open_port=True):
        all_ports = self.delete_duplicated_usb_serial(Communication.Print_Used_Com())
        window.title('Manual mode')
        l1 = tk.Label(window, font='sans 14 bold')
        l1['text'] = 'Manual mode'
        l1.grid(row=0, column=0)
        l2 = tk.Label(window, font='sans 14 bold')
        l2["text"] = 'Please select the port from the list'
        l2.grid(row=1, column=0)
        ls = tk.Listbox(window, selectmode="multiple")
        ls.grid(row=2, column=0)

        def refresh_box(ls):
            all_ports = self.delete_duplicated_usb_serial(Communication.Print_Used_Com())
            ls.delete(0, tk.END)
            for p in all_ports:
                ls.insert(tk.END, p)

        for p in all_ports:
            ls.insert(tk.END, p)
        bu = tk.Button(
            window,
            text='OK',
            command=lambda: self.select_list(port_list, ls, window, need_send_task, need_open_port),
        )
        bu.grid(row=2, column=1)
        bu2 = tk.Button(window, text='Refresh', command=lambda: refresh_box(ls))
        bu2.grid(row=1, column=1)
        tk.messagebox.showwarning(title='Warning', message='Manual mode')
        window.mainloop()

    def select_list(self, port_list, ls, win, need_send_task=True, need_open_port=True):
        for i in ls.curselection():
            p = ls.get(i)
            try:
                if need_open_port:
                    serial_object = Communication(p, 115200, 1)
                    port_list.update({serial_object: p.split('/')[-1]})
                self.port_str_list.append(p.split('/')[-1])
                self.good_port_count += 1
                self.logger.info(f"Connected to serial port: {p}")
                if need_open_port and need_send_task:
                    time.sleep(2)
                    robot_controller = RobotController()
                    result = robot_controller.send_task(port_list, serial_object, ['?', 0])
                win.withdraw()
            except Exception as e:
                tk.messagebox.showwarning(
                    title='Warning', message=f'* Port {p} cannot be opened'
                )
                print(f"Cannot open {p}")
                raise e
        win.destroy()


if __name__ == '__main__':
    try:
        port_manager = PortManager()
        robot_controller = RobotController()
        port_manager.connect_port(port_manager.good_ports)
        t = threading.Thread(
            target=port_manager.check_port_list, args=(port_manager.good_ports, [], True)
        )
        t.start()
        if len(sys.argv) >= 2:
            token = sys.argv[1][0]
            robot_controller.send(port_manager.good_ports, [token, sys.argv[1:], 1])
        print_header('Model list', config.modelList)
        print("You can type 'quit' or 'q' to exit.")
        robot_controller.keep_reading_input(port_manager.good_ports)
        robot_controller.close_all_serial(port_manager.good_ports)
        logger.info("Finish!")
        os._exit(0)
    except Exception as e:
        logger.info("Exception occurred.")
        robot_controller.close_all_serial(port_manager.good_ports)
        raise e

