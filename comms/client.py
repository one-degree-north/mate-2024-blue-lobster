# import RPi.GPIO as GPIO
import socket, select, queue, threading, struct
from collections import namedtuple
from dataclasses import dataclass

import sys
from pathlib import Path
path = Path(sys.path[0])
sys.path.insert(1, str((path.parent.parent.parent).absolute()))
sys.path.insert(1, str(path.absolute()))

# from firmware.bno_lib.bno import BNOPowerMode, BNODataOutputType, BNOOperationalMode
# from enum import *
import enum

class BNOPowerMode(enum.Enum):
    NORMAL = "normal"
    LOW = "low"
    SUSPEND = "suspend"


class BNOOperationalMode(enum.Enum):
    CONFIG = "config"
    ACCONLY = "acconly"
    MAGONLY = "magonly"
    GYRONLY = "gyronly"
    ACCMAG = "accmag"
    ACCGYRO = "accgyro"
    MAGGYRO = "maggyro"
    AMG = "amg"
    IMU = "imu"
    COMPASS = "compass"
    M4G = "m4g"
    NDOF = "ndof"
    NDOF_FMC = "ndof_fmc"


class BNODataOutputType(enum.Enum):
    ACC = "acc"
    GYR = "gyr"
    MAG = "mag"
    EUL = "eul"
    QUA = "qua"
    GRA = "gra"
    LIN = "lin"
    INF = "inf"
    CAL = "cal"
    CON = "con"

@dataclass
class data:
    accel:list[float]
    gyro:list[float]
    eul:list[float]
    vel:list[float]
    quat:list[float]
    lin:list[float]
    inf:list[float] #information
    cal:list[float] # calibration
    con:list[float] # continuous data (?), CHANGE THIS!!
    mag:list[float]
    gra: list[float]    # gravitaitonal vector
    def __init__(self):
        self.accel = [0, 0, 0]
        self.gyro = [0, 0, 0]
        self.eul = [0, 0, 0]
        self.vel = [0, 0, 0]
        self.quat = [0, 0, 0, 0]
        self.lin = [0, 0, 0]
        self.inf = [0, 0, 0]    # not sure how this should be represented
        self.cal = [0, 0, 0]    # also not sure
        self.mag = [0, 0, 0]
        self.gra = [0, 0, 0]
    # I don't want to use a dict, ok?
    def set_value(self, key, value):
        if key == BNODataOutputType.ACC:
            self.accel = value
        elif key ==  BNODataOutputType.GYR:
            self.gyro = value
        elif key ==  BNODataOutputType.EUL:
            self.eul = value
        elif key ==  BNODataOutputType.MAG:
            self.mag = value
        elif key ==  BNODataOutputType.QUA:
            self.quat = value
        elif key ==  BNODataOutputType.GRA:
            self.gra = value
        elif key ==  BNODataOutputType.LIN:
            self.lin = value
        elif key ==  BNODataOutputType.INF:
            self.inf = value
        elif key ==  BNODataOutputType.CAL:
            self.cal = value

class PIClient:
    #code to communicate to opi
    move = namedtuple("move", ['f', 's', 'u', 'p', 'r', 'y'])
    def __init__(self, server_address=("192.168.1.2", 7774)):
        self.out_queue = queue.Queue()
        self.client_thread = threading.Thread(target=self.client_loop, args=[server_address], daemon=True)
        self.client_thread.start()
        # self.bno_data = data()
        self.bno_data = {
        "accel":[0, 0, 0],
        "gyro":[0, 0, 0],
        "eul":[0, 0, 0],
        "lin":[0, 0, 0],
        "quat":[0, 0, 0, 0],
        "mag":None,  # not using atm
        "inf":None,  # not using atm
        "gra":None # not using atm
    }
    
    def client_loop(self, server_address):
        while True:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            # self.sock.connect(address)
            self.sock.sendto(bytes([0x00]), server_address) #try to connect
            print("client sent attempt to connect")
            while True:
                r, w, x = select.select([self.sock], [self.sock], [self.sock])
                for sock in r:  #ready to read!
                    print("received data")
                    self.process_data(sock.recv(2048))
                
                for sock in w:  #ready to write!
                    if not self.out_queue.empty():
                        sock.sendto(self.out_queue.get(), server_address)
                        print("wrote data")
                
                for sock in x:  #exception 8^(. Create new socket and try to connect again.
                    print("exception in networking")


    def process_data(self, pkt):
        # turn into packet
        cmd = pkt[0]
        param = pkt[1]
        data = pkt[2:]
        length = len(data)
        if cmd == 0x00:
            print(f"opi confirm: {data}")
        elif cmd == 0x1A:
            # thruster positions
            pass
        elif cmd == 0x33:
            # sensor data
            #TODO: make this shorter (but whatever), add calibration, add BNO mode
            if param == 0x00:   # accel
                if length == 12:
                    acc = struct.unpack("!fff", data)
                    self.bno_data.set_value(BNODataOutputType.ACC, acc)
            elif param == 0x01: # euler
                if length == 12:
                        eul = struct.unpack("!fff", data)
                        self.bno_data.set_value(BNODataOutputType.EUL, eul)
            elif param == 0x02: # mag
                if length == 12:
                        mag = struct.unpack("!fff", data)
                        self.bno_data.set_value(BNODataOutputType.MAG)
            elif param == 0x03:    # quat
                if length == 16:
                    quat = struct.unpack("!ffff", data)
                    self.bno_data.set_value(BNODataOutputType.QUA, quat)
            elif param == 0x04: # gra
                if length == 12:
                    gra = struct.unpack("!fff", data)
                    self.bno_data.set_value(BNODataOutputType.GRA, gra)
            elif param == 0x05: # linear accel
                if length == 12:
                    lin = struct.unpack("!fff", data)
                    self.bno_data.set_value(BNODataOutputType.LIN, lin)
        # match(cmd):
        #     case 0x00:
        #         # echo or hello
        #         print(f"opi confirm: {data}")
        #     case 0x1A:
        #         # thruster positions
        #         pass
        #     case 0x2A:
        #         # servo positions
        #         pass
        #     case 0x33:
        #         # sensor data
        #         #TODO: make this shorter (but whatever), add calibration, add BNO mode
        #         match(param):
        #             case 0x00:  # accel
        #                 if length == 12:
        #                     acc = struct.unpack("!fff", data)
        #                     self.bno_data["accel"] = acc
        #             case 0x01:  # euler
        #                 if length == 12:
        #                     eul = struct.unpack("!fff", data)
        #                     self.bno_data["eul"] = eul
        #             case 0x02:  # mag
        #                 if length == 12:
        #                     mag = struct.unpack("!fff", data)
        #                     self.bno_data["mag"] = mag
        #             case 0x03:  # quaternion
        #                 if length == 16:
        #                     quat = struct.unpack("!ffff", data)
        #                     self.bno_data["quat"] = quat
        #             case 0x04:  # gra
        #                 if length == 12:
        #                     gra = struct.unpack("!fff", data)
        #                     self.bno_data["gra"] = gra
        #             case 0x05:  # linear accel
        #                 if length == 12:
        #                     lin = struct.unpack("!fff", data)
        #                     self.bno_data["lin"] = lin
        #             case 0x06:  # inf
        #                 if length == 12:
        #                     inf = struct.unpack("!fff", data)
        #                     self.bno_data["inf"] = inf
        #             case 0x07:  # calibration
        #                 # self.bno_data.set_value(BNODataOutputType.CAL)
        #                 pass
        #             case 0x08:  # mode
        #                 pass

    def move_servo(self, pulse1, pulse2):
        self.out_queue.put(struct.pack("!cHH", bytes([0x20]), int(pulse1//2), int(pulse2//2)))

    def turn_flashlight_off(self):
        self.out_queue.put(struct.pack("!cc", bytes([0x30, 0x00])))

    def turn_flashlight_on(self):
        self.out_queue.put(struct.pack("!cc", bytes([0x30, 0x01])))

    def set_manual_thrust(self, moves):
        assert isinstance(moves, list), "thrusts must be an array of floats"
        assert len(moves) == 6, "thrusts must be 6 long"
        # print(thrusts[1])
        # self.out_queue.put(struct.pack("!cHHHHHHHH"), thrusts_int[0], thrusts_int[1], thrusts_int[2])
        # self.out_queue.put(struct.pack("!cfff", bytes([0x00]), *(moves[0:3])))
        # self.out_queue.put(struct.pack("!cfff", bytes([0x04]), *(moves[3:])))
        self.out_queue.put(struct.pack("!cffffff", bytes([0x20]), *(moves[0:6])))

    def set_pid_target(self, moves):    # moves are in target m/s
        self.out_queue.put(struct.pack("!cffffff", bytes([0x21]), *(moves[0:6])))

    def open_claw(self):
        self.out_queue.put(struct.pack("!c", bytes([0x30])))
    
    def close_claw(self):
        self.out_queue.put(struct.pack("!c", bytes([0x31])))
    # def set_pos_manual(self, moves):
    #     self.out_queue.put(struct.pack("!cfff", bytes([0x20]), *moves))
    
    # def set_rot_manual(self, moves):
    #     self.out_queue.put(struct.pack("!cfff", bytes([0x21]), *moves))

    # def set_pos_pid(self, target_vel):
    #     self.out_queue.put(struct.pack("!cfff", bytes([0x22]), *target_vel))
    
    # def set_rot_angle(self, target_eul):
    #     self.out_queue.put(struct.pack("!cfff", bytes([0x23]), *target_eul))
    
    # def set_rot_vel(self, target_vel):
    #     self.out_queue.put(struct.pack("!cfff", bytes([0x24]), *target_vel))

    def test_connection(self):
        self.out_queue.put(struct.pack("!c", bytes([0x10])))

if __name__ == "__main__":
    addr = str(input("enter server address> "))
    comms = PIClient((addr, 7774))
    while True:
        command = input()
        if command == "bruh":
            comms.test_connection()
        if command == "all":
            val = float(input(">"))
            comms.set_manual_thrust([val, val, val, val, val, val])
    # while True:
    #     command = input()
    #     match(command):
    #         case "bruh":
    #             comms.test_connection()
    #         case 'tt':
    #             comms.set_manual_thrust([1, 0, 0, 0, 0, 0])
    #         case 'stop':
    #             comms.set_manual_thrust([0, 0, 0, 0, 0, 0])
    #         case 'tf':
    #             comms.set_manual_thrust([0.2, 0, 0, 0, 0, 0])
    #         case 'ts':
    #             comms.set_manual_thrust([0, 0.2, 0, 0, 0, 0])
    #         case 'tu':
    #             comms.set_manual_thrust([0, 0, 0.2, 0, 0, 0])
    #         case 'all':
    #             val = float(input(">"))
    #             comms.set_manual_thrust([val, val, val, val, val, val])
    #         case 'sv':
    #             comms.move_servo(int(input("1: ")), int(input("2: ")))
    #         case 'on':
    #             comms.turn_flashlight_on()
    #         case 'off':
    #             comms.turn_flashlight_off()
    #         case "lel":
    #             thrust = input(">")
    #             comms.set_manual_thrust([float(thrust), 0, 0, 0, 0, 0])