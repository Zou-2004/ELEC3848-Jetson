#communication.Communication(main_port, gyro_port)# Copyright Eric Jin 2020
# Modified in 2024 by Eric Jin 

import struct
import time
import crc16
 
import serial
import serial.tools.list_ports

import threading

###send###
# KEEP SYNCHRONIZED WITH MCU CODE!!!
class chassisCommand:
    ID = 0x01

    op_mode = 0
    vx = 0
    vy = 0
    wz = 0

    pack_format = "".join(["=","BH", "Bhhh"])

    def packed(self):
        ret = struct.pack(self.pack_format, self.ID, struct.calcsize(self.pack_format) - struct.calcsize("=BH"), self.op_mode, self.vx, self.vy, self.wz)
        crc = crc16.crc16(0xffff, ret, len(ret))
        crc_packed = struct.pack("H", crc)
        return ret + crc_packed + b'\xd9\xff'


class robortarmCommand:
    ID = 0x03

    op_mode=0
    height=0
    angle=0
    
    pack_format="".join(["=", "BH", "BHb"])

    def packed(self):
        ret = struct.pack(self.pack_format, self.ID, struct.calcsize(self.pack_format) - struct.calcsize("=BH"), self.op_mode, self.height, self.angle)
        crc = crc16.crc16(0xffff, ret, len(ret))
        crc_packed = struct.pack("H", crc)
        return ret + crc_packed + b'\xd9\xff'


###receive###
class chassisInfo:
    ID = 0x2

    finish_state=0
    follow_state = 0    #tracker
    pos_x_mm, pos_y_mm = 0, 0

    angle_deg = 0

    pack_format = "=BH" + "BBhhh" + "H"

    def size(self):
        return struct.calcsize(self.pack_format)
    
    def unpack(self, packed):
        if (len(packed) != struct.calcsize(self.pack_format)):
            print("LEN ERR", len(packed))
            return "LEN_ERR"
        unpacked = struct.unpack(self.pack_format, packed)
        # print("len packed =", len(packed))
        crc_calc = crc16.crc16(0xffff, packed, len(packed)-2)
        if crc_calc == unpacked[-1]:
            # print("unpacked", unpacked)
            # checksum correct
            id, size,self.finish_state, self.follow_state, self.pos_x_mm, self.pos_y_mm, self.angle_deg, _ = unpacked
            return "OK"
        else:
            # checksum wrong
            print("CRC error!", crc_calc, unpacked[-1])
            return "CRC_ERR"
        
    def __repr__(self) -> str:
        return f"e={self.finish_state}, s={bin(self.follow_state)}, x={self.pos_x_mm}, y={self.pos_y_mm}, a={self.angle_deg}"


class sensorInfo:
    ID = 0x4

    is_holding=0
    distance_mm = 0

    angleX = 0
    angleY = 0
    angleZ = 0

    current_mA = 0
    voltage_mV = 0

    pack_format="=BH" + "BH" + "hhh" + "HH" + "H"

    def size(self):
        return struct.calcsize(self.pack_format)
    
    def unpack(self, packed):
        if (len(packed) != struct.calcsize(self.pack_format)):
            print("LEN ERR", len(packed))
            return "LEN_ERR"
        unpacked = struct.unpack(self.pack_format, packed)
        # print("len packed =", len(packed))
        crc_calc = crc16.crc16(0xffff, packed, len(packed)-2)
        if crc_calc == unpacked[-1]:
            # print("unpacked", unpacked)
            # checksum correct
            id, size, self.is_holding, self.distance_mm, angleX_x10, angleY_x10, angleZ_x10, self.current_mA, self.voltage_mV,_ = unpacked
            
            self.angleX = angleX_x10 / 10.0
            self.angleY = angleY_x10 / 10.0
            self.angleZ = angleZ_x10 / 10.0
            
            return "OK"
        else:
            # checksum wrong
            print("CRC error!", crc_calc, unpacked[-1])
            return "CRC_ERR"
        
    def __repr__(self) -> str:
        return f"sw={self.is_holding}, d={self.distance_mm}, X={self.angleX}, Y={self.angleY}, Z={self.angleZ}, I={self.current_mA}, V={self.voltage_mV}"



SEND_INTERVAL = 0.025

class Communication:
    
    t = None
    send_time = 0

    def __init__(self, port, baud=115200):
        self.s = serial.Serial(port, baud, timeout=1.0)

        # send
        self.chassis_cmd = chassisCommand()
        self.roboarm_cmd = robortarmCommand()
        
        #receive
        self.chassis_info = chassisInfo()
        self.sensor_info = sensorInfo()

        self.running = False

    def __send(self, data):
        delta_t = time.time() - self.send_time
        if delta_t < SEND_INTERVAL:
            time.sleep(SEND_INTERVAL - delta_t)
        self.s.write(data)
        self.send_time = time.time()

    def send_chassis(self):
        self.__send(self.chassis_cmd.packed())
        
    def send_roboarm(self):
        self.__send(self.roboarm_cmd.packed())

    def start(self):
        self.running = True
        self.t = threading.Thread(target=self.recv)
        self.t.start()

    @staticmethod
    def get_id_size(pkt_bytes):
        id, size = struct.unpack("=BH", pkt_bytes[0:3])
        return id, size

    @staticmethod
    def detect_ports():
        ports = serial.tools.list_ports.comports()
        ret = dict()
        for p in ports:
            ret[p.description] = p.device
        return ret

    # todo: add send queue and send thread to pop the queue and write to serial
    def recv(self):
        while self.running:
            # packed = self.s.read(1024)
            packed = self.s.read_until(b'\xd9\xff')[:-2]
            # print(packed)
            if (packed != None and len(packed) > 3):
                id, size = self.get_id_size(packed)
                if id == self.chassis_info.ID:
                    self.chassis_info.unpack(packed)
                elif id == self.sensor_info.ID:
                    self.sensor_info.unpack(packed)
                else:
                    print(f"ERROR: ID = {id}, size = {size}")

    def stop(self):
        self.running = False
        if (self.t != None):
            self.t.join()
            self.t = None
            self.s.close()


if __name__ == "__main__":
    ports = Communication.detect_ports()
    
    port = ports["CP2102 USB to UART Bridge Controller - CP2102 USB to UART Bridge Controller"]
    print(port)
    comm = Communication(port)
    comm.start()

    

    try:
        while True:
            comm.roboarm_cmd.op_mode = 0x82
            comm.send_roboarm()
            print("holdinng? ",comm.sensor_info.is_holding)
            time.sleep(0.3)
        while True:
            print(comm.sensor_info.distance_mm)
            print("holdinng? ",comm.sensor_info.is_holding)
            print(comm.sensor_info.angleX)
            time.sleep(0.2)

        print("MODE 1:")
        while comm.chassis_info.follow_state == 0b0000:
            print(bin(comm.chassis_info.follow_state))
            print(comm.sensor_info.distance_mm)
            comm.chassis_cmd.op_mode = 1
            comm.chassis_cmd.vx = 40
            comm.send_chassis()
            time.sleep(0.1)

        print("MODE 2:")
        print(comm.chassis_info.finish_state)
        while not comm.chassis_info.finish_state:
            # print("CH: ", comm.chassis_info)
            # print("SI: ", comm.sensor_info)
            # comm.chassis_cmd.op_mode=2
            # comm.chassis_cmd.vx=20
            comm.send_chassis()
            time.sleep(0.1)

        print("Finish!!!")
        comm.chassis_cmd.op_mode = 0
        comm.send_chassis()
        print("Finish sent.")
        time.sleep(1)

    except KeyboardInterrupt: 
        del comm




    # while True:
    #     # print(planeData.imu_r2r(data.roll), planeData.imu_r2r(data.pitch), planeData.imu_r2r(data.yaw))
    #     print(psr2alt(planeData.psr_r2r(data.pressure), 996), planeData.tmp_r2r(data.temperature))
    #     time.sleep(0.5)
