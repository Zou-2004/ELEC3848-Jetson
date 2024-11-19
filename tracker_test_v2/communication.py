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
        ret = struct.pack(self.pack_format, self.ID, struct.calcsize(self.pack_format) - struct.calcsize("=BH"), op_mode, self.vx, self.vy, self.wz)
        crc = crc16.crc16(0xffff, ret, len(ret))
        crc_packed = struct.pack("H", crc)
        return ret + crc_packed


class robortarmCommand:

    ID = 0x03

    angle=0
    height=0
    op_mode=0
    
    pack_format="".join(["=", "BH", "BHb"])

    def packed(self):
        ret = struct.pack(self.pack_format, self.ID, struct.calcsize(self.pack_format) - struct.calcsize("=BH"), self.op_mode, self.height, self.angle)
        crc = crc16.crc16(0xffff, ret, len(ret))
        crc_packed = struct.pack("H", crc)
        return ret + crc_packed
###send###

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
            id, size,finish_state,follow_state,pos_x_mm,pos_y_mm, angle_deg, _ = unpacked
            self.distance = distance_x10 / 10.0
            self.angleX = angleX_x10 / 10.0
            self.angleY = angleY_x10 / 10.0
            self.angleZ = angleZ_x10 / 10.0
            self.follow_state = follow_state
            return "OK"
        else:
            # checksum wrong
            print("CRC error!", crc_calc, unpacked[-1])
            return "CRC_ERR"


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
            id, size, is_holding,distance_mm, angleX_x10, angleY_x10, angleZ_x10, current_mA, voltage_mV,_ = unpacked
            
            self.is_holding = is_holding
            self.distance_mm = distance_mm

            self.angleX = angleX_x10 / 10.0
            self.angleY = angleY_x10 / 10.0
            self.angleZ = angleZ_x10 / 10.0
            
            self.current_mA = current_mA
            self.voltage_mV = voltage_mV

            return "OK"
        else:
            # checksum wrong
            print("CRC error!", crc_calc, unpacked[-1])
            return "CRC_ERR"


        pass

###receive###


#receive tof, robortarm, tracker,
#send detect_light_flag, tracker_flag,detect_plane_flag



class Communication:
    
    t = None
    def __init__(self, port, gyro_port, baud=115200):
        self.s = serial.Serial(port, baud, timeout=1)
        # self.s.set_buffer_size(64)
        self.s_gyro = serial.Serial(gyro_port, baud, timeout=1)

        self.chassis_cmd = chassisCommand()
        self.chassis_info = chassisInfo()
        self.roboarm = robortarmCommand()
        self.sensor = sensorInfo()

        self.running = False
        
    def send(self):
        self.s.write(self.chassis_cmd.packed())

    def start(self):
    self.running = True
        self.t = threading.Thread(target=self.recv)
        self.t_gyro = threading.Thread(target=self.recv_gyro)
        self.t.start()
        self.t_gyro.start()

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

    def recv(self):
        while self.running:
            packed = self.s.read(struct.calcsize(self.chassis_info.pack_format))
            id, size = self.get_id_size(packed)
            if id == self.chassis_info.ID:
                self.chassis_info.unpack(packed)
            elif id == self.roboarm.ID:
                self.roboarm.unpack(packed)
            else:
                print(f"ERROR: ID = {id}, size = {size}")


    def recv_gyro(self):
        while self.running:
            packed = self.s_gyro.read(struct.calcsize(self.gyro_info.pack_format))
            self.gyro_info.unpack(packed)

    def __del__(self):
        # Buggy
        self.running = False
        if (self.t != None):
            self.t.join()
            self.t = None
            self.s.close()



if __name__ == "__main__":
    ports = Communication.detect_ports()
    
    port_main = ports["CP2102 USB to UART Bridge Controller - CP2102 USB to UART Bridge Controller"]
    port_gyro = ports["USB Serial"]
    
    print(port_main, port_gyro)
    comm = Communication(port_main, port_gyro)

    comm.start()

    vx = 0

    try:
        while True:
            print(comm.chassis_info.distance,
                comm.gyro_info.angleX,
                comm.gyro_info.angleY,
                comm.gyro_info.angleZ)
            comm.chassis_cmd.vx = vx
            comm.send()
            vx = vx + 10 if vx < 300 else vx
            time.sleep(1)
    except KeyboardInterrupt: 
        del comm    


    # while True:
    #     # print(planeData.imu_r2r(data.roll), planeData.imu_r2r(data.pitch), planeData.imu_r2r(data.yaw))
    #     print(psr2alt(planeData.psr_r2r(data.pressure), 996), planeData.tmp_r2r(data.temperature))
    #     time.sleep(0.5)
