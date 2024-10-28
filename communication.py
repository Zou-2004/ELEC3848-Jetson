# Copyright Eric Jin 2020
# Modified in 2024 by Eric Jin 

import struct
import time
import crc16

import serial
import threading

# KEEP SYNCHRONIZED WITH MCU CODE!!!
class chassisCommand:
    ID = 0x01
    vx = 0
    vy = 0
    wz = 0

    pack_format = "".join(["=","BH", "hhh"])

    def packed(self):
        ret = struct.pack(self.pack_format, self.ID, struct.calcsize(self.pack_format) - struct.calcsize("=BH"), self.vx, self.vy, self.wz)
        crc = crc16.crc16(0xffff, ret, len(ret))
        crc_packed = struct.pack("H", crc)
        return ret + crc_packed


class chassisInfo:
    distance = 0
    angleX = 0
    angleY = 0
    angleZ = 0

    pack_format = "=BH" + "hhhh" + "H"

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
            id, size, distance_x10, angleX_x10, angleY_x10, angleZ_x10, _ = unpacked
            self.distance = distance_x10 / 10.0
            self.angleX = angleX_x10 / 10.0
            self.angleY = angleY_x10 / 10.0
            self.angleZ = angleZ_x10 / 10.0
            return "OK"
        else:
            # checksum wrong
            print("CRC error!", crc_calc, unpacked[-1])
            return "CRC_ERR"


class Communication:
    def __init__(self, port, baud=115200):
        self.s = serial.Serial(port, baud, timeout=1)
        self.s.set_buffer_size(64)

        self.chassis_cmd = chassisCommand()
        self.chassis_info = chassisInfo()

        self.running = False

    def send(self):
        self.s.write(self.chassis_cmd.packed())

    def start(self):
        self.running = True
        self.t = threading.Thread(target=self.recv)
        self.t.start()

    def recv(self):
        while self.running:
            packed = self.s.read(struct.calcsize(self.chassis_info.pack_format))
            self.chassis_info.unpack(packed)

    def __del__(self):
        # Buggy
        self.running = False
        self.t.join()
        self.s.close()


if __name__ == "__main__":
    comm = Communication("COM5")
    comm.start()


    try:
        while True:
            print(comm.chassis_info.distance,
                comm.chassis_info.angleX,
                comm.chassis_info.angleY,
                comm.chassis_info.angleZ)
            time.sleep(1)
    except KeyboardInterrupt:
        del comm    

    # while True:
    #     # print(planeData.imu_r2r(data.roll), planeData.imu_r2r(data.pitch), planeData.imu_r2r(data.yaw))
    #     print(psr2alt(planeData.psr_r2r(data.pressure), 996), planeData.tmp_r2r(data.temperature))
    #     time.sleep(0.5)