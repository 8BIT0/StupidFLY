import serial
from ctypes import *
import serial.tools.list_ports

class frame_def(Structure):
    _pack_ = 1
    _fields_ = [
        ("h_magic", c_uint16),
        ("cmd",     c_uint8),
        ("buf",     c_uint16 * 16),
        ("chk_sum", c_uint16)
    ]

    _H_MAGIC = 0x55AA

    __CMD_SET_PWM_TYPE = 0
    __CMD_SET_CHANNEL_REVERT = 1

    __CMD_ACK_PWM_TYPE = 2
    __CMD_ACK_CHANNEL_REVERT = 3

    def __swap_u16(self, val: int) -> int:
        return ((val & 0x00FF) << 8) | ((val & 0xFF00) >> 8)

    def __calc_partial_checksum(self, raw_data : bytearray) -> int:
        raw = bytearray(self)
        start_off = 2
        calc_len  = 1 + 16 * 2
        end_off   = start_off + calc_len

        total = 0
        for b in raw[start_off:end_off]:
            total += b
        return total & 0xFFFF

    def __check_ack(self, data : dict, port : serial.Serial):
        ack_frame = frame_def()

        # clear frame
        ack_frame.h_magic = 0
        ack_frame.cmd = 0
        for i in range(16):
            ack_frame.buf[i] = 0
        ack_frame.chk_sum = 0

        # check header
        if data['h_magic'] != self._H_MAGIC:
            print(f"bad h_magic ---- rec header:{data['h_magic']}")
            return

        # check cmd
        if (data['cmd'] != self.__CMD_SET_PWM_TYPE) and (data['cmd'] != self.__CMD_SET_CHANNEL_REVERT):
            print(f'cmd invalid ---- cmd:{data["cmd"]}')
            return

        if data['cmd'] == self.__CMD_SET_PWM_TYPE:
            print('CMD ---- SET PWM TYPE')
            ack_frame.cmd = self.__CMD_ACK_PWM_TYPE
        elif data['cmd'] == self.__CMD_SET_CHANNEL_REVERT:
            print('CMD ---- CHANNEL REVERT')
            ack_frame.cmd = self.__CMD_ACK_CHANNEL_REVERT

        # check chk_sum
        calc = self.__calc_partial_checksum(self)
        if data['chk_sum'] != calc:
            print(f'check sum error ---- rec:{data["chk_sum"]} ---- calc:{calc}')
            return

        # ACK
        print('ACK type setting')

        ack_frame.h_magic = self._H_MAGIC
        for i in range(16):
            ack_frame.buf[i] = 0

        # 计算应答帧校验和（同样只算cmd~buf结尾）
        sum_ack = ack_frame.__calc_partial_checksum(ack_frame)
        ack_frame.chk_sum = ack_frame.__swap_u16(sum_ack)

        send_bytes = bytearray(ack_frame)
        port.write(send_bytes)

        pass

    def parse(self, port : serial.Serial):
        data = {}
        data["h_magic"] = self.h_magic
        data["cmd"] = self.cmd
        data["buf"] = [self.__swap_u16(x) for x in self.buf]
        data["chk_sum"] = self.__swap_u16(self.chk_sum)
        self.__check_ack(data, port)
        return data

    def print_data(self, data : dict):
        print(f"Magic    : 0x{data['h_magic']:04X}")
        print(f"CMD      : {data['cmd']}")
        print(f"BUF[16]  : {data['buf']}")
        print(f"CheckSum : 0x{data['chk_sum']:04X}")
        print("-" * 50)

def serial_recv_line(port, baud=115200):
    ser = serial.Serial(
        port=port,
        baudrate=baud,
        timeout=0.05,
        bytesize=8, stopbits=1, parity='N'
    )
    print("serial opened, Ctrl+C quit")

    try:
        buf = bytearray()
        buf.clear()

        while True:
            rec_len = ser.in_waiting
            if rec_len > 0:
                buf.extend(ser.read(rec_len))

            if len(buf) >= 38:
                raw_frame = buf[ : 38]
                buf = buf[38 : ]
                # parse
                frame = frame_def.from_buffer_copy(raw_frame)
                parse_data = frame.parse(ser)
                frame.print_data(parse_data)


    except KeyboardInterrupt:
        ser.close()
        print("serial closed")

serial_recv_line("/dev/ttyACM0", 1500000)
