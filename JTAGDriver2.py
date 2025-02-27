# Author: Ying-Jiun, Low - Tron Future Tech Inc.
import socket

class JTAGDriver:
    def __init__(self, bit_stream_path, device_name, debug=False):
        self.debug = debug
        print("[INIT] JTAGDriver 2.0 (TCP version)")

        self.JTAG_CONNECT_STR  = ["open_hw_manager",
            "connect_hw_server -allow_non_jtag",
            "open_hw_target",
            f"set_property PROGRAM.FILE {{{bit_stream_path}}} [get_hw_devices {device_name}]",
            f"current_hw_device [get_hw_devices {device_name}]",
            f"refresh_hw_device [lindex [get_hw_devices {device_name}] 0]"]
        self.JTAG_PROGRAM_STR = [ f"program_hw_devices [get_hw_devices {device_name}]",
            f"refresh_hw_device [lindex [get_hw_devices {device_name}] 0]"]


        # 01. TCP BLOCING SOCKET(localhost:9999)
        HOST = '127.0.0.1'  # The server's hostname or IP address
        PORT = 9999        # The port used by the server

        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.s.connect((HOST, PORT))

        resp_status, resp = self.read_server_response()
        if (resp_status == 0):
            print("CONNECTION ESTABLISHED")
            print(f"Server replied: {resp}")

    def __del__(self):
        self.s.close()

    def read_server_response(self):
        resp = ''
        while True:
            try:
                resp += self.s.recv(1024).decode('UTF-8')
                if 'END\r\n' in resp: 
                    # print(resp,end='')
                    return 0,resp.replace('END\r\n','') 
            except socket.timeout:
                print("NO RESP. AVAILIABLE")
                return 1,''

    def jtag_connect(self):
        for i,cmd in enumerate(self.JTAG_CONNECT_STR):
            self.sendline(cmd)
            resp_status, _ = self.read_server_response()
            assert(resp_status==0)

    def jtag_program(self):
        for i,cmd in enumerate(self.JTAG_PROGRAM_STR):
            self.sendline(cmd)
            resp_status, _ = self.read_server_response()
            assert(resp_status==0)


    def sendline(self,cmd_str):
        cmd_bytes = (cmd_str+'\r\n').encode('UTF-8')
        if self.debug:
            print(f"[DEBUG] send: {cmd_bytes}")
        self.s.sendall(cmd_bytes)


    def read(self, start_address, word_length, addr_mode=1, debug=False):
        datas_32bit = []
        for length_offset in range(0, word_length, 256):
            if(self.debug):
                print(f"[DEBUG] Read(start_address = {start_address+length_offset:08x}"
                    f", length = {min(256,word_length-length_offset)})")
            _start_address = start_address+4*length_offset
            _word_length = min(256, word_length-length_offset)
            self.sendline(f"create_hw_axi_txn rd_txn [get_hw_axis hw_axi_1] -force -address {_start_address:08x} -len {_word_length} -type read")
            resp_status, _ = self.read_server_response()
            assert(resp_status==0)

            self.sendline('run_hw_axi rd_txn -verbose')
            resp_status, data = self.read_server_response()
            assert(resp_status==0)
            if(self.debug):
                print(f"DATA = {data}")
            datas_32bit.extend([int(d[1],16) for d in [t.split() for t in  filter(lambda x: x != "", data.split('\r\n'))]])
        return datas_32bit


    def write(self,start_address, data, addr_mode=1, debug=False):
        # Slice Data to 256 chunks
        data_chunks = [data[i:i+256] for i in range(0, len(data), 256)]

        for i in range(0, len(data_chunks)):
            if(debug):
                print("[DEBUG] DataChunks #",i)
                print("[DEBUG]",data_chunks[i])

            _data = ''.join("{:08x}".format(i&0xFFFF_FFFF) for i in data_chunks[i][::-1])

            if(debug):
                print(f"[DEBUG]{_data}")

            _length = len(data_chunks[i])
            _start_address = start_address + 4*256*i
            self.sendline(
                f'create_hw_axi_txn wr_txn [get_hw_axis hw_axi_1] -force -address {_start_address:08x} -data {_data} -len {_length} -type write')
            resp_status, _ = self.read_server_response()
            assert(resp_status==0)
            
            self.sendline('run_hw_axi wr_txn -verbose')
            resp_status, _ = self.read_server_response()
            assert(resp_status==0)


    def wait(self,addr, mask, expected, timeout,debug=False):  # time unit: us sec

        import time
        TIMEOUT_FLAG = 1

        time_elapsed = 0

        start_time = time.time()
        while(time_elapsed < timeout/1000000):

            time_elapsed = time.time() - start_time

            if(debug):
                print("[INFO] [{:f}s]".format(time_elapsed))

            # read
            data = self.read(start_address=addr, word_length=1)[0]
            time.sleep(0.000001)  # for read delay
            res = (data & mask == expected)

            if(debug):
              print(f"[DEBUG]{res:032b}")

            if(res):
                # print("mask matched")
                TIMEOUT_FLAG = 0
                break
        if TIMEOUT_FLAG == 1:
            print("mask not matched")
        return TIMEOUT_FLAG
