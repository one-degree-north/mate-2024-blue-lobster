#server communicates with the surface
import threading, queue, socket, select, struct

class Server:
    def __init__(self, server_address: tuple, stop_event=None, use_stop_event=False, debug=False):
        self.connected = False
        self.server_address = server_address
        self.thruster_control = None
        self.interface = None
        self.client_addr = ()
        self.server_thread = threading.Thread(target=self._server_loop)
        self.out_queue = queue.Queue()

        self.use_stop_event=use_stop_event
        self.debug=debug
        self.stop_event = stop_event
    
    def set_thruster_control(self, thruster_control):
        self.thruster_control = thruster_control

    def set_interface(self, interface):
        self.interface = interface

    # starts the server for surface client to communicate with
    def start_server(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(self.server_address)
        self.self_addr = self.server_address
        if self.debug:
            print(f"server set up at {self.server_address}")
        self.server_thread.start()

    # read and write data to and from surface client
    def _server_loop(self):
        while True:
            if self.use_stop_event:
                if self.stop_event.is_set():
                    break
            r, w, x = select.select([self.sock], [self.sock], [self.sock])
            for sock in r:  #ready to read!
                if self.debug:
                    print("attempting to read network data")
                data, address = sock.recvfrom(2048)
                if address != self.client_addr: #client switched address (something wrong happened)
                    self.client_addr = address
                self._parse_data(data)
            
            for sock in w:  #ready to write!
                if not self.out_queue.empty() and self.connected:
                    if self.debug:
                        print(f"attempting to write to {self.client_addr}")
                    sock.sendto(self.out_queue.get(), self.client_addr)
            
            for sock in x:  #exception 8^(. Create new socket and try to connect again.
                if self.debug:
                    print("exception apparently occured")
    
    #parse data from surface client
    def _parse_data(self, data):
        cmd = data[0]

        if cmd == 0x00: # test connection
            self.interface.test_connection()
        elif cmd == 0x01: # echo
            if len(data) > 0:
                self.interface.echo(data[1:])
        elif cmd == 0x10:   # first connection (just to get addr)
            print("client connected!")
        elif cmd == 0x20:   # set manual thrust
            if len(data) == 25:
                trans = struct.unpack("!fff", data[1:13])
                rot = struct.unpack("!fff", data[13:25])
                speed = struct.unpack("!f", data[25:29])
                self.thruster_control.set_manual_thrust(trans[0], trans[1], trans[2], rot[0], rot[1], rot[2], speed)
        elif cmd == 0x21:   # set pid thrust
            if len(data) == 25:
                trans = struct.unpack("!fff", data[1:13])
                rot = struct.unpack("!fff", data[13:25])
                speed = struct.unpack("!f", data[25:29])
                self.thruster_control.set_pid_thrust(trans[0], trans[1], trans[2], rot[0], rot[1], rot[2], speed)
        elif cmd == 0x22:   # set manual pos
            pass
        elif cmd == 0x30:   # close claw
            self.interface.move_claw(False)
        elif cmd == 0x31:   # open claw
            self.interface.move_claw(True)
            
    def send_sens_data(self, param, values):
        self.out_queue.put(struct.pack("!" + "c"*(3+len(values)), 0x33, param, len(values), *values))

if __name__ == "__main__":
    pass