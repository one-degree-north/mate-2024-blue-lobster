from sensors import Sensors
from server import Server
from controls import Controls

if __name__ == "__main__":
    sensors = Sensors()
    controls = Controls(sensors=sensors)
    server = Server(("192.168.2.2", 7774))
    server.set_interface(controls)
    controls.server = server
    print("starting controls")
    controls.start_loop()
    print("starting server")
    server_loop = server.start_server()
    server_loop.join()
