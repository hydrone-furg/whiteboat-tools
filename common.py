from pymavlink import mavutil

class MAVLinkConnection:
    def __init__(self, device='/dev/ttyACM0', baud=115200):
        self.device = device
        self.baud = baud
        self.connection = None
        self.target_system = None
        self.target_component = None

    def connect(self):
        try:
            self.connection = mavutil.mavlink_connection(self.device, self.baud)
            self.connection.wait_heartbeat()
            self.target_system = self.connection.target_system
            self.target_component = self.connection.target_component
            return f"Conectado a {self.device}"
        except Exception as e:
            raise ConnectionError(f"Erro na conexão: {e}")