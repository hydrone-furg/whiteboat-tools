from pymavlink import mavutil

class ArmTest:
    def __init__(self, device='/dev/ttyACM0', baud=115200):
        self.device = device
        self.baud = baud
        self.connection = None

    def connect(self):
        try:
            self.connection = mavutil.mavlink_connection(self.device, self.baud)
            return f"Conexão estabelecida com {self.device}"
        except Exception as e:
            raise ConnectionError(f"Falha na conexão: {e}")

    def arm_status(self):
        while True:
            msg = self.connection.recv_msg()
            if msg and msg.get_type() == 'HEARTBEAT':
                self.check_arm_status(msg)

    def check_arm_status(self, msg):
        if msg.type != mavutil.mavlink.MAV_TYPE_GCS:
            armed = self.connection.motors_armed()
            status = "ARMED" if armed else "DISARMED"
            print(status)