from pymavlink import mavutil

class Motor:
    def __init__(self, device='/dev/ttyACM0', baud=115200):
        self.device = device
        self.baud = baud
        self.connection = None
        self.target_system = None
        self.target_component = None

    def connect(self):
        self.connection = mavutil.mavlink_connection(self.device, self.baud)
        self.connection.wait_heartbeat()
        self.target_system = self.connection.target_system
        self.target_component = self.connection.target_component
        return "Conexão MAVLink estabelecida"

    def set_motor_pwm(self, channel, pwm):
        if 1100 <= pwm <= 1900:
            self.connection.mav.command_long_send(
                self.target_system,
                self.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
                0,
                channel,
                pwm,
                0, 0, 0, 0, 0
            )
            return f"Motor {channel} configurado para {pwm}μs"
        raise ValueError("PWM deve estar entre 1100 e 1900")