from pymavlink import mavutil
from common import MAVLinkConnection

class ArmMotor(MAVLinkConnection):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.armed = False

    def check_armed_status(self):
        self.armed = self.connection.motors_armed()
        return self.armed

    def arm_vehicle(self):
        self.connection.mav.command_long_send(
            self.target_system,
            self.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1,
            0, 0, 0, 0, 0, 0
        )
        conf = self.connection.recv_match(type='COMMAND_conf', blocking=True, timeout=5)
        if conf and conf.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM and conf.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
            self.armed = True
            return "Veículo armado com sucesso!"
        else:
            raise RuntimeError("Falha ao armar o veículo.")

    def set_motor_pwm(self, channel, pwm):
        if not self.check_armed_status():
            raise PermissionError("Sistema deve estar armado para ativar os motores!")
        
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
            return f"Motor {channel} ativado: {pwm}μs (Status ARMADO: {self.armed})"
        raise ValueError("Valor PWM inválido (deve ser entre 1100 e 1900)")

    def force_disarm(self):
        self.connection.mav.command_long_send(
            self.target_system,
            self.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            0,
            0, 0, 0, 0, 0, 0
        )
        self.armed = False
        return "Desarmado com sucesso!"
