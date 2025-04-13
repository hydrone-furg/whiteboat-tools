from common import MAVLinkConnection
from arm_motor import ArmMotor
from log import Log

connection_string = 'udpin:127.0.0.1:14550' # connection_string = '/dev/ttyACM0', baud = 115200
arm_motor = ArmMotor(device=connection_string) # ArmMotor(device=connection_string, baud=baud)

# Conexão
print("Conectando-se ao veículo...")
print(arm_motor.connect())

# Armar
print("Armando o veículo...")
print(arm_motor.arm_vehicle())

# Canal 1 com PWM configurado em 1530
print("Configurando PWM do motor...")
print(arm_motor.set_motor_pwm(1, 1530))

# Desarmar
print("Desarmando o veículo...")
print(arm_motor.force_disarm())

# Log
log_ = Log(device=connection_string)
print("Iniciando leitura dos sensores (pressione CTRL+C para interromper)...")
log_result = log_.read_sensors()
print(log_result)