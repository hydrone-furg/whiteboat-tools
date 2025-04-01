from pymavlink import mavutil
from datetime import datetime
import time

class Log:
    def __init__(self, device='/dev/ttyACM0', baud=115200):
        self.logs = []
        self.device = device
        self.baud = baud
        self.connection = None

    def connect(self):
        try:
            self.connection = mavutil.mavlink_connection(self.device, self.baud)
            return f"Conectado a {self.device}"
        except Exception as e:
            raise ConnectionError(f"Erro na conexão: {e}")

    def read_sensors(self):
        try:
            while True:
                msg = self.connection.recv_match(blocking=True)
                if msg and msg.get_type() in ['ATTITUDE', 'GPS_RAW_INT', 'SYS_STATUS']:
                    self._process_message(msg)
        except KeyboardInterrupt:
            self.save_log()
            return "Leitura interrompida pelo usuário"

    def _process_message(self, msg):
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")
        log_entry = f"{timestamp} | {msg.to_dict()}\n"
        self.logs.append(log_entry)
        print(log_entry.strip())

    def save_log_file(self):
        if self.logs:
            timestamp = datetime.now().strftime("%Y-%m-%d-%H%M%S")
            filename = f'{timestamp}-sensor_log.txt'
            with open(filename, 'w') as f:
                f.writelines(self.logs)
            return f"Logs salvos em {filename}"
        return "Nenhum log para salvar"