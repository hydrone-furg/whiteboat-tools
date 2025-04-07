from datetime import datetime
from common import MAVLinkConnection

class Log(MAVLinkConnection):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.logs = []

    def read_sensors(self):
        try:
            while True:
                msg = self.connection.recv_match(blocking=True)
                if msg and msg.get_type() in ['ATTITUDE', 'GPS_RAW_INT', 'SYS_STATUS']:
                    self._process_message(msg)
        except KeyboardInterrupt:
            self.save_log_file()
            return "Leitura interrompida pelo usuário"

    def _process_message(self, msg):
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")
        log_entry = f"{timestamp} | {msg.to_dict()}\n"
        self.logs.append(log_entry)
        print(log_entry.strip())

    def save_log_file(self):
        if self.logs:
            filename = f'{datetime.now().strftime("%Y-%m-%d-%H%M%S")}-sensor_log.txt'
            with open(filename, 'w') as f:
                f.writelines(self.logs)
            return f"Log salvo em {filename}"
        return "Nenhum log para salvar"