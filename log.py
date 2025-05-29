from datetime import datetime

def process_message(msg):
    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")
    log_entry = f"{timestamp} | {msg.to_dict()}\n"
    return log_entry

def save_log_file(logs):
    # if logs:
    #     filename = f'{datetime.now().strftime("%Y-%m-%d-%H%M%S")}-sensor_log.txt'
    #     with open(filename, 'w') as f:
    #         f.writelines(logs)
    #     return f"Log salvo em {filename}"
    return "Nenhum log para salvar"