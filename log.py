import csv
from datetime import datetime

def process_message(msg):
    if msg.get_type() == 'GPS_RAW_INT':
        lat = msg.lat / 1e7
        lon = msg.lon / 1e7
        return [lat, lon]
    return None

def save_log_file(logs):
    if logs:
        filename = f'{datetime.now().strftime("%Y-%m-%d-%H%M%S")}-gps_log.csv'
        header = ['lat', 'long']
        
        with open(filename, 'w', newline='', encoding='utf-8') as f:
            writer = csv.writer(f)
            writer.writerow(header)
            writer.writerows(logs)
            
        print(f"\nLog de GPS salvo com sucesso em {filename}")
        return f"Log salvo em {filename}"
    print("\nNenhum dado de GPS foi capturado para salvar.")
    return "Nenhum log para salvar"
