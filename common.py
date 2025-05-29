#!/usr/bin/env python3
from pymavlink import mavutil
import serial.tools.list_ports

class MAVLinkConnection:
    def __init__(self, baud=115200):
        pixhawk_port = find_pixhawk_port()
        if pixhawk_port:
            print(f"Porta do Pixhawk detectada: {pixhawk_port}")
        else:
            print("Pixhawk não encontrado. Verifique a conexão USB.")
        self.port = pixhawk_port
        self.baud = baud
        self.connection = None
        self.target_system = None
        self.target_component = None

    def connect(self):
        try:
            self.connection = mavutil.mavlink_connection(self.port, self.baud) # self.connection = mavutil.mavlink_connection(self.device)
            self.connection.wait_heartbeat()
            self.target_system = self.connection.target_system
            self.target_component = self.connection.target_component
            return f"Conectado a {self.port}"
        except Exception as e:
            raise ConnectionError(f"Erro na conexão: {e}")

def find_pixhawk_port():
    # Lista todas as portas seriais disponíveis
    ports = serial.tools.list_ports.comports()
    
    for port in ports: print(port.description)

    # Caso 1: Verifica por portas ACM (USB comum para Pixhawk)
    for port in ports:
        if 'ACM' in port.device:
            print(f"Possível Pixhawk encontrado em {port.device} (USB/ACM)")
            return port.device
    
    # Caso 2: Verifica por portas USB (adaptadores FTDI/telemetria)
    for port in ports:
        if 'USB' in port.device:
            print(f"Possível Pixhawk encontrado em {port.device} (USB/FTDI)")
            return port.device
    
    # Caso 3: Verifica via descrição do hardware (vendor ID)
    for port in ports:
        if '3D Robotics' in port.description or 'Silicon Labs' in port.description or \
            'Pixhawk' in port.description:
            print(f"Pixhawk identificado por descrição em {port.device}")
            return port.device
    
    return None
