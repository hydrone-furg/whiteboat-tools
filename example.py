from common import MAVLinkConnection

# mav = MAVLinkConnection(baud=57600)
mav = MAVLinkConnection(sitl_address='udp:127.0.0.1:14550') # para usar via SITL
mav.connect()
mav.read_sensors()
