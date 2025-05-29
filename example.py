from common import MAVLinkConnection

mav = MAVLinkConnection(baud=57600)
mav.connect()
mav.read_sensors()