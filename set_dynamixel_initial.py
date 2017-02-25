import pypot.dynamixel
import itertools
ports = pypot.dynamixel.get_available_ports()
dxl_io = pypot.dynamixel.DxlIO(ports[0])
dxl_io.set_goal_position(dict(zip([2], itertools.repeat(-120))))
dxl_io.set_goal_position(dict(zip([3], itertools.repeat(-130))))
