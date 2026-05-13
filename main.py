from fsm import Phase1FSM
from robot import Robot


vid_ip = ''
robot = Robot(ip=vid_ip)

Phase1FSM(robot).run()