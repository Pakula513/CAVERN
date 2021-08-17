"""
Copyright MIT and Harvey Mudd College
MIT License
Summer 2020
"""
### CLIENT ###
########################################################################################
# Imports
########################################################################################

import sys
from typing import List, Optional, Tuple

sys.path.insert(1, "../../library")
import racecar_core
import racecar_utils as rc_utils
from enum import Enum

import math
import socket
import select
import time

########################################################################################
# Global variables
########################################################################################

rc = racecar_core.create_racecar()

# Put any global variables here

class State(Enum):
    FOLLOWING_LEFT = 1,
    FOLLOWING_RIGHT = 2,
    REVERSING_LEFT = 3,
    REVERSING_RIGHT = 4,
    FOLLOWING_OTHER_CAR = 5,
    FINISHED_HIGH_PRIORITY = 6,
    FINISHED_LOW_PRIORITY = 7


state : State = State.FOLLOWING_RIGHT

END_OF_PATH_ID : int = 0
RED = ((170, 100, 100), (10, 255, 255), "red")
GREEN = ((40, 60, 60), (90, 255, 255), "green")

KPA : float = 0.03
DIS_FROM_WALL : int = 60

cur_path : List[int] = [1]
target_path : List[int] = []

time_since_last_ar : float = 10000


s : socket.socket = None

########################################################################################
# Functions
########################################################################################

def connect_to_server():
    global s

    host = socket.gethostname()  # get local machine name
    port = 1024  # Make sure it's within the > 1024 $$ <65535 range

    s = socket.socket()
    s.connect((host, port))

def send(message: str):
    print('sending ' + message)
    s.send(message.encode('utf-8'))

def read() -> Optional[str]:
    read_sockets, _, _ = select.select([s] , [], [], 0.1)
    if s in read_sockets:
        data = s.recv(1024).decode('utf-8')
        if len(data) == 0: return None
        print('received ' + data)
        return data
    return None


def cur_path_to_string() -> str:
    a : str = ""
    for num in cur_path:
        a += str(num)
    return a

def at_correct_root() -> bool:
    for i in range(min(len(cur_path), len(target_path))):
        if cur_path[i] != target_path[i]: return False
    return True


def get_left_angle(scan, forward: bool) -> float:
    window = (-50, -40) if forward else (-140, -130)
    _, left_dis = rc_utils.get_lidar_closest_point(scan, window)
    angle = KPA * (DIS_FROM_WALL - left_dis)
    return angle

def get_right_angle(scan, forward: bool) -> float:
    window = (40, 50) if forward else (130, 140)
    _, right_dis = rc_utils.get_lidar_closest_point(scan, window)
    angle = KPA * (right_dis - DIS_FROM_WALL)
    return angle

def get_left_controller(scan, forward: bool) -> Tuple[float, float]:
    speed = 0.5 if forward else -0.3
    return speed, get_left_angle(scan, forward)

def get_right_controller(scan, forward: bool) -> Tuple[float, float]:
    speed = 0.5 if forward else -0.3
    return speed, get_right_angle(scan, forward)


def start():
    """
    This function is run once every time the start button is pressed
    """
    # Begin at a full stop
    rc.drive.stop()

    global state, cur_path

    state = State.FOLLOWING_RIGHT
    cur_path = [1]


def update():
    global state, cur_path, target_path, time_since_last_ar
    """
    After start() is run, this function is run every frame until the back button
    is pressed
    """

    time_since_last_ar += rc.get_delta_time()

    print(state)
    print(cur_path)

    depth_image = rc.camera.get_depth_image()
    depth_image = (depth_image - 0.01) % 10000

    color_image = rc.camera.get_color_image()
    # rc.display.show_color_image(color_image)
    ar_markers = rc_utils.get_ar_markers(color_image)

    scan = rc.lidar.get_samples()
    scan = (scan - 0.01) % 10000

    ar_marker = None
    min_dis = 175
    for marker in ar_markers:
        print(min_dis)
        corner = marker.get_corners()[0]
        if depth_image[corner[0]][corner[1]] < min_dis:
            ar_marker = marker
            min_dis = depth_image[corner[0]][corner[1]]
    
    print(min_dis)

    if ar_marker is not None and state != State.FINISHED_HIGH_PRIORITY:
        if time_since_last_ar > 2:
            print('found new ar marker')
            if state != State.FOLLOWING_OTHER_CAR:
                if ar_marker.get_id() == END_OF_PATH_ID:
                    ar_marker.detect_colors(color_image, (RED, GREEN))
                    if ar_marker.get_color() == 'red':
                        # TODO: Send info to the other racecar that it found the high priority situation
                        state = State.FINISHED_HIGH_PRIORITY
                        send(cur_path_to_string())
                    elif ar_marker.get_color() == 'green':
                        # TODO: No sending info but stop because it found the low priority situation
                        state = State.FINISHED_LOW_PRIORITY
                    else:
                        # TODO: Reverse while following the correct wall to back out
                        if state == State.FOLLOWING_LEFT:
                            state = State.REVERSING_LEFT
                        else:
                            state = State.REVERSING_RIGHT
                else:
                    # TODO: This is an intersection
                    if state == State.REVERSING_LEFT:
                        cur_path.pop()
                        if cur_path[-1] == 1:
                            state = State.REVERSING_RIGHT
                        else:
                            state = State.REVERSING_LEFT
                    elif state == State.REVERSING_RIGHT:
                        cur_path[-1] = 0
                        state = State.FOLLOWING_LEFT
                    else:
                        cur_path.append(1)
                        state = State.FOLLOWING_RIGHT
            elif not at_correct_root():
                cur_path.pop()
                if at_correct_root():
                    cur_path.append(target_path[len(cur_path)])
            else:
                cur_path.append(target_path[len(cur_path)])
        time_since_last_ar = 0.0
    
    # TODO: Check if the other car found the high priority position (go right away)
    # If the other car only found the low priority position, keep searching
    r = read()
    if r is not None:
        for i in range(len(r)):
            target_path.append(int(r[i]))
        state = State.FOLLOWING_OTHER_CAR
    
    speed : float = 0.0
    angle : float = 0.0

    if state == State.FOLLOWING_LEFT:
        # TODO: Follow the left wall while going forwards
        speed, angle = get_left_controller(scan, True)
    elif state == State.FOLLOWING_RIGHT:
        # TODO: Follow the right wall while going forwards
        speed, angle = get_right_controller(scan, True)
    elif state == State.REVERSING_LEFT:
        # TODO: Follow the left wall while going backwards
        speed, angle = get_left_controller(scan, False)
    elif state == State.REVERSING_RIGHT:
        # TODO: Follow the right wall while going backwards
        speed, angle = get_right_controller(scan, False)
    elif state == State.FOLLOWING_OTHER_CAR:
        # TODO: Follow the other car
        print(target_path)
        
        # Left car reversing as right car has found high priority 
        if not at_correct_root():
            if cur_path[-1] == 1:
                speed, angle = get_right_controller(scan, False)
            elif cur_path[-1] == 0:
                speed, angle = get_left_controller(scan, False)
        else:
            if cur_path[-1] == 1:
                speed, angle = get_right_controller(scan, True)
            elif cur_path[-1] == 0:
                speed, angle = get_left_controller(scan, True)

    else:
        # TODO: Done (for now in low priority, completely finished in high priority)
        pass
    
    angle = rc_utils.clamp(angle, -0.8, 0.8)
    
    rc.drive.set_speed_angle(speed, angle)
    pass



########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update)
    connect_to_server()
    time.sleep(1)
    rc.go()






# def define_server():
#     global s 

#     host = socket.gethostname()   # get local machine name
#     port = 1024  # Make sure it's within the > 1024 $$ <65535 range
  
#     sock = socket.socket()
#     sock.bind((host, port))
  
#     sock.listen(1)
#     s, addr = sock.accept()

#     print("Connection from: " + str(addr))

# def send(message: str):
#     s.send(message.encode('utf-8'))

# def read() -> Optional[str]:
#     read_sockets, _, _ = select.select([s] , [], [], 0.1)
#     if s in read_sockets:
#         data = s.recv(1024).decode('utf-8')
#         if len(data) == 0: return None
#         return data
#     return None