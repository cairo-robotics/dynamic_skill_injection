#!/usr/bin/env python
import pdb
'''
Definition of the operators and methods for the planning domain
'''

# Utilities
def get_tgt_room(state, tgt):
    #state = state.data
    room = None
    if state[tgt]['type']=='room':
        room = tgt
    else:
        for location in state[tgt]['location_str']:
            if location[0]=='in' and state[location[1]]['type']=='room':
                room = location[1]
    if room is None:
        print "Error:  could not determine room where tgt is located."
    return room

def find_path(state, a, b):
    # TODO: This should be a search of a graph representing the building layout
    room_b = b
    path = None
    if state[b]['type']!='room':
        room_b = get_tgt_room(state, b)
    if a==room_b:
        path = [a, b]
    else:
        if room_b in state[a]['location'][0]:
            path = [a, room_b]
        else:
            path = [a, 'hallway1', room_b]
        if room_b!=b:
            path.append(b)
    return path

def determine_door(state, a, b):
    door_from_a_to_b = None
    for door in state[a]['doors']:
        if a in state[door]['location_str'] and b in state[door]['location_str']:
            door_from_a_to_b = door
    return door_from_a_to_b

# Define the operators
def move_to(state_struct, tgt):
    state = state_struct.data
    room = get_tgt_room(state, tgt)
    state['movo']['location_str'] = [('in', room)]
    return state_struct

"""
def open_door(state_struct, door):
    state = state_struct.data
    room1 = state[door]['location_str'][0][1]
    room2 = state[door]['location_str'][0][2]
    robot_loc = state['movo']['location_str']
    if (('in',room1) in robot_loc or ('in',room2) in robot_loc) and not state[door]['locked']:
        state[door]['open'] = True
        return state_struct
    else:
        return False
"""


def detect(state_struct, obj):
    state = state_struct.data
    room = get_tgt_room(state, obj)
    lights = state[room]['lights_on']
    robot_loc = state['movo']['location_str']
    #if ('in',room) in robot_loc and lights:
    state[obj]['visible'] = True
    return state_struct
    #else:
    #    return False


def pickup(state_struct, obj):
    state = state_struct.data
    room = get_tgt_room(state, obj)
    detected = state[obj]['visible']
    robot_loc = state['movo']['location_str']
    if ('in', room) in robot_loc and detected:
        state['movo']['carrying'].append(obj)
        state_struct.data = state
        return state_struct
    else:
        return False

def set_down(state_struct, obj, preposition, tgt):
    state = state_struct.data
    room = get_tgt_room(state, tgt)
    robot_loc = state['movo']['location_str']
    if ('in',room) in robot_loc:
        # TODO: The effects empty both hands even if is only setting down one object
        state['movo']['left_hand_empty'] = True
        state['movo']['right_hand_empty'] = True
        state['movo']['carrying'].remove(obj)
        # TODO: Make the new object position assignment more robust
        state[obj]['location_str'] = [('in'),room]
        if room!=tgt:
            state[obj]['location_str'].append((preposition,tgt))
        return state_struct
    else:
        return False
"""
def unlock_door(state_struct, door):
    state = state_struct.data
    room1 = state[door]['location_str'][0][1]
    room2 = state[door]['location_str'][0][2]
    robot_loc = state['movo']['location_str']
    if ('in',room1) in robot_loc or ('in',room2) in robot_loc:
        state[door]['locked'] = False
        return state_struct
    else:
        return False
"""
def turn_on_lights(state_struct, room):
    state = state_struct.data
    state[room]['lights_on'] = True
    return state_struct

def turn_off_lights(state_struct, room):
    state = state_struct.data
    state[room]['lights_on'] = False
    return state_struct
# Define the methods
"""
def navigate_to(state_struct, tgt):
    state = state_struct.data
    robot_loc = state['movo']['location_str']
    robot_room = get_tgt_room(state, 'movo')
    path = find_path(state, robot_room, tgt)
    for idx in range(len(path)-1):
        start = path[idx]
        dest = path[idx+1]
        if state[dest]['type']=='room':
            door = determine_door(state, start, dest)
            if door:
                move_to(state_struct, door)
                open_door(state_struct, door)
                move_to(state_struct, dest)
        else:
            move_to(state_struct, dest)
    return state_struct
"""
"""
def fetch(state_struct, obj, loc, preposition, tgt):
    #navigate_to(state, loc)
    state = state_struct.data
    move_to(state_struct, loc)
    #navigate_to(state_struct, loc)
    #detect(state_struct, obj)
    ##pickup(state_struct, obj)
    move_to(state_struct, tgt)
    #navigate_to(state, tgt)
    #navigate_to(state_struct, tgt)
    ##set_down(state_struct, obj, preposition, tgt)
    return state_struct
"""
