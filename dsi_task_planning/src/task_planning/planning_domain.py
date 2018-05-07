#!/usr/bin/env python

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

def open_door(state_struct, door):
    state = state_struct.data
    room1 = state[door]['location_str'][0][1]
    room2 = state[door]['location_str'][0][2]
    robot_loc = state['robot1']['location_str']
    if (('in',room1) in robot_loc or ('in',room2) in robot_loc) and not state[door]['locked']:
        state[door]['open'] = True
        return state_struct
    else:
        return False

def detect(state_struct, obj):
    state = state_struct.data
    room = get_tgt_room(state, obj)
    lights = state[room]['lights']
    robot_loc = state['robot1']['location_str']
    if ('in',room) in robot_loc and state[lights]['on']:
        state[obj]['detected'] = True
        return state_struct
    else:
        return False

def pickup(state_struct, obj):
    state = state_struct.data
    room = get_tgt_room(state, obj)
    robot_loc = state['robot1']['location_str']
    if ('in',room) in robot_loc and state[obj]['detected']:
        if state['robot1']['right_hand_empty']:
            state['robot1']['right_hand_empty'] = False
        elif state['robot1']['left_hand_empty']:
            state['robot1']['left_hand_empty'] = False
        else:
            return False
        state['robot1']['carrying'].append(obj)
        return state_struct
    else:
        return False

def set_down(state_struct, obj, preposition, tgt):
    state = state_struct.data
    room = get_tgt_room(state, tgt)
    robot_loc = state['robot1']['location_str']
    if ('in',room) in robot_loc:
        # TODO: The effects empty both hands even if is only setting down one object
        state['robot1']['left_hand_empty'] = True
        state['robot1']['right_hand_empty'] = True
        state['robot1']['carrying'].remove(obj)
        # TODO: Make the new object position assignment more robust
        state[obj]['location_str'] = [('in'),room]
        if room!=tgt:
            state[obj]['location_str'].append((preposition,tgt))
        return state_struct
    else:
        return False

def unlock_door(state_struct, door):
    state = state_struct.data
    room1 = state[door]['location_str'][0][1]
    room2 = state[door]['location_str'][0][2]
    robot_loc = state['robot1']['location_str']
    if ('in',room1) in robot_loc or ('in',room2) in robot_loc:
        state[door]['locked'] = False
        return state_struct
    else:
        return False

def turn_on_lights(state_struct, lights):
    state = state_struct.data
    room = state[lights]['location_str'][0][1]
    if ('in',room) in state['robot1']['location_str']:
        state[lights]['on'] = True
        return state_struct
    else:
        return False

# Define the methods

def navigate_to(state_struct, tgt):
    state = state_struct.data
    robot_loc = state['robot1']['location_str']
    robot_room = get_tgt_room(state, 'robot1')
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

def fetch(state_struct, obj, loc, preposition, tgt):
    #navigate_to(state, loc)
    state = state_struct.data
    move_to(state_struct, loc)
    #navigate_to(state_struct, loc)
    #detect(state_struct, obj)
    #pickup(state_struct, obj)
    #navigate_to(state, tgt)
    #navigate_to(state_struct, tgt)
    #set_down(state_struct, obj, preposition, tgt)
    return state_struct
