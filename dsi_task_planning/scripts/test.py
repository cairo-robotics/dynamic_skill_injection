import abstraction_engine as ae

def main():
    obs_state = {}
    obs_state['room'] = {'type':'room', 'location_xyz':(0.0, 0.0, 0.0), 'location_str':[], 'size':(10.0, 10.0, 8.0)}
    obs_state['table'] = {'type':'object', 'location_xyz':(0.0, 0.0, 0.0), 'location_str':[], 'size':(5.0, 5.0, 2.0)}
    obs_state['mug'] = {'type':'object', 'location_xyz':(-1.0, 1.2, 1.5), 'location_str':[], 'size':(1.0, 1.0, 1.0)}
    obs_state['box'] = {'type':'object', 'location_xyz':(-1.4, 0.8, 1.9), 'location_str':[], 'size':(2.0, 2.0, 1.8)}

    # for key in obs_state:
    #     print key, obs_state[key]['location_str']
    # ae.generalize_batch(obs_state)
    # for key in obs_state:
    #     print "     ", key, obs_state[key]['location_str']

    print 'mug', obs_state['mug']['location_str']

    obj1_loc_str = obs_state['mug']['location_str']
    obj1_loc_xyz = obs_state['mug']['location_xyz']
    obj1_dims = obs_state['mug']['size']
    obj2 = 'table'
    obj2_loc_str = obs_state['table']['location_str']
    obj2_loc_xyz = obs_state['table']['location_xyz']
    obj2_dims = obs_state['table']['size']
    ae.generalize_single(obj1_loc_str, obj1_loc_xyz, obj1_dims, obj2, obj2_loc_xyz, obj2_dims)
    print 'mug', obs_state['mug']['location_str']


if __name__ == '__main__':
    main()
