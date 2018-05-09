#!/usr/bin/python

from failure_resolution.resolution import FailureMapper

world_state = {  
   'movo':{  
      'right_hand_empty':True,
      'location_str':[  
         [  
            'in',
            'blue_5m_room'
         ]
      ],
      'dimensions':[  
         0.4,
         0.2,
         1.0
      ],
      'visible':True,
      'carrying':[  

      ],
      'left_hand_empty':True,
      'velocity':[  
         -1.573153870259355e-06,
         -0.00010299083286994883,
         0.0010427399075212146,
         -8.109990792594981e-06,
         -0.006121071218324431,
         1.6958155964482204e-06
      ],
      'orientation_rpq':[  
         -1.4757321346264476e-08,
         -1.0441566451482611e-06,
         1.8338191209304903e-06
      ],
      'type':'agent',
      'location_xyz':[  
         -0.5000014923402628,
         -9.649642775266636e-05,
         -2.4071848300777354e-07
      ]
   }
}


operator = 'pick_up'

action_dictionary = {
   "pick_up": {
        "move_to": {
            "failure_conditions": [
                {
                    'movo':
                        {
                            'right_hand_empty': True
                        }
                }
            ],
        "parameterization": ["Target"]
        }
    }
}

mapper = FailureMapper(resolution_actions = action_dictionary)

print(mapper._map_failure_to_resolution(world_state, operator))