#!/usr/bin/env python

import rospy
import json
from dynamic_skill_injection_msgs.srv import FailureNotification

# Not strictly required to have all data types, only those that are affected by the current task operator.
effect = {
    u'movo': 
        {
            u'location_str': [('in', 'red_5m_room')], 
            u'orientation_rpq': [-6.599567256198967e-09, 5.255610232621944e-06, 2.5162805461869586e-05], 
            u'location_xyz': [-0.4907107396040672, 5.060855618075925, -1.3289987767228784e-06], 
            u'velocity': [-1.8916128783190173e-06, -0.00013714909174842698, -0.0010479390560907522, 8.011678144514673e-06, 0.006460918545800547, 2.568022993246033e-06]
        }
    }


# Not strictly required to have all data types, only those that are affected by the current task operator.
world_state = {
    u'cafe_beer': 
        {
            u'location_str': 
                [[u'in', u'folding_table_4x2'], [u'in', u'red_5m_room']], 
                u'dimensions': [0.066, 0.066, 0.122], 
                u'visible': True, 
                u'velocity': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 
                u'orientation_rpq': [-0.0010953467905048702, 0.0005336966633416002, -2.905181720484242e-07], 
                u'type': u'container', 
                u'location_xyz': [0.8786726105705384, 5.383354107952973, 0.4285110216314646]
        },
    u'ground_plane': 
        {
            u'velocity': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 
            u'_comment': 
            u'Bullshit', 
            u'orientation_rpq': [0.0, -0.0, 0.0], 
            u'type': 
            u'none', 
            u'location_xyz': [0.0, 0.0, 0.0]
        }, 
    u'movo': 
        {
            u'right_hand_empty': [u'True'], 
            u'location_str': [[u'in', u'red_5m_room']], 
            u'dimensions': [0.4, 0.2, 1.0], 
            u'visible': u'True', 
            u'carrying': [u'None'], 
            u'left_hand_empty': [u'True'], 
            u'velocity': [-1.6107279817900635e-06, -0.0001232466188071857, 0.0010479836455093226, -6.847722887102694e-06, -0.006460933546287295, 1.6820581433852563e-06], 
            u'orientation_rpq': [-1.4790662860104751e-08, -1.2052738466292656e-06, 0.00020765243992752094], 
            u'type': u'agent', 
            u'location_xyz': [-0.49086115839172206, 5.049655676349373, -2.810418510135193e-07]
        }, 
    u'folding_table_4x2': 
        {
            u'location_str': [[u'in', u'red_5m_room']], 
            u'dimensions': [1.0, 1.0, 0.5], 
            u'_comment': u'dimensions are wrong need to scrape stl', 
            u'location_xyz': [0.8196784853935242, 5.2346320152282715, 0.4285709857940674], 
            u'velocity': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 
            u'orientation_rpq': [0.0, -0.0, 0.0], 
            u'type': u'static', 
            u'visibile': u'True'
        }, 
    u'blue_5m_room': 
        {
            u'location_str': [], 
            u'door': u'unconfigured possibly location?', 
            u'dimensions': [5.0, 5.0, 2.5], 
            u'velocity': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 
            u'orientation_rpq': [0.0, -0.0, 0.0], 
            u'type': u'room', 
            u'location_xyz': [0.0, 2.220446049250313e-16, 0.0]
        }, 
    u'red_5m_room': 
        {
            u'location_str': [], 
            u'door': u'unconfigured possibly location?',
            u'dimensions': [5.0, 5.0, 2.5], 
            u'velocity': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 
            u'orientation_rpq': [0.0, -0.0, 0.0], 
            u'type': u'room', 
            u'location_xyz': [0.009309999999999707, 5.0624, 0.0]
        }
    }

def failure_notification_client(effect, world_state):
    rospy.wait_for_service('failure_notification_server')
    try:
        rospy.init_node('failure_notification_client')
        failure_notifier = rospy.ServiceProxy('failure_notification_server', FailureNotification)
        response = failure_notifier(effect, world_state)
        return response
    except rospy.ServiceException as e:
        print "Service call failed: {}".format(e)


if __name__ == "__main__":
    try:
        effect = json.dumps(effect)
        world_state = json.dumps(world_state)
        response = failure_notification_client(effect, world_state)
        rospy.loginfo(response)
    except rospy.ROSInterruptException:
        pass