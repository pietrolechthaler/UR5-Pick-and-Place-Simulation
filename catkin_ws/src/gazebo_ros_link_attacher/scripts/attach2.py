#!/usr/bin/env python
import sys
import rospy
from gazebo_ros_link_attacher.srv import Attach, AttachRequest, AttachResponse

    
   
if __name__ == '__main__':
    class_to_attach=""
    obj1=sys.argv[1]
    obj2=sys.argv[2]
    
    rospy.init_node('demo_attach_links')
    attach_srv = rospy.ServiceProxy('/link_attacher_node/attach',
                                    Attach)
    attach_srv.wait_for_service()
    
    # Link them
    rospy.loginfo("Attaching objects")
    req = AttachRequest()
    req.model_name_1 = obj1
    req.link_name_1 = "link"
    req.model_name_2 = obj2
    req.link_name_2 = "link"
    attach_srv.call(req)
    # From the shell:
    """
rosservice call /link_attacher_node/attach "model_name_1: 'cube1'
link_name_1: 'link'
model_name_2: 'cube2'
link_name_2: 'link'"
    """
'''
    rospy.loginfo("Attaching cube2 and cube3")
    req = AttachRequest()
    req.model_name_1 = "cube2"
    req.link_name_1 = "link"
    req.model_name_2 = "cube3"
    req.link_name_2 = "link"

    attach_srv.call(req)

    rospy.loginfo("Attaching cube3 and cube1")
    req = AttachRequest()
    req.model_name_1 = "cube3"
    req.link_name_1 = "link"
    req.model_name_2 = "cube1"
    req.link_name_2 = "link"

    attach_srv.call(req)
'''
