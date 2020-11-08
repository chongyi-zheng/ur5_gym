import rospy


class MujocoROS:
    def __init__(self, node_name):
        self.node_name = node_name

        rospy.init_node(self.node_name, anonymous=True)



