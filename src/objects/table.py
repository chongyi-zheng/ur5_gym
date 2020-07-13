from geometry_msgs.msg import PoseStamped

from src.objects.base import Object


class BoxTable(Object):
    """BoxTable object"""
    def __init__(self, frame='world'):
        # Add table to moveit
        init_pose = PoseStamped()
        init_pose.header.frame_id = frame
        init_pose.pose.position.x = 0.6
        init_pose.pose.position.y = 0.0
        init_pose.pose.position.z = 0.4
        init_pose.pose.orientation.x = 0
        init_pose.pose.orientation.y = 0
        init_pose.pose.orientation.z = 0
        init_pose.pose.orientation.w = 1.0

        primitive_attrs = {'size': (0.9, 0.9, 0.85)}

        Object.__init__(self, init_pose, name='table', primitive_attrs=primitive_attrs)
