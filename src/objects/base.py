
class Object:
    """Object base class"""
    def __init__(self, init_pose, name='object', mesh_path=None, primitive_attrs=None):
        """Initialize an Object in the environment

        Arguments
        ----------
        - init_pose: geometry_msgs.msg.PoseStamped
            Initial pose and reference frame of the object

        - name: str (default = 'object')
            Name of the object in MoveIt! planning scene

        - mesh_path: str or None (default = None)
            Path to the mesh file of the object

        - primitive_attrs: dict or None (default = None)
            When object is not a mesh, define the primitive attributes

        Returns
        ----------

        """
        self.init_pose = init_pose
        self.name = name

        if mesh_path is None and primitive_attrs is None:
            raise ValueError("Object must either a mesh or a primitive!")

        self.mesh_path = mesh_path
        self.primitive_attrs = primitive_attrs
