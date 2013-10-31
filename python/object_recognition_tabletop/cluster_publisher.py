"""
Module defining the Cluster Publisher\
@Liang-Ting Jiang
"""

from ecto import BlackBoxCellInfo as CellInfo, BlackBoxForward as Forward
from object_recognition_core.io.sink import SinkBase
from object_recognition_ros import init_ros
import ecto
from object_recognition_tabletop.ecto_cells.tabletop_table import ClusterConverter
import ecto_ros.ecto_sensor_msgs

PointCloudPub = ecto_ros.ecto_sensor_msgs.Publisher_PointCloud2

########################################################################################################################

class ClusterPublisher(ecto.BlackBox, SinkBase):
    """
    Class publishing the different results of tabletop
    """
    def __init__(self, *args, **kwargs):
        init_ros()
        ecto.BlackBox.__init__(self, *args, **kwargs)
        SinkBase.__init__(self)

    @classmethod
    def declare_cells(cls, p):
        return {'cluster_converter': CellInfo(ClusterConverter),
                'pc_publisher': CellInfo(PointCloudPub, params={'latched': p.latched}),
                'passthrough': ecto.PassthroughN(items=dict(image_message='The original imagemessage',
                                                        pose_results='The final results'))
                }

    @staticmethod
    def declare_direct_params(p):
        p.declare('latched', 'Determines if the topics will be latched.', True)

    @staticmethod
    def declare_forwards(_p):
        p = {'pc_publisher': [Forward('topic_name', 'tabletop_cluster_topic',
                     'The ROS topic to use for the PointCloud2 message.', 'tabletop_cluster')]
             }

        i = {'cluster_converter': [Forward('clusters3d')], 
             'passthrough': [Forward('image_message'), Forward('pose_results')]}

        return (p,i,{})

    def connections(self, _p):
  
        connections = [ self.passthrough['image_message'] >> self.cluster_converter['image_message'],
                        self.cluster_converter['cluster_pc'] >> self.pc_publisher[:] ]
        return connections
