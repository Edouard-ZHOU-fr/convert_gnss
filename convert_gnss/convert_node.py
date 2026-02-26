import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped

class OdomToPoseConverter(Node):
    def __init__(self):
        super().__init__('odom_to_pose_converter')
        
        
        self.subscription = self.create_subscription(
            Odometry,
            '/localization/kinematic_state',  # topic origine
            self.listener_callback,
            100)
            
        self.publisher_ = self.create_publisher(
            PoseWithCovarianceStamped,
            '/localization/pose_with_covariance_stamped', # topic apres
            100)

    def listener_callback(self, msg):
        new_msg = PoseWithCovarianceStamped()
        

        new_msg.header = msg.header
        new_msg.pose = msg.pose
        
        self.publisher_.publish(new_msg)
        # self.get_logger().info('Publishing converted Pose...')

def main(args=None):
    rclpy.init(args=args)
    node = OdomToPoseConverter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
