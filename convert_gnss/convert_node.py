import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf2_msgs.msg import TFMessage
import csv
import math

class OdomToPoseConverter(Node):
    def __init__(self):
        super().__init__('odom_to_pose_converter')
        
        self.target_frame = 'base_link'
        self.source_frame = 'map'
        self.csv_filename = 'Navette_traj.csv'
        self.init_csv()

        #frequence = 1/record_interval
        self.record_interval = 0.1  #(second) 10hz fréquence de enregistrement le traj 
        self.last_record_time = 0.0  #

        self.subscription = self.create_subscription(
            Odometry,
            '/localization/kinematic_state',  # topic origine
            self.listener_callback,
            100)
        
        self.tf_sub = self.create_subscription(
            TFMessage,
            '/tf',
            self.tf_callback,  
            5)
            
        self.publisher_ = self.create_publisher(
            PoseWithCovarianceStamped,
            '/localization/pose_with_covariance_stamped', # topic apres
            100)
        
    def init_csv(self):
        with open(self.csv_filename, mode='w', newline='') as f:
            writer = csv.writer(f)
            # yaw en radien
            writer.writerow(['x', 'y', 'z', 'yaw'])

    def quaternion_to_yaw(self, x, y, z, w):

        # sin(yaw) * cos(pitch) and cos(yaw) * cos(pitch)
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return yaw  

    def tf_callback(self, msg):
        for transform in msg.transforms:
            # matching frame_id
            if (transform.header.frame_id == self.source_frame and 
                transform.child_frame_id == self.target_frame):
                
                stamp = transform.header.stamp
                current_time = stamp.sec + stamp.nanosec * 1e-9
                
                if (current_time - self.last_record_time) < self.record_interval:
                    continue
                # timestamp = transform.header.stamp.sec + transform.header.stamp.nanosec * 1e-9
                
                # translation
                x = transform.transform.translation.x
                y = transform.transform.translation.y
                z = transform.transform.translation.z
                
                #  rotation Yaw
                rx = transform.transform.rotation.x
                ry = transform.transform.rotation.y
                rz = transform.transform.rotation.z
                rw = transform.transform.rotation.w
                
                yaw_rad = self.quaternion_to_yaw(rx, ry, rz, rw)
                
                # 4. csv
                with open(self.csv_filename, mode='a', newline='') as f:
                    writer = csv.writer(f)
                    writer.writerow([x, y, z, yaw_rad])

                self.last_record_time = current_time
                break 

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
