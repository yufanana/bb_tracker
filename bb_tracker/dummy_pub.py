import rclpy
import numpy as np

from rclpy.node import Node
from agrirobot_msgs.msg import PolygonArrayStamped
from geometry_msgs.msg import Polygon, Point32, PointStamped, PolygonStamped
from std_msgs.msg import Header
from .sort import *
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, \
    QoSDurabilityPolicy

_DUMMY_PUB_GT_TOPIC = "/tractor/radar/dynamic/obstacle_detection/clustering/bb_2d_gt"
_DUMMY_PUB_TOPIC = "/tractor/radar/dynamic/obstacle_detection/clustering/bb_2d"
_DUMMY_NODE_NAME = "dummy_pub"


class DummyPub(Node):
    def __init__(self):
        super().__init__(_DUMMY_NODE_NAME)
        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=5)

        # dummy data publisher
        self.dummy_publisher = self.create_publisher(
            PolygonArrayStamped, _DUMMY_PUB_TOPIC, qos_profile)
        
        self.dummyc_publisher = self.create_publisher(
            PointStamped, _DUMMY_PUB_TOPIC+"/center", qos_profile)

        # ground truth publisher
        self.gt_publisher = self.create_publisher(
            PolygonStamped, _DUMMY_PUB_GT_TOPIC, qos_profile)
        
        self.gtc_publisher = self.create_publisher(
            PointStamped, _DUMMY_PUB_GT_TOPIC+"/center", qos_profile)

        # timer to generate path
        self.freq = 10   # arbitrary
        self.timer = self.create_timer(1/self.freq, self.timer_callback)
        self.initial_x = -5.0
        self.initial_y = 5.0

        # counters
        self.path_i = 0.0
        self.pub_i = 0

        # for dynamic obstacle
        self.speed_i = 0.0
        self.path_x = -5.0

        # flags to create dynamic obstacle
        self.constant = True
        self.acc = 1      # -1 means decelerate
        self.direction = 1

        # BB Parameters
        self.bb_width = 1.0
        self.bb_length = 0.7

    def point_to_polygon(self, center: np.ndarray) -> Polygon:
        """
        Takes a bb center point [x,y] and return a Polygon with 4 points clockwise
        """
        # Add noise to bbox width and length
        w = self.bb_width + np.random.normal(0, 0.1)
        l = self.bb_length + np.random.normal(0, 0.1)

        # Obtain 4 corner points
        x1 = center[0] - w/2.0
        x2 = center[0] + w/2.0
        y1 = center[1] - l/2.0
        y2 = center[1] + l/2.0

        # Create 4 points to define Polygon
        polygon = Polygon()
        x = [x1, x2, x2, x1]
        y = [y1, y1, y2, y2]
        for i in range(len(x)):
            point = Point32()
            point.x = x[i]
            point.y = y[i]
            point.z = 0.0
            polygon.points.append(point)
        return polygon
    
    def publish_all(self) -> bool:
        return True
    
    def publish_randomly(self, pub_rate=0.3) -> bool:
        """
        Publish randomly based on a uniform distribution.
        
        pub_rate : float
            Percentage of publish
        """
        if np.random.uniform(0.0, 1.0) < pub_rate:
            return True
        
        return False

    def publish_regular_intervals(self) -> bool:
        """
        Skip publishing at regular intervals.
        """
        self.pub_i += 1
        
        if self.pub_i > 5 and self.pub_i < 10:
            return False
        
        if self.pub_i == 10:
            # reset counter
            self.pub_i = 0
        
        return True

    def publish_occlusion(self, seen=3.0, hidden=3.0) -> bool:
        """
        Publish with simulated occlusion with seen and hidden durations
        specified in seconds.

        |---> time
        <---seen---><--hidden-->
        |<---t1--->|<----t2--->|
        """
        self.pub_i += 1
        t1 = seen
        t2 = hidden

        if self.pub_i == (t1+t2)*self.freq:
            self.pub_i = 0
        if self.pub_i > t1*self.freq:
            return False
        return True

    def get_circle_path(self) -> np.ndarray:
        """
        Object follows a circular path around the origin.
        """
        path_radius = 5.0
        self.path_i += 0.02
        bb_center = [path_radius *
                     np.sin(self.path_i), path_radius*np.cos(self.path_i)]
        return np.array(bb_center)

    def get_sine_path(self) -> np.ndarray:
        """
        Object oscillates along y-axis and has constant speed in x-axis.
        """
        # Sine path
        self.path_i += 0.03
        bb_center = [self.initial_x+self.path_i, self.initial_y+1.5*np.sin(self.path_i)]
        return np.array(bb_center)

    def get_straight_path(self) -> np.ndarray:
        """
        Object follows a straight path.
        """
        # Straight line path
        self.path_i += 0.03
        bb_center = [self.initial_x+self.path_i, self.initial_y]
        return np.array(bb_center)
    
    def get_start_stop_path(self) -> np.ndarray:
        """
        Path where the object accelerates and decelerates cyclically.

        |---> time
        <--const--><--accel/decel--><--const-->
        |<---t1--->|<------t2----->|<---t3---->|
        """

        self.path_i += 0.1
        
        # Time profile
        t1 = 2.     # in seconds, constant speed
        t2 = 4.     # in seconds, accel/decel
        t3 = 2.     # in seconds, constant speed

        # Speed profile
        if self.path_i < t1:
            self.constant = True
        elif self.path_i < t1+t2:
            self.constant = False
        elif self.path_i < t1+t2+t3:
            self.constant = True

        if self.path_i >= t1+t2+t3:
            # Reset and flip acceleration
            self.acc = -self.acc
            self.path_i = 0.
            
        # Calculate speed
        acc = 0.2      #  m/s
        max_speed = 1.5
        if self.constant is False:
            if self.acc == 1:
                self.speed_i = min(self.speed_i+acc/self.freq, max_speed)
            elif self.acc == -1:
                self.speed_i = max(0, self.speed_i-acc/self.freq)

        # Calculate path positions
        dt = 1/self.freq

        # Flip the object's direction if it is too far
        if abs(self.path_x) > 6.0:
            self.direction = -self.direction    # -1 or 1

        self.path_x = self.path_x + self.direction*self.speed_i*dt
        bb_center = [self.path_x, 5.0]
        return np.array(bb_center)

    def timer_callback(self):
        ## Generate paths
        # bb_center = self.get_circle_path()
        bb_center = self.get_straight_path()
        # bb_center = self.get_sine_path()
        # bb_center = self.get_start_stop_path()

        # Add noise to bb_center
        sd = 0.3
        noise = np.random.normal(0, sd, 2)
        noisy_polygon = self.point_to_polygon(bb_center+noise)

        ## Publish dummy detections
        # publish = self.publish_randomly(pub_rate=0.3)
        # publish = self.publish_regular_intervals()
        publish = self.publish_occlusion(seen=3., hidden=3.)
        # publish = self.publish_all()

        # Noisy detection data
        polygons = []
        polygons.append(noisy_polygon)
        msg = PolygonArrayStamped()
        header = Header()
        header.stamp = Node.get_clock(self).now().to_msg()
        header.frame_id = 'base_link'
        msg.header = header

        if publish:
            msg.polygons = polygons
            self.dummy_publisher.publish(msg)
            # print(f"Publishing: {polygons}")
        else:
            pass # print(f"Not publishing")

        # Ground truth
        polygons = []
        polygon = self.point_to_polygon(bb_center)
        polygons.append(polygon)
        gt_msg = PolygonStamped()
        gt_msg.header = header
        gt_msg.polygon = polygon
        self.gt_publisher.publish(gt_msg)

        # Publish centers for plotting
        pt_msg = PointStamped()
        pt_msg.header = header
        pt_msg.point.x = bb_center[0]+noise[0]
        pt_msg.point.y = bb_center[1]+noise[1]
        pt_msg.point.z = 0.
        self.dummyc_publisher.publish(pt_msg)

        pt_msg.point.x = bb_center[0]
        pt_msg.point.y = bb_center[1]
        pt_msg.point.z = 0.
        self.gtc_publisher.publish(pt_msg)


def main(args=None) -> None:
    rclpy.init(args=args)

    publisher = DummyPub()
    try:
        rclpy.spin(publisher)
    except KeyboardInterrupt:
        pass

    publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
