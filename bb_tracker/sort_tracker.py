import rclpy
import numpy as np

from rclpy.node import Node
from std_msgs.msg import Header
from agrirobot_msgs.msg import PolygonArrayStamped
from geometry_msgs.msg import Polygon, Point, Point32, PointStamped
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, \
    QoSDurabilityPolicy
from .sort import *
from .marker_pub import *


_TRACKER_NODE_NAME = "bb_2d_tracker"
_TRACKER_SUB_TOPIC = "/tractor/radar/dynamic/obstacle_detection/clustering/bb_2d"
_TRACKER_PUB_TOPIC = "/tractor/radar/dynamic/obstacle_detection/tracking/bb_2d"


class BBTracker(Node):
    def __init__(self):
        super().__init__(_TRACKER_NODE_NAME)
        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=5)

        self.subscriber = self.create_subscription(
            PolygonArrayStamped, _TRACKER_SUB_TOPIC, self.bb_2d_callback, qos_profile)

        self.tracker_publisher = self.create_publisher(
            PolygonArrayStamped, _TRACKER_PUB_TOPIC, qos_profile)

        self.center_publisher = self.create_publisher(
            PointStamped, _TRACKER_PUB_TOPIC+"/center", qos_profile)

        self.marker_pubs = MarkerPub()

        # create instance of SORT
        freq = 5   # arbitrary
        min_hits = 3    # no. of iterations
        max_age = 10    # in seconds
        self.sort_tracker = Sort(max_age=max_age*freq,min_hits=min_hits, max_speed=2.0, 
                                 iou_threshold=0.01, dt=1/freq)
        self.sort_timer = self.create_timer(1/freq, self.sort_callback)

        # init variables
        self.polygons = []
        self.frame_id = ''

    def polygon_to_bb(self, polygon: Polygon) -> list[float]:
        """
        Converts a 2D polygon to a 2D bounding box.

        Parameters
        -------
        polygon : Polygon
            Polygon with 4 Points

        Return
        ------
        bb : list[float]
            [x1,y1,x2,y2] where x1,y1 is the top left and x2,y2 is the bottom right.
        """
        x = []
        y = []
        for point in polygon.points:
            x.append(point.x)
            y.append(point.y)
        x1 = min(x)
        x2 = max(x)
        y1 = min(y)
        y2 = max(y)
        return [x1, y1, x2, y2]

    def bb_to_polygon(self, bb) -> Polygon:
        """
        Concerts a 2D bounding box to a 2D Polygon.

        Parameters
        -------
        bb : list[float]
            Array [x1,y1,x2,y2,score]

        Returns
        --------
        polygon : Polygon
            Polygon comprised of 4 Points taken clockwise
        """
        x1 = bb[0]
        y1 = bb[1]
        x2 = bb[2]
        y2 = bb[3]

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

    def bb_to_point(self, bb) -> Point:
        """
        Extracts a 2D center point of a bounding box (z=0).

        Parameters
        -------
        bb : list[float]
            Array [x1,y1,x2,y2,score]

        Returns
        --------
        pt : Point
        """
        pt = Point()
        pt.x = (bb[0]+bb[2])/2
        pt.y = (bb[1]+bb[3])/2
        pt.z = 0.
        return pt

    def publish_trackers(self, detected_bbs, undetected_bbs, frame_id: str) -> None:
        '''
        Publish detected and undetected polygon arrays from SORT tracker as 
        PolygonArrayStamped
        '''
        polygons = []
        pt = Point()

        header = Header()
        header.stamp = Node.get_clock(self).now().to_msg()
        header.frame_id = frame_id

        if len(detected_bbs) > 0:
            for bb in detected_bbs:
                polygon = self.bb_to_polygon(bb)
                polygons.append(polygon)
                pt = self.bb_to_point(bb)

        if len(undetected_bbs) > 0:
            for bb in undetected_bbs:
                polygon = self.bb_to_polygon(bb)
                polygons.append(polygon)
                # pt = self.bb_to_point(bb)

        msg = PolygonArrayStamped()
        msg.header = header
        msg.polygons = polygons
        self.tracker_publisher.publish(msg)

        msg = PointStamped()
        msg.point = pt
        self.center_publisher.publish(msg)

    def bb_2d_callback(self, msg: PolygonArrayStamped) -> None:
        """
        Updates polygons variable when a detection occurs.
        """
        self.polygons = msg.polygons
        self.frame_id = msg.header.frame_id

    def sort_callback(self) -> None:
        """
        Feeds detections to the SORT tracker and publishes bb trackers.
        """
        undetected_ids = []
        detected_ids = []
        if len(self.polygons) == 0:
            self.get_logger().info(f'-------- No detections --------')
            undetected_bbs = self.sort_tracker.update_without_detections()
            undetected_ids = [id for [_, _, _, _, id] in undetected_bbs]
            # Publish trackers
            self.publish_trackers([], undetected_bbs, self.frame_id)
            # Publisher markers
            self.marker_pubs.publish_tracker_markers(
                [], undetected_bbs, self.frame_id)

        else:
            self.get_logger().info(f'--------- Detected --------')
            bboxes = []
            for polygon in self.polygons:
                bb = self.polygon_to_bb(polygon)
                bb.append(0)        # add dummy score
                bboxes.append(bb)
            bboxes = np.array(bboxes)
            # Update Sort with the detections
            detected_bbs, undetected_bbs = self.sort_tracker.update(bboxes)
            undetected_ids = [id for [_, _, _, _, id] in undetected_bbs]
            detected_ids = [id for [_, _, _, _, id] in detected_bbs]
            # Publish trackers
            self.publish_trackers(detected_bbs, undetected_bbs, self.frame_id)
            # Publisher markers
            self.marker_pubs.publish_tracker_markers(
                detected_bbs, undetected_bbs, self.frame_id)

            # Reset
            self.polygons = []

        self.get_logger().info(f"detected_ids: {detected_ids}")
        self.get_logger().info(f"undetected_ids: {undetected_ids}")

        # Remove dead trackers from RVIZ
        live_ids = []
        for tracker in self.sort_tracker.trackers:
            live_ids.append(tracker.id)
        dead_ids = [id_ for id_ in range(self.marker_pubs.n) if id_ not in live_ids]
        self.marker_pubs.clear_trackers(dead_ids)

        if len(self.sort_tracker.trackers) == 0:
            # Reset counter if there are no trackers
            self.get_logger().info("Restarting ID count from 0")
            KalmanBoxTracker.count = 0

def main(args=None):
    rclpy.init(args=args)

    tracker = BBTracker()
    try:
        rclpy.spin(tracker)
    except KeyboardInterrupt:
        pass

    tracker.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
