"""
ROS node to publish RVIZ markers for tracker based on their object IDs.
"""

import rclpy
import numpy as np

from rclpy.node import Node
from std_msgs.msg import Header
from geometry_msgs.msg import Polygon, PolygonStamped, Point32, PointStamped, Point
from agrirobot_msgs.msg import PolygonArrayStamped
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, \
    QoSDurabilityPolicy


_NODE_NAME = "tracker_marker_pub"
_PUB_TOPIC = "/tractor/radar/dynamic/obstacle_detection/tracking"

def bb_to_polygon(bb) -> Polygon:
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

def bb_to_point(bb) -> Point:
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

def polygon_to_bb(polygon: Polygon) -> list[float]:
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
    
    
class MarkerPub(Node):
    def __init__(self):
        super().__init__(_NODE_NAME)
        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=5)
        
        # Number of tracker topics to maintain
        self.n = 10

        # Detected trackers
        self.det_poly_pubs = []
        for i in range(self.n):
            self.det_poly_pubs.append(self.create_publisher(
            PolygonStamped, _PUB_TOPIC+"/detected/marker_poly"+str(i), qos_profile))

        # Undetected trackers
        self.undet_poly_pubs = []
        for i in range(self.n):
            self.undet_poly_pubs.append(self.create_publisher(
            PolygonStamped, _PUB_TOPIC+"/undetected/marker_poly"+str(i), qos_profile))            

        # Published for detected and undetected trackers
        self.point_pubs = []
        for i in range(self.n):
            self.point_pubs.append(self.create_publisher(
            PointStamped, _PUB_TOPIC+"/marker_point"+str(i), qos_profile))

        # Publish clustering bbox as a point
        _TRACKER_SUB_TOPIC = "/tractor/radar/dynamic/obstacle_detection/clustering/bb_2d"
        self.cluster_sub = self.create_subscription(PolygonArrayStamped, _TRACKER_SUB_TOPIC, 
                                                    self.cluster_callback, qos_profile)
        self.cluster_pubs = []
        for i in range(self.n):
            self.cluster_pubs.append(self.create_publisher(PointStamped, 
                                                           _TRACKER_SUB_TOPIC+"/marker_point"+str(i),
                                                           qos_profile))

    def cluster_callback(self, msg: PolygonArrayStamped):
        """
        Publish center points of clustering detections for plotting on PlotJuggler.
        """
        try:
            for i,polygon in enumerate(msg.polygons):  
                bb = polygon_to_bb(polygon)
                pt = bb_to_point(bb)

                header = Header()
                header.stamp = Node.get_clock(self).now().to_msg()
                header.frame_id = msg.header.frame_id
                point_msg = PointStamped()
                point_msg.point = pt
                point_msg.header = header
                self.get_logger().info(f"pt: {pt}")
                print(f"pt: {pt}")
                self.cluster_pubs[i].publish(point_msg)
        except:
            # No detections
            pass
      

    def publish_tracker_markers(self, detected_bbs, undetected_bbs, frame_id) -> None:
        """
        Publish detected and undetected bbs into the ID's topic for RVIZ.
        """
        header = Header()
        header.stamp = Node.get_clock(self).now().to_msg()
        header.frame_id = frame_id

        polygon_msg = PolygonStamped()
        point_msg = PointStamped()
        polygon_msg.header = header

        if len(detected_bbs) > 0:
            for bb in detected_bbs:
                polygon = bb_to_polygon(bb)
                polygon_msg.polygon = polygon
                pt = bb_to_point(bb)
                point_msg.point = pt
                id_ = int(bb[4])
                try:
                    self.det_poly_pubs[id_].publish(polygon_msg)
                    self.point_pubs[id_].publish(point_msg)
                except IndexError as e:
                    self.get_logger().error(f"{e}: Insufficient marker_pubs initialised")
                    print("Insufficient marker_pubs initialised")
                
        if len(undetected_bbs) > 0:
            for bb in undetected_bbs:
                polygon = bb_to_polygon(bb)
                polygon_msg.polygon = polygon
                pt = bb_to_point(bb)
                point_msg.point = pt
                id_ = int(bb[4])
                try:
                    self.undet_poly_pubs[id_].publish(polygon_msg)
                    self.point_pubs[id_].publish(point_msg)
                except IndexError as e:
                    self.get_logger().error(f"{e}: Insufficient marker_pubs initialised")
                    print("Insufficient marker_pubs initialised")

    def clear_trackers(self, ids) -> None:
        """
        Publish empty messages to clear polygons from RVIZ.
        TODO: Inefficient as most IDs are usually dead, should
        clear once the tracker dies.
        TODO: not working, polygons still visible on RVIZ
        """
        header = Header()
        header.stamp = Node.get_clock(self).now().to_msg()
        header.frame_id = ''
        msg = PolygonStamped()
        msg.header = header

        bb = [0.,0.,0.,0.]
        polygon = bb_to_polygon(bb)
        msg.polygon = polygon
        for id_ in ids:
            self.det_poly_pubs[id_].publish(msg)
            self.undet_poly_pubs[id_].publish(msg)


def main(args=None):
    rclpy.init(args=args)
    marker_pub = MarkerPub()
    try:
        rclpy.spin(marker_pub)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()
