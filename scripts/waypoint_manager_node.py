#!/usr/bin/python3

from dataclasses import dataclass

import rospy
import yaml
from geometry_msgs.msg import Point, Quaternion, Vector3
from std_msgs.msg import ColorRGBA
from tf.transformations import quaternion_from_euler
from visualization_msgs.msg import Marker, MarkerArray


@dataclass(frozen=True)
class Params:
    frame_id: str = "map"
    waypoint_file: str = "waypoints.yaml"
    hz: int = 1
    width_ratio: float = 1.0
    is_visible_text: bool = True
    is_visible_edge: bool = True

    def print(self) -> None:
        rospy.loginfo(f"frame_id: {self.frame_id}")
        rospy.loginfo(f"waypoint_file: {self.waypoint_file}")
        rospy.loginfo(f"hz: {self.hz}")
        rospy.loginfo(f"width_ratio: {self.width_ratio}")
        rospy.loginfo(f"is_visible_text: {self.is_visible_text}")
        rospy.loginfo(f"is_visible_edge: {self.is_visible_edge}")


class WaypointManager:
    def __init__(self) -> None:
        rospy.init_node("waypoint_manager")
        self._params: Params = Params(
            frame_id=rospy.get_param("~frame_id", "map"),
            waypoint_file=rospy.get_param("~waypoint_file", "waypoints.yaml"),
            hz=rospy.get_param("~hz", 1),
            width_ratio=rospy.get_param("~width_ratio", 1.0),
            is_visible_text=rospy.get_param("~is_visible_text", True),
            is_visible_edge=rospy.get_param("~is_visible_edge", True),
        )

        self._waypoint_pub = rospy.Publisher(
            "waypoints", MarkerArray, queue_size=1, latch=True
        )

        # Print parameters
        rospy.loginfo(f"{rospy.get_name()} node has started...")
        rospy.loginfo("")
        rospy.loginfo("Parameters:")
        self._params.print()

    def process(self) -> None:
        with open(self._params.waypoint_file, "r") as file:
            waypoints = yaml.safe_load(file)
        msg: MarkerArray = self.create_visualization(waypoints)

        r = rospy.Rate(self._params.hz)
        while not rospy.is_shutdown():
            self._waypoint_pub.publish(msg)
            r.sleep()

    def create_visualization(self, waypoints) -> MarkerArray:
        msg = MarkerArray()
        for waypoint in waypoints:
            # Draw waypoints
            arrow_maker = self.create_maker(
                id=waypoint["id"],
                type=Marker.ARROW,
                scale=Vector3(
                    self._params.width_ratio * 0.7,
                    self._params.width_ratio * 0.2,
                    self._params.width_ratio * 0.2,
                ),
                rgba=ColorRGBA(0.88, 0.0, 1.0, 1.0),
                yaw=waypoint["yaw"],
                point=Point(waypoint["x"], waypoint["y"], 0.0),
            )
            msg.markers.append(arrow_maker)

            # Draw text
            if self._params.is_visible_text:
                test_maker = self.create_maker(
                    id=100 + waypoint["id"],
                    type=Marker.TEXT_VIEW_FACING,
                    scale=Vector3(
                        self._params.width_ratio,
                        self._params.width_ratio,
                        self._params.width_ratio,
                    ),
                    rgba=ColorRGBA(0.0, 0.0, 0.0, 1.0),
                    point=Point(
                        waypoint["x"], waypoint["y"], self._params.width_ratio * 0.5
                    ),
                )
                test_maker.text = str(waypoint["id"])
                msg.markers.append(test_maker)

        # Draw lines
        if self._params.is_visible_edge:
            for i in range(len(waypoints) - 1):
                line_maker = self.create_maker(
                    id=200 + i,
                    type=Marker.LINE_STRIP,
                    scale=Vector3(
                        self._params.width_ratio * 0.1,
                        0.0,
                        0.0,
                    ),
                    rgba=ColorRGBA(0.0, 0.0, 1.0, 0.5),
                )
                line_maker.points.append(Point(waypoints[i]["x"], waypoints[i]["y"], 0))
                line_maker.points.append(
                    Point(waypoints[i + 1]["x"], waypoints[i + 1]["y"], 0)
                )
                msg.markers.append(line_maker)

        return msg

    def create_maker(
        self,
        id: int,
        type: int,
        scale: Vector3,
        rgba: ColorRGBA,
        yaw: float = 0.0,
        point: Point = Point(),
    ) -> Marker:
        marker = Marker()
        marker.header.frame_id = self._params.frame_id
        marker.header.stamp = rospy.Time.now()
        marker.ns = "waypoints"
        marker.id = id
        marker.type = type
        marker.action = Marker.ADD
        marker.pose.position = point
        marker.pose.orientation = Quaternion(*quaternion_from_euler(0, 0, yaw))
        marker.scale = scale
        marker.color = rgba
        marker.lifetime = rospy.Duration()
        return marker


if __name__ == "__main__":
    try:
        node = WaypointManager()
        node.process()
    except rospy.ROSInterruptException:
        rospy.loginfo("Exception caught")
        pass
