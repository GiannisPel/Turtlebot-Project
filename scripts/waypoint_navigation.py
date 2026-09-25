#!/usr/bin/env python3
"""Send an ordered list of target poses to the ROS move_base action server."""

import math
import sys

import actionlib
import rospy
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


STATUS_NAMES = {
    GoalStatus.PENDING: "PENDING",
    GoalStatus.ACTIVE: "ACTIVE",
    GoalStatus.PREEMPTED: "PREEMPTED",
    GoalStatus.SUCCEEDED: "SUCCEEDED",
    GoalStatus.ABORTED: "ABORTED",
    GoalStatus.REJECTED: "REJECTED",
    GoalStatus.PREEMPTING: "PREEMPTING",
    GoalStatus.RECALLING: "RECALLING",
    GoalStatus.RECALLED: "RECALLED",
    GoalStatus.LOST: "LOST",
}


class WaypointNavigator:
    """Execute configured waypoints sequentially through move_base."""

    def __init__(
        self,
        action_name,
        frame_id,
        server_timeout,
        goal_timeout,
        continue_on_failure,
    ):
        self.frame_id = frame_id
        self.server_timeout = server_timeout
        self.goal_timeout = goal_timeout
        self.continue_on_failure = continue_on_failure
        self.client = actionlib.SimpleActionClient(action_name, MoveBaseAction)
        rospy.on_shutdown(self.client.cancel_all_goals)

    def wait_for_server(self):
        """Wait for move_base until the configured timeout expires."""
        rospy.loginfo(
            "Waiting up to %.1f seconds for the move_base action server...",
            self.server_timeout,
        )
        available = self.client.wait_for_server(rospy.Duration(self.server_timeout))
        if available:
            rospy.loginfo("Connected to the move_base action server.")
        return available

    def execute(self, waypoints):
        """Send every waypoint and return True only if all goals succeed."""
        all_succeeded = True

        for index, waypoint in enumerate(waypoints, start=1):
            if rospy.is_shutdown():
                return False

            try:
                name, goal, x, y, yaw = self._create_goal(waypoint, index)
            except (TypeError, ValueError) as error:
                rospy.logerr("Invalid waypoint %d: %s", index, error)
                return False

            rospy.loginfo(
                "Sending waypoint %d/%d '%s' (x=%.2f, y=%.2f, yaw=%.3f rad)",
                index,
                len(waypoints),
                name,
                x,
                y,
                yaw,
            )
            self.client.send_goal(goal)

            finished = self.client.wait_for_result(rospy.Duration(self.goal_timeout))
            if not finished:
                self.client.cancel_goal()
                rospy.logerr(
                    "Waypoint '%s' timed out after %.1f seconds.",
                    name,
                    self.goal_timeout,
                )
                all_succeeded = False
                if not self.continue_on_failure:
                    return False
                continue

            state = self.client.get_state()
            if state == GoalStatus.SUCCEEDED:
                rospy.loginfo("Waypoint '%s' reached successfully.", name)
                continue

            status_name = STATUS_NAMES.get(state, "UNKNOWN")
            status_text = self.client.get_goal_status_text() or "no status message"
            rospy.logerr(
                "Waypoint '%s' failed with state %s: %s",
                name,
                status_name,
                status_text,
            )
            all_succeeded = False
            if not self.continue_on_failure:
                return False

        return all_succeeded

    def _create_goal(self, waypoint, index):
        """Validate a waypoint and convert it into a MoveBaseGoal."""
        if not isinstance(waypoint, dict):
            raise TypeError("each waypoint must be a dictionary")

        try:
            x = float(waypoint["x"])
            y = float(waypoint["y"])
            yaw = float(waypoint.get("yaw", 0.0))
        except KeyError as error:
            raise ValueError("missing required field: {}".format(error.args[0]))
        except (TypeError, ValueError):
            raise ValueError("x, y, and yaw must be numeric")

        if not all(math.isfinite(value) for value in (x, y, yaw)):
            raise ValueError("x, y, and yaw must be finite numbers")

        name = str(waypoint.get("name", "waypoint_{}".format(index)))
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = self.frame_id
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.target_pose.pose.orientation.w = math.cos(yaw / 2.0)

        return name, goal, x, y, yaw


def positive_float_param(name, default):
    """Read and validate a positive floating-point ROS parameter."""
    value = float(rospy.get_param(name, default))
    if not math.isfinite(value) or value <= 0.0:
        raise ValueError("{} must be a positive finite number".format(name))
    return value


def main():
    rospy.init_node("turtlebot3_waypoint_navigation")

    waypoints = rospy.get_param("~waypoints", [])
    if not isinstance(waypoints, list) or not waypoints:
        rospy.logfatal("Parameter '~waypoints' must be a non-empty list.")
        return 1

    try:
        navigator = WaypointNavigator(
            action_name=rospy.get_param("~move_base_action", "/move_base"),
            frame_id=rospy.get_param("~frame_id", "map"),
            server_timeout=positive_float_param("~server_timeout", 60.0),
            goal_timeout=positive_float_param("~goal_timeout", 180.0),
            continue_on_failure=bool(
                rospy.get_param("~continue_on_failure", False)
            ),
        )
    except (TypeError, ValueError) as error:
        rospy.logfatal("Invalid navigation configuration: %s", error)
        return 1

    if not navigator.wait_for_server():
        rospy.logfatal("move_base was not available before the timeout expired.")
        return 1

    if not navigator.execute(waypoints):
        rospy.logerr("Waypoint route did not complete successfully.")
        return 1

    rospy.loginfo("All waypoints reached successfully.")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except rospy.ROSInterruptException:
        pass
