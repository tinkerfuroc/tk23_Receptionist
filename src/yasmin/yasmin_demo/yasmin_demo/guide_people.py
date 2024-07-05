import rclpy
import yasmin
from geometry_msgs.msg import Pose
from nav2_msgs.action import NavigateToPose
from yasmin import CbState
from yasmin import Blackboard
from yasmin import StateMachine
from yasmin_ros import ActionState
from yasmin_ros.basic_outcomes import SUCCEED, ABORT, CANCEL
from yasmin_viewer import YasminViewerPub
from tinker_vision_msgs import ObjectDetection
HAS_NEXT = "has_next"
END = "end"


class GuidePeople:
    def __init__(self, beacon1, beacon2, person=None) -> None:
        self.person = person
        self.init_pose = "entrance"
        self.beacons = []
        self.beacons[0] = self.init_pose
        self.beacons[1] = beacon1
        self.beacons[2] = beacon2
        
def create_nav_points(blackboard: Blackboard) -> str:
    blackboard.waypoints = {
        "entrance": [1.25, 6.30, -0.78, 0.67],
        "bathroom": [4.89, 1.64, 0.0, 1.0],
        "livingroom": [1.55, 4.03, -0.69, 0.72],
        "kitchen": [3.79, 6.77, 0.99, 0.12],
        "bedroom": [7.50, 4.89, 0.76, 0.65],
    }
    return SUCCEED

def get_beacon_waypoint(blackboard: Blackboard, beacons) -> str:
    blackboard.waypoints = []
    for beacon in beacons:
        waypoint = blackboard.waypoints[beacon]
        blackboard.waypoints.append(waypoint)
        pose = Pose()
        pose.position.x = waypoint[0]
        pose.position.y = waypoint[1]
        pose.orientation.z = waypoint[2]
        pose.orientation.w = waypoint[3]
    return SUCCEED

def get_next_waypoint(blackboard: Blackboard) -> str:
    if not blackboard.waypoints:
        return END
    blackboard.to_pose = blackboard.waypoints.pop(0)
    return SUCCEED

class MoveState(ActionState):
    def __init__(self):
        super().__init__(
            NavigateToPose,
            "/navigate_to_pose",
            self.create_goal_handler,
            None,
            None
        )
    
    def create_goal_handler(self, blackboard: yasmin.Blackboard) -> NavigateToPose.Goal:
        goal = NavigateToPose.Goal()
        goal.pose.pose = blackboard.pose
        goal.pose.header.frame_id = "map"
        return goal

class GetObjectState(ServiceState):
    def __init__(self) -> None:
        super().__init__(
            ObjectDetection,  # srv type
            "/object_detection",  # service name
            self.create_request_handler,  # cb to create the request
            ["outcome1"],  # outcomes. Includes (SUCCEED, ABORT)
            self.response_handler  # cb to process the response
        )

    def create_request_handler(self, blackboard: Blackboard) -> AddTwoInts.Request:

        req = ObjectDetection.Request()
        req.mode = 
        return req

    def response_handler(
        self,
        blackboard: Blackboard,
        response: ObjectDetection.Response
    ) -> str:

        objs = response.Object
        blackboard.object_pos = objs[0].centroid
        
        return "outcome1"

def main():
    
    rclpy.init()
    
    guide_sm = StateMachine(outcomes=[SUCCEED, ABORT, CANCEL])
    
    guide_sm.add_state(
        "CREATE_WAYPOINTS",
        CbState([SUCCEED], create_nav_points),
        transitions={SUCCEED: "GET_NEXT_WAYPOINT"}
    )
    
    guide_sm.add_state(
        "GET_NEXT_WAYPOINT",
        CbState([HAS_NEXT, END], get_next_waypoint),
        transitions={HAS_NEXT: "MOVE_TO_WAYPOINT", END: SUCCEED}
    )
    
    guide_sm.add_state(
        "MOVE_TO_WAYPOINT",
        MoveState(),
        transitions={
            SUCCEED: "GET_NEXT_WAYPOINT",
            CANCEL: CANCEL,
            ABORT:ABORT
        }
    )
    
    guide_sm.add_state(
        "FIND_PEOPLE",
        GetObjectState(),
        transitions={
            'outcome1': "MOVE_TO_WAYPOINT"
        }
    )

    YasminViewerPub("YASMIN_NAV_DEMO",guide_sm)
    
    blackboard = Blackboard()
    
    outcome = guide_sm(blackboard)
    
    print(outcome)
    
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()