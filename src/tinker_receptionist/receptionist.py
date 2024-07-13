#!/usr/bin/env python3
import rclpy
from tts_interfaces.action import TTSAction
from asr_communication_interface.action import Asrcommand
from tinker_vision_msgs.srv import FaceRegister, ObjectDetection
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
from yasmin import CbState
from yasmin import State
from yasmin import Blackboard
from yasmin import StateMachine
from yasmin_ros import ActionState,ServiceState
from yasmin_ros.basic_outcomes import SUCCEED, ABORT, CANCEL
from yasmin_viewer import YasminViewerPub
from nav2_msgs.action import NavigateToPose
import numpy as np

class Person:
    def __init__(self, name='Foo', drink='Foo', rec_info=None):
        self.name = name
        self.drink = drink
        self.rec_info = {} # face recognition info (include age and gender)

    def desp_gen(self):
        return f"This is {self.name}, {self.name}'s favourite drink is {self.drink}"
    
    def ensure_gen(self):
        return f"Hi! Your name is {self.name} and your favourite drink is {self.drink}, right?"

def set_tts(blackboard: Blackboard, target_string: str) -> str:
    print(target_string)
    blackboard.tts_target = target_string
    return True

###############################################################
##################### State Definition ########################
###############################################################

########### Init State ##########
class InitState(State):
    def __init__(self) -> None:
        super().__init__(["tts"])
        self.counter = 0

    def execute(self, blackboard: Blackboard) -> str:        
        print(f'Init Task')
        # persons: dictionary with id
        blackboard.persons = {}
        # active_persons: dictionary with host/guest1/guest2
        blackboard.active_persons = {}
        blackboard.active_rec_info = {}
        blackboard.active_id = None
        # locations: dictionary with coordinates
        blackboard.locations = {
                                "door": Pose(position=Point(x=-1.0319, y=4.5870 , z=0.0),orientation=Quaternion(x=0.0, y=0.0, z=0.7030, w= 0.7111)),  # registeration pose
                                "sofa": Pose(position=Point(x=2.9052, y=3.6829, z=0.0), orientation=Quaternion(x=0.0, y=0.0, z=0.6892, w=0.7245))   # observation pose
                                }
        blackboard.person_cnt = 0
        blackboard.active_location = blackboard.locations["door"]

        # transformation of the seat locations
        blackboard.seats_locations = [
            (2.1495, 5.6036, 0.0),   #left
            (2.9498, 5.5726, 0.0),   #middle
            (3.8365, 5.5462, 0.0),   #right
        ]
        blackboard.seats_occupied = [-1, -1, -1]
        blackboard.seats_observation_poses = [
            Pose(position=Point(x=2.9052, y=3.6829, z=0.0), orientation=Quaternion(x=0.0, y=0.0, z=0.6892, w=0.7245)),
            Pose(position=Point(x=0.0, y=0.0, z=0.0), orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)),
        ]

        set_tts(blackboard, "Hello! My name is Tinker. I'm going to welcome the guests!")
        blackboard.after_tts = 'register' # Register Host first

        blackboard.face_state = 0
        return "tts"

########### TTS State ##########
class TTSState(ActionState):
    def __init__(self) -> None:
        super().__init__(
            TTSAction,  # action type
            "/tts_command",  # action name
            self.create_goal_handler,  # cb to create the goal
            ['register', 'move', 'tts', 'done'],  # outcomes. 
            self.response_handler  # cb to process the response
        )

    def create_goal_handler(self, blackboard: Blackboard) -> TTSAction.Goal:
        print("TTS state")
        goal = TTSAction.Goal()
        goal.target_string = blackboard.tts_target
        return goal

    def response_handler(
        self,
        blackboard: Blackboard,
        response: TTSAction.Result
    ) -> str:

        if blackboard.after_tts == 'register':
            return 'register'
        elif blackboard.after_tts == 'move':
            if  blackboard.person_cnt == 2:
                return 'done'
            else: 
                return 'move'
        elif blackboard.after_tts == 'introduce_host':
            set_tts(blackboard, blackboard.active_persons['Host'].desp_gen())
            print("Set target location to door")
            blackboard.active_location = 'door'
            blackboard.after_tts = 'move' # Wait until introduction finished
            return 'tts'
        elif blackboard.after_tts == 'introduce_guest1':
            set_tts(blackboard, blackboard.active_persons['Guest1'].desp_gen())
            blackboard.after_tts = 'introduce_host'
            return 'tts'

########### Register State ##########
class RegisterState(ServiceState):
    def __init__(self) -> None:
        super().__init__(
            FaceRegister,  # srv type
            "/vision/face/register",  # service name
            self.create_request_handler,  # cb to create the request
            ['asr', 'move'],  # outcomes.
            self.response_handler  # cb to process the response
        )

    def create_request_handler(self, blackboard: Blackboard) -> FaceRegister.Request:
        print("Register State")
        req = FaceRegister.Request()
        if blackboard.face_state == 0: # Register
            req.state = 0
            if blackboard.person_cnt > 0: # Guest
                print("Begin register Guest...")   
            else: # Register Host
                print("Begin register Host...")
            
        elif blackboard.face_state == 1: # match
            req.state = 1
            print('Matching...')
        
        return req
    
    def response_handler(
        self,
        blackboard: Blackboard,
        response: FaceRegister.Response
    ) -> str:
        if response.success == False:
            return 'asr'
        else:
            blackboard.active_id = response.id[0]
            blackboard.active_rec_info = response.rec_info     
            if blackboard.person_cnt == 0: # Host, go to door    
                blackboard.persons[blackboard.active_id] = Person("Host Name", "Host Drink", blackboard.active_rec_info)
                blackboard.active_persons['Host'] = blackboard.persons[blackboard.active_id] # Set as Host
                blackboard.active_location = 'door'
                return 'move'
            else:
                return 'asr'
            
########### Move State ##########
class MoveState(ActionState):
    def __init__(self) -> None:
        super().__init__(
            NavigateToPose,
            "/navigate_to_pose",
            self.create_goal_handler,  # cb to create the goal
            ['register', 'detection', 'tts'], 
            self.response_handler  # cb to process the response
        )


    def create_goal_handler(self, blackboard: Blackboard) -> TTSAction.Goal:
        print("Move State")
        if blackboard.active_location == 'door':
            print(f"Person Cnt: {blackboard.person_cnt}")
            blackboard.person_cnt += 1
        goal = NavigateToPose.Goal()
        goal.pose.pose = blackboard.locations[blackboard.active_location]
        goal.pose.header.frame_id = "map"
        return goal
    
    def response_handler(
        self,
        blackboard: Blackboard,
        response: NavigateToPose.Result
    ) -> str:
        
        blackboard.nav_res = response.result
        if blackboard.active_location == 'door':
            print("Going to the door")
            return 'register'
        elif blackboard.active_location == 'first_observation' or blackboard.active_location == 'second_observation':
            print("Going to the Observation")
            return 'detection'
        elif blackboard.active_location == 'host_position':
            print("Going to the Host position")
            set_tts(blackboard, blackboard.persons[blackboard.active_id].desp_gen())
            if blackboard.person_cnt == 2:
                 print("I'm going to introduce guest1")
                 blackboard.after_tts = 'introduce_guest1'
            else:
                print("I'm going to introduce host")
                blackboard.after_tts = 'introduce_host' # Introduce host to guest
            return 'tts'
        
########### ASR State ##########
class ASRState(ActionState):
    def __init__(self) -> None:
        super().__init__(
            Asrcommand,  # action type
            "/asr_command",  # action name
            self.create_goal_handler,  # cb to create the goal
            ['move'],  # outcomes. Includes (SUCCEED, ABORT, CANCEL)
            self.response_handler  # cb to process the response
        )

    def create_goal_handler(self, blackboard: Blackboard) -> Asrcommand.Goal:
        print("ASR State")
        goal = Asrcommand.Goal()
        goal.trigger = True
        return goal
    
    def response_handler(
        self,
        blackboard: Blackboard,
        response: Asrcommand.Result
    ) -> str:
        

        blackboard.persons[blackboard.active_id] = Person(response.name, response.drink, blackboard.active_rec_info)
        if blackboard.person_cnt == 1:
            blackboard.active_persons['Guest1'] = blackboard.persons[blackboard.active_id]
        elif blackboard.person_cnt == 2:
            blackboard.active_persons['Guest2'] = blackboard.persons[blackboard.active_id]
        print(blackboard.persons[blackboard.active_id].ensure_gen())
        # Move to the first observation pose
        blackboard.active_location = 'first_observation'
        return 'move'
    
########### Detection State ##########
class DetectionState(ServiceState):
    def __init__(self) -> None:
        super().__init__(
            ObjectDetection,  # srv type
            "/object_detection",  # service name
            self.create_request_handler,  # cb to create the request
            ['move'],  # outcomes.
            self.response_handler  # cb to process the response
        )

    def create_request_handler(self, blackboard: Blackboard) -> ObjectDetection.Request:
        print("Detection State")
        req = ObjectDetection.Request()
        req.mode = ''
        # req.target_frame = 'map'    # obj.centroid frame
        return req
    
    def response_handler(
        self,
        blackboard: Blackboard,
        response: ObjectDetection.Response
    ) -> str:
        if response.status > 0: # Nothing found, go to the next observation place
            blackboard.active_location = 'second_observation'
            return 'move'
        else:
            def distance(p1, p2):
                x1, y1, z1 = p1
                x2, y2, z2 = p2
                return np.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2 + (z1 - z2) ** 2)
            
            for obj in response.objects:
                if obj.cls == 'person':
                    # coordinate in /map
                    x, y, z = obj.centroid.x, obj.centroid.y, obj.centroid.z
                    min_dis, min_idx = 1e6, -1
                    for i, loc in enumerate(blackboard.seats_locations):
                        dis = distance((x, y, z), loc)
                        if min_dis > dis:
                            min_idx, min_dis = i, dis
                    if min_dis < 0.3:
                        blackboard.seats_occupied[min_idx] = 0  # can be set to person ID later
            blackboard.active_location = 'host_position'
            return 'move'    

# main
def main():

    print("Receptionist")

    # init ROS 2
    rclpy.init()

    # create a FSM
    sm = StateMachine(outcomes=["done"])

    # add states
    sm.add_state(
        "init_task",
        InitState(),
        transitions={
            "tts": "TTS"
        }
    )

    sm.add_state(
        "TTS",
        TTSState(),
        transitions={
            'register': 'Register',
            'move': 'Move',
            'tts': 'TTS'
        }
    )

    sm.add_state(
        "ASR",
        ASRState(),
        transitions={
            'move': 'Move'
        }
    )

    sm.add_state(
        "Register",
        RegisterState(),
        transitions={
            'asr': 'ASR',
            'move': 'Move'
        }
    )

    sm.add_state(
        "Move",
        MoveState(),
        transitions={
            'tts': 'TTS',
            'register': "Register",
            'detection': "Detection"
        }
    )

    sm.add_state(
        "Detection",
        DetectionState(),
        transitions={
            'move': "Move"
        }
    )
    

    # pub FSM info
    YasminViewerPub("YASMIN_ACTION_CLIENT_DEMO", sm)

    # execute FSM
    outcome = sm()
    print(outcome)

    # shutdown ROS 2
    rclpy.shutdown()


if __name__ == "__main__":
    main()