#!/usr/bin/env python3

# Copyright (C) 2023  Miguel Ángel González Santamarta

# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.


import rclpy
from tts_interfaces.action import TTSAction
from asr_communication_interface.action import Asrcommand
from tinker_vision_msgs.srv import FaceRegister, ObjectDetection
from yasmin import CbState
from yasmin import State
from yasmin import Blackboard
from yasmin import StateMachine
from yasmin_ros import ActionState,ServiceState
from yasmin_ros.basic_outcomes import SUCCEED, ABORT, CANCEL
from yasmin_viewer import YasminViewerPub
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
    return SUCCEED


def print_result(blackboard: Blackboard) -> str:
    print(f"Result: {blackboard.tts_res}")
    return SUCCEED



# TODO: implement Register
class RegisterState(ServiceState):
    def __init__(self) -> None:
        super().__init__(
            FaceRegister,  # srv type
            "/vision/face/register",  # service name
            self.create_request_handler,  # cb to create the request
            ['asr', 'confirm'],  # outcomes.
            self.response_handler  # cb to process the response
        )

    def create_request_handler(self, blackboard: Blackboard) -> FaceRegister.Request:
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
            blackboard.active_id = response.id
            blackboard.active_rec_info = response.rec_info     
            if blackboard.person_cnt == 0: # Host, no asr    
                blackboard.persons[blackboard.active_id] = Person("Host Name", "Host Drink", blackboard.active_rec_info)
                blackboard.active_persons['Host'] = blackboard.persons[blackboard.active_id] # Set as Host
                set_tts(blackboard. blackboard.persons[blackboard.active_id].ensure_gen())
                blackboard.after_tts = 'move'
                blackboard.active_location = 'door'
                return 'confirm'
            else:
                return 'asr'
            

class DetectionState(ServiceState):
    def __init__(self) -> None:
        super().__init__(
            ObjectDetection,  # srv type
            "/object_detection_service",  # service name
            self.create_request_handler,  # cb to create the request
            ['asr', 'done'],  # outcomes.
            self.response_handler  # cb to process the response
        )

    def create_request_handler(self, blackboard: Blackboard) -> ObjectDetection.Request:
        req = ObjectDetection.Request()
        req.flags = ''
        req.target_frame = 'map'    # obj.centroid frame
        return req
    
    def response_handler(
        self,
        blackboard: Blackboard,
        response: ObjectDetection.Response
    ) -> str:
        if response.status > 0:
            return 'asr'
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
            
            return 'done'


class ASRState(ActionState):
    def __init__(self) -> None:
        super().__init__(
            Asrcommand,  # action type
            "/asr_command",  # action name
            self.create_goal_handler,  # cb to create the goal
            ['confirm'],  # outcomes. Includes (SUCCEED, ABORT, CANCEL)
            self.response_handler  # cb to process the response
        )

    def create_goal_handler(self, blackboard: Blackboard) -> Asrcommand.Goal:
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
        set_tts(blackboard, blackboard.persons[blackboard.active_id].ensure_gen())
        # Confirm, then go to target position
        blackboard.after_tts = 'move'
        blackboard.active_location = 'sofa'
        return 'confirm'
    

class MoveState(ActionState):
    def __init__(self) -> None:
        super().__init__(
            TTSAction,  # action type
            "/tts_command",  # action name
            self.create_goal_handler,  # cb to create the goal
            ['tts'],  # outcomes. Includes (SUCCEED, ABORT, CANCEL)
            self.response_handler  # cb to process the response
        )

    def create_goal_handler(self, blackboard: Blackboard) -> TTSAction.Goal:
        if blackboard.active_location == 'door':
            set_tts(blackboard, 'Going to the door')
            blackboard.person_cnt += 1
            blackboard.after_tts = 'register'
        else:
            set_tts(blackboard, "Going to sofa")
            blackboard.after_tts = 'introduce_guest'

        # TODO: Change to navigation goal
        goal = TTSAction.Goal()
        goal.target_string = blackboard.tts_target
        return goal
    
    def response_handler(
        self,
        blackboard: Blackboard,
        response: TTSAction.Result
    ) -> str:
        
        blackboard.tts_res = response.finished
        return 'tts'
    

class TTSState(ActionState):
    def __init__(self) -> None:
        super().__init__(
            TTSAction,  # action type
            "/tts_command",  # action name
            self.create_goal_handler,  # cb to create the goal
            ['register', 'move', 'introduce_host'],  # outcomes. 
            self.response_handler  # cb to process the response
        )

    def create_goal_handler(self, blackboard: Blackboard) -> TTSAction.Goal:
        if blackboard.next_tts_state == 'introduce_guest':
            if blackboard.person_cnt == 1:
                set_tts(blackboard, blackboard.active_persons['Guest1'].desp_gen())
        elif blackboard.next_tts_state == 'introduce_host':
            set_tts(blackboard, blackboard.active_persons['Host'].desp_gen())
            blackboard.after_tts = 'move'
            blackboard.active_location = 'door'
        goal = TTSAction.Goal()
        goal.target_string = blackboard.tts_target
        return goal

    def response_handler(
        self,
        blackboard: Blackboard,
        response: TTSAction.Result
    ) -> str:

        blackboard.tts_res = response.finished
        if blackboard.after_tts == 'register':
            return 'register'
        elif blackboard.after_tts == 'move':
            return 'move'
        elif blackboard.after_tts == 'introduce_guest':
            blackboard.after_tts = None # Wait until introduction finished
            blackboard.next_tts_state == 'introduce_guest'
            return 'introduce_host'
    
# define state Foo
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
        # locations: dictionary with coordinates
        blackboard.locations = {"door": (), "sofa": ()}
        blackboard.person_cnt = 0
        blackboard.active_location = blackboard.locations["door"]

        blackboard.after_tts = 'register' # Register Host first

        blackboard.seats_location = [
            (0, 0, 0),
            (0, 0, 0),
            (0, 0, 0),
        ]
        blackboard.seats_occupied = [-1, -1, -1]
        blackboard.seats_observation_poses = [
            (0, 0, 0, 0),
            (0, 0, 0, 0),
        ]

        set_tts(blackboard, "Hello! My name is Tinker. I'm going to welcome the guests!")
        return "tts"
        
# main
def main():

    print("yasmin_action_client_demo")

    # init ROS 2
    rclpy.init()

    # create a FSM
    sm = StateMachine(outcomes=["outcome4"])

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
            'introduce_host': 'TTS'
        }
    )

    sm.add_state(
        "ASR",
        ASRState(),
        transitions={
            'confirm': 'TTS'
        }
    )

    sm.add_state(
        "Register",
        RegisterState(),
        transitions={
            'asr': 'ASR',
            'confirm': 'TTS'
        }
    )

    sm.add_state(
        "Move",
        MoveState(),
        transitions={
            'tts': 'TTS'
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
