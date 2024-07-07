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
from tinker_vision_msgs.srv import FaceRegister
from yasmin import CbState
from yasmin import State
from yasmin import Blackboard
from yasmin import StateMachine
from yasmin_ros import ActionState,ServiceState
from yasmin_ros.basic_outcomes import SUCCEED, ABORT, CANCEL
from yasmin_viewer import YasminViewerPub

class Person:
    def __init__(self, id=0, name='Foo', drink='Foo'):
        self.id = id
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

# # TODO: implement Register
# class RegisterState(ActionState):
#     def __init__(self) -> None:
#         super().__init__(
#             TTSAction,  # action type
#             "/tts_command",  # action name
#             self.create_goal_handler,  # cb to create the goal
#             ['start_move'],  # outcomes. Includes (SUCCEED, ABORT, CANCEL)
#             self.response_handler  # cb to process the response
#         )

#     def create_goal_handler(self, blackboard: Blackboard) -> TTSAction.Goal:
#         if blackboard.person_cnt > 0: # Guest
#             print("Begin register Guest...")
#             # TODO: CHANGE NAME
#             set_tts(blackboard, blackboard.active_person.ensure_gen())
#             blackboard.location_lable = 1
#         else: # Host
#             set_tts(blackboard, "Begin register Host...")
#             blackboard.location_lable = 0

#         blackboard.active_location = blackboard.locations[blackboard.location_lable]

#         goal = TTSAction.Goal()
#         goal.target_string = blackboard.tts_target
#         print('Registering...')
#         return goal
    
#     def response_handler(
#         self,
#         blackboard: Blackboard,
#         response: TTSAction.Result
#     ) -> str:
        
#         blackboard.tts_res = response.finished
#         return 'start_move'


# TODO: implement Register
class RegisterState(ServiceState):
    def __init__(self) -> None:
        super().__init__(
            FaceRegister,  # srv type
            "/vision/face/register",  # service name
            self.create_request_handler,  # cb to create the request
            ['start_move'],  # outcomes. Includes (SUCCEED, ABORT, CANCEL)
            self.response_handler  # cb to process the response
        )

    def create_request_handler(self, blackboard: Blackboard) -> FaceRegister.Request:


        # TODO: CHANGE STATE for register/match
        if blackboard.face_state == 0: # register
            if blackboard.person_cnt > 0: # Guest
                print("Begin register Guest...")
                # TODO: CHANGE NAME
                set_tts(blackboard, blackboard.active_person.ensure_gen())
                blackboard.location_lable = 1
            else: # Host
                set_tts(blackboard, "Begin register Host...")
                blackboard.location_lable = 0
            blackboard.active_location = blackboard.locations[blackboard.location_lable]

            req = FaceRegister.Request()
            req.state = 0

        elif blackboard.face_state == 1: # match
            req = FaceRegister.Request()
            req.state = 1
            print('Matching...')
        
        return req
    
    def response_handler(
        self,
        blackboard: Blackboard,
        response: FaceRegister.Response
    ) -> str:
        
        blackboard.face_suceess = response.success
        blackboard.id = response.id
        blackboard.age = response.rec_info[0]
        blackboard.gender = response.rec_info[1]
        return 'start_move'

# TODO: implement Move
class MoveState(ActionState):
    def __init__(self) -> None:
        super().__init__(
            TTSAction,  # action type
            "/tts_command",  # action name
            self.create_goal_handler,  # cb to create the goal
            ['register', 'introduce_guest'],  # outcomes. Includes (SUCCEED, ABORT, CANCEL)
            self.response_handler  # cb to process the response
        )

    def create_goal_handler(self, blackboard: Blackboard) -> TTSAction.Goal:
        if blackboard.active_location == 'door':
            set_tts(blackboard, "Going to the door")
            blackboard.person_cnt += 1
            blackboard.active_person = blackboard.persons[blackboard.person_cnt]
            blackboard.next_state = 'register'
        else:
            set_tts(blackboard, "Going to sofa")
            blackboard.next_state = 'introduce_guest'

        goal = TTSAction.Goal()
        goal.target_string = blackboard.tts_target
        print('Registering...')
        return goal
    
    def response_handler(
        self,
        blackboard: Blackboard,
        response: TTSAction.Result
    ) -> str:
        
        blackboard.tts_res = response.finished
        return blackboard.next_state
    

class TTSState(ActionState):
    def __init__(self) -> None:
        super().__init__(
            TTSAction,  # action type
            "/tts_command",  # action name
            self.create_goal_handler,  # cb to create the goal
            ['register', 'move', 'introduce_host'],  # outcomes. Includes (SUCCEED, ABORT, CANCEL)
            self.response_handler  # cb to process the response
        )

    def create_goal_handler(self, blackboard: Blackboard) -> TTSAction.Goal:
        if blackboard.next_state == 'introduce_guest':
            set_tts(blackboard, blackboard.active_person.desp_gen())
        elif blackboard.next_state == 'introduce_host':
            set_tts(blackboard, blackboard.persons[0].desp_gen())
        goal = TTSAction.Goal()
        goal.target_string = blackboard.tts_target
        return goal

    def response_handler(
        self,
        blackboard: Blackboard,
        response: TTSAction.Result
    ) -> str:

        blackboard.tts_res = response.finished
        if blackboard.next_state == 'register':
            return 'register'
        elif blackboard.next_state == 'move':
            return 'move'
        elif blackboard.next_state == 'introduce_guest':
            blackboard.next_state = 'introduce_host'
            print(f'Active person: {blackboard.active_person}')
            return 'introduce_host'
        elif blackboard.next_state == 'introduce_host':
            blackboard.next_state = 'move'
            blackboard.location_lable = 0
            blackboard.active_location = blackboard.locations[blackboard.location_lable]
            print(f'Active location: {blackboard.active_location}')
            return 'move'
    
# define state Foo
class InitState(State):
    def __init__(self) -> None:
        super().__init__(["outcome1"])
        self.counter = 0

    def execute(self, blackboard: Blackboard) -> str:
        set_tts(blackboard, "Hello! My name is Tinker. I'm going to welcome the guests!")
        print(f'Init Task')
        blackboard.persons = [Person(1, 'Lungdge', 'Coffee'), Person(2, "Cabbage Dog", 'Kai shui'), Person(3, 'Queen xin yau', 'Milk')]
        blackboard.locations = ['door', 'sofa']
        blackboard.person_cnt = 0
        blackboard.location_lable = 0
        blackboard.active_person = blackboard.persons[blackboard.person_cnt]
        blackboard.active_location = blackboard.locations[blackboard.location_lable]

        blackboard.next_state = 'register'
        return "outcome1"
        
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
            "outcome1": "TTS"
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
        "Register",
        RegisterState(),
        transitions={
            'start_move': 'Move'
        }
    )

    sm.add_state(
        "Move",
        MoveState(),
        transitions={
            'register': 'Register',
            'introduce_guest': 'TTS'
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
