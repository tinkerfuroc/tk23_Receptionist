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


from typing import List, Callable, Type, Any
from threading import RLock, Event

from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from rclpy.callback_groups import ReentrantCallbackGroup
from action_msgs.msg import GoalStatus

from yasmin import State
from yasmin import Blackboard
from yasmin_ros.yasmin_node import YasminNode
from yasmin_ros.basic_outcomes import SUCCEED, ABORT, CANCEL


class ActionState(State):

    _node: Node

    _action_client: ActionClient
    _action_done_event: Event = Event()

    _action_result: Any
    _action_status: GoalStatus

    _goal_handle: ClientGoalHandle
    _goal_handle_lock: RLock = RLock()

    _create_goal_handler: Callable
    _result_handler: Callable

    def __init__(
        self,
        action_type: Type,
        action_name: str,
        create_goal_handler: Callable,
        outcomes: List[str] = None,
        result_handler: Callable = None,
        node: Node = None
    ) -> None:

        _outcomes = [SUCCEED, ABORT, CANCEL]

        if outcomes:
            _outcomes = _outcomes + outcomes

        if node is None:
            self._node = YasminNode.get_instance()
        else:
            self._node = node

        self._action_client = ActionClient(
            self._node,
            action_type,
            action_name,
            callback_group=ReentrantCallbackGroup()
        )

        self._create_goal_handler = create_goal_handler
        self._result_handler = result_handler

        if not self._create_goal_handler:
            raise Exception("create_goal_handler is needed")

        super().__init__(_outcomes)

    def cancel_state(self) -> None:

        with self._goal_handle_lock:
            if self._goal_handle is not None:
                self._goal_handle.cancel_goal()

        super().cancel_state()

    def execute(self, blackboard: Blackboard) -> str:

        goal = self._create_goal_handler(blackboard)
        self._action_client.wait_for_server()

        self._action_done_event.clear()
        send_goal_future = self._action_client.send_goal_async(goal)
        send_goal_future.add_done_callback(self._goal_response_callback)

        # Wait for action to be done
        self._action_done_event.wait()

        if self._action_status == GoalStatus.STATUS_CANCELED:
            return CANCEL

        elif self._action_status == GoalStatus.STATUS_ABORTED:
            return ABORT

        elif self._action_status == GoalStatus.STATUS_SUCCEEDED:

            if self._result_handler is not None:
                return self._result_handler(blackboard, self._action_result)

            return SUCCEED

        return ABORT

    def _goal_response_callback(self, future) -> None:
        with self._goal_handle_lock:
            self._goal_handle = future.result()
            get_result_future = self._goal_handle.get_result_async()
            get_result_future.add_done_callback(self._get_result_callback)

    def _get_result_callback(self, future) -> None:
        self._action_result = future.result().result
        self._action_status = future.result().status
        self._action_done_event.set()
