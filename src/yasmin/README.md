# YASMIN (Yet Another State MachINe)

<p align="center">
  <img src="./docs/logo.png" width="50%" />
</p>

YASMIN is a project focused on implementing robot behaviors using Finite State Machines (FSM). It is available for ROS 2, Python and C++.

## Table of Contents

1. [Features](#features)
2. [Installation](#installation)
3. [Demos](#demos)
   - [Python](#python)
   - [Cpp](#cpp)
4. [YASMIN Viewer](#yasmin-viewer)
5. [Citations](#citations)

## Features

- Fully integrated into ROS 2.
- Available for Python and C++.
- Fast prototyping.
- Default states for ROS 2 action and service clients.
- Blackboards are used to share data between states and state machines.
- State machines can be canceled and stopped, which means stopping the current executing state.
- A web viewer is included, which allows monitoring of the execution of the state machines.

## Installation

```shell
# clone
$ cd ~/ros2_ws/src
$ git clone https://github.com/uleroboticsgroup/yasmin.git

# dependencies
$ cd yasmin
$ pip3 install -r requirements.txt

# colcon
$ cd ~/ros2_ws
$ colcon build
```

## Demos

There are some examples, for both Python and C++, that can be found in [yasmin_demo](./yasmin_demo/).

### Python

#### Vanilla Demo (FSM)

```shell
$ ros2 run yasmin_demo yasmin_demo.py
```

<p align="center">
  <img src="./docs/demo.gif" width="65%" />
</p>

<details>
<summary>Click to expand</summary>

```python
#!/usr/bin/env python3

import time
import rclpy
from yasmin import State
from yasmin import Blackboard
from yasmin import StateMachine
from yasmin_viewer import YasminViewerPub


# define state Foo
class FooState(State):
    def __init__(self) -> None:
        super().__init__(["outcome1", "outcome2"])
        self.counter = 0

    def execute(self, blackboard: Blackboard) -> str:
        print("Executing state FOO")
        time.sleep(3)

        if self.counter < 3:
            self.counter += 1
            blackboard.foo_str = f"Counter: {self.counter}"
            return "outcome1"
        else:
            return "outcome2"


# define state Bar
class BarState(State):
    def __init__(self) -> None:
        super().__init__(outcomes=["outcome3"])

    def execute(self, blackboard: Blackboard) -> str:
        print("Executing state BAR")
        time.sleep(3)

        print(blackboard.foo_str)
        return "outcome3"


# main
def main():

    print("yasmin_demo")

    # init ROS 2
    rclpy.init()

    # create a FSM
    sm = StateMachine(outcomes=["outcome4"])

    # add states
    sm.add_state(
        "FOO",
        FooState(),
        transitions={
            "outcome1": "BAR",
            "outcome2": "outcome4"
        }
    )
    sm.add_state(
        "BAR",
        BarState(),
        transitions={
            "outcome3": "FOO"
        }
    )

    # pub FSM info
    YasminViewerPub("YASMIN_DEMO", sm)

    # execute FSM
    outcome = sm()
    print(outcome)

    # shutdown ROS 2
    rclpy.shutdown()


if __name__ == "__main__":
    main()
```

</details>

#### Service Demo (FSM + ROS 2 Service Client)

```shell
$ ros2 run demo_nodes_py add_two_ints_server
```

```shell
$ ros2 run yasmin_demo service_client_demo.py
```

<details>
<summary>Click to expand</summary>

```python
import rclpy
from example_interfaces.srv import AddTwoInts
from yasmin import CbState
from yasmin import Blackboard
from yasmin import StateMachine
from yasmin_ros import ServiceState
from yasmin_ros.basic_outcomes import SUCCEED, ABORT
from yasmin_viewer import YasminViewerPub


class AddTwoIntsState(ServiceState):
    def __init__(self) -> None:
        super().__init__(
            AddTwoInts,  # srv type
            "/add_two_ints",  # service name
            self.create_request_handler,  # cb to create the request
            ["outcome1"],  # outcomes. Includes (SUCCEED, ABORT)
            self.response_handler  # cb to process the response
        )

    def create_request_handler(self, blackboard: Blackboard) -> AddTwoInts.Request:

        req = AddTwoInts.Request()
        req.a = blackboard.a
        req.b = blackboard.b
        return req

    def response_handler(
        self,
        blackboard: Blackboard,
        response: AddTwoInts.Response
    ) -> str:

        blackboard.sum = response.sum
        return "outcome1"


def set_ints(blackboard: Blackboard) -> str:
    blackboard.a = 10
    blackboard.b = 5
    return SUCCEED


def print_sum(blackboard: Blackboard) -> str:
    print(f"Sum: {blackboard.sum}")
    return SUCCEED


# main
def main():

    print("yasmin_service_client_demo")

    # init ROS 2
    rclpy.init()

    # create a FSM
    sm = StateMachine(outcomes=["outcome4"])

    # add states
    sm.add_state(
        "SETTING_INTS",
        CbState([SUCCEED], set_ints),
        transitions={
            SUCCEED: "ADD_TWO_INTS"
        }
    )
    sm.add_state(
        "ADD_TWO_INTS",
        AddTwoIntsState(),
        transitions={
            "outcome1": "PRINTING_SUM",
            SUCCEED: "outcome4",
            ABORT: "outcome4"
        }
    )
    sm.add_state(
        "PRINTING_SUM",
        CbState([SUCCEED], print_sum),
        transitions={
            SUCCEED: "outcome4"
        }
    )

    # pub FSM info
    YasminViewerPub("YASMIN_SERVICE_CLIENT_DEMO", sm)

    # execute FSM
    outcome = sm()
    print(outcome)

    # shutdown ROS 2
    rclpy.shutdown()


if __name__ == "__main__":
    main()
```

</details>

#### Action Demo (FSM + ROS 2 Action)

```shell
$ ros2 run action_tutorials_cpp fibonacci_action_server
```

```shell
$ ros2 run yasmin_demo action_client_demo.py
```

<details>
<summary>Click to expand</summary>

```python
import rclpy
from action_tutorials_interfaces.action import Fibonacci
from yasmin import CbState
from yasmin import Blackboard
from yasmin import StateMachine
from yasmin_ros import ActionState
from yasmin_ros.basic_outcomes import SUCCEED, ABORT, CANCEL
from yasmin_viewer import YasminViewerPub


class FibonacciState(ActionState):
    def __init__(self) -> None:
        super().__init__(
            Fibonacci,  # action type
            "/fibonacci",  # action name
            self.create_goal_handler,  # cb to create the goal
            None,  # outcomes. Includes (SUCCEED, ABORT, CANCEL)
            self.response_handler  # cb to process the response
        )

    def create_goal_handler(self, blackboard: Blackboard) -> Fibonacci.Goal:

        goal = Fibonacci.Goal()
        goal.order = blackboard.n
        return goal

    def response_handler(
        self,
        blackboard: Blackboard,
        response: Fibonacci.Result
    ) -> str:

        blackboard.fibo_res = response.sequence
        return SUCCEED


def set_int(blackboard: Blackboard) -> str:
    blackboard.n = 3
    return SUCCEED


def print_result(blackboard: Blackboard) -> str:
    print(f"Result: {blackboard.fibo_res}")
    return SUCCEED


# main
def main():

    print("yasmin_action_client_demo")

    # init ROS 2
    rclpy.init()

    # create a FSM
    sm = StateMachine(outcomes=["outcome4"])

    # add states
    sm.add_state(
        "SETTING_INT",
        CbState([SUCCEED], set_int),
        transitions={
            SUCCEED: "CALLING_FIBONACCI"
        }
    )
    sm.add_state(
        "CALLING_FIBONACCI",
        FibonacciState(),
        transitions={
            SUCCEED: "PRINTING_RESULT",
            CANCEL: "outcome4",
            ABORT: "outcome4"
        }
    )
    sm.add_state(
        "PRINTING_RESULT",
        CbState([SUCCEED], print_result),
        transitions={
            SUCCEED: "outcome4"
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
```

</details>

#### Monitor Demo (FSM + ROS 2 Subscriber)

```shell
$ ros2 run yasmin_demo monitor_demo.py
```

<details>
<summary>Click to expand</summary>

```python
import rclpy
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry

from yasmin import Blackboard
from yasmin import StateMachine
from yasmin_ros import MonitorState
from yasmin_ros.basic_outcomes import CANCEL
from yasmin_viewer import YasminViewerPub


class PrintOdometryState(MonitorState):
    def __init__(self, times: int) -> None:
        super().__init__(
            Odometry,  # msg type
            "odom",  # topic name
            ["outcome1", "outcome2"],  # outcomes
            self.monitor_handler,  # monitor handler callback
            qos=qos_profile_sensor_data,  # qos for the topic sbscription
            msg_queue=10,  # queue of the monitor handler callback
            timeout=10  # timeout to wait for msgs in seconds
            # if not None, CANCEL outcome is added
        )
        self.times = times

    def monitor_handler(self, blackboard: Blackboard, msg: Odometry) -> str:
        print(msg)

        self.times -= 1

        if self.times <= 0:
            return "outcome2"

        return "outcome1"


# main
def main():

    print("yasmin_monitor_demo")

    # init ROS 2
    rclpy.init()

    # create a FSM
    sm = StateMachine(outcomes=["outcome4"])

    # add states
    sm.add_state(
        "PRINTING_ODOM",
        PrintOdometryState(5),
        transitions={
            "outcome1": "PRINTING_ODOM",
            "outcome2": "outcome4",
            CANCEL: "outcome4"
        }
    )

    # pub FSM info
    YasminViewerPub("YASMIN_MONITOR_DEMO", sm)

    # execute FSM
    outcome = sm()
    print(outcome)

    # shutdown ROS 2
    rclpy.shutdown()


if __name__ == "__main__":
    main()
```

</details>

#### Nav2 Demo (Hierarchical FSM + ROS 2 Action)

```shell
$ ros2 run yasmin_demo nav_demo.py
```

<details>
<summary>Click to expand</summary>

```python
import random

import rclpy
from geometry_msgs.msg import Pose
from nav2_msgs.action import NavigateToPose

from yasmin import CbState
from yasmin import Blackboard
from yasmin import StateMachine
from yasmin_ros import ActionState
from yasmin_ros.basic_outcomes import SUCCEED, ABORT, CANCEL
from yasmin_viewer import YasminViewerPub

HAS_NEXT = "has_next"
END = "end"


class Nav2State(ActionState):
    def __init__(self) -> None:
        super().__init__(
            NavigateToPose,  # action type
            "/navigate_to_pose",  # action name
            self.create_goal_handler,  # cb to create the goal
            None,  # outcomes. Includes (SUCCEED, ABORT, CANCEL)
            None  # cb to process the response
        )

    def create_goal_handler(self, blackboard: Blackboard) -> NavigateToPose.Goal:

        goal = NavigateToPose.Goal()
        goal.pose.pose = blackboard.pose
        goal.pose.header.frame_id = "map"
        return goal


def create_waypoints(blackboard: Blackboard) -> str:
    blackboard.waypoints = {
        "entrance": [1.25, 6.30, -0.78, 0.67],
        "bathroom": [4.89, 1.64, 0.0, 1.0],
        "livingroom": [1.55, 4.03, -0.69, 0.72],
        "kitchen": [3.79, 6.77, 0.99, 0.12],
        "bedroom": [7.50, 4.89, 0.76, 0.65],
    }
    return SUCCEED


def take_random_waypoint(blackboard) -> str:
    blackboard.random_waypoints = random.sample(
        list(blackboard.waypoints.keys()),
        blackboard.waypoints_num)
    return SUCCEED


def get_next_waypoint(blackboard: Blackboard) -> str:

    if not blackboard.random_waypoints:
        return END

    wp_name = blackboard.random_waypoints.pop(0)
    wp = blackboard.waypoints[wp_name]

    pose = Pose()
    pose.position.x = wp[0]
    pose.position.y = wp[1]

    pose.orientation.z = wp[2]
    pose.orientation.w = wp[3]

    blackboard.pose = pose
    blackboard.text = f"I have reached waypoint {wp_name}"

    return HAS_NEXT


# main
def main():

    print("yasmin_nav2_demo")

    # init ROS 2
    rclpy.init()

    # create state machines
    sm = StateMachine(outcomes=[SUCCEED, ABORT, CANCEL])
    nav_sm = StateMachine(outcomes=[SUCCEED, ABORT, CANCEL])

    # add states
    sm.add_state(
        "CREATING_WAYPOINTS",
        CbState([SUCCEED], create_waypoints),
        transitions={
            SUCCEED: "TAKING_RANDOM_WAYPOINTS"
        }
    )
    sm.add_state(
        "TAKING_RANDOM_WAYPOINTS",
        CbState([SUCCEED], take_random_waypoint),
        transitions={
            SUCCEED: "NAVIGATING"
        }
    )

    nav_sm.add_state(
        "GETTING_NEXT_WAYPOINT",
        CbState([END, HAS_NEXT], get_next_waypoint),
        transitions={
            END: SUCCEED,
            HAS_NEXT: "NAVIGATING"
        }
    )
    nav_sm.add_state(
        "NAVIGATING",
        Nav2State(),
        transitions={
            SUCCEED: "GETTING_NEXT_WAYPOINT",
            CANCEL: CANCEL,
            ABORT: ABORT
        }
    )

    sm.add_state(
        "NAVIGATING",
        nav_sm,
        transitions={
            SUCCEED: SUCCEED,
            CANCEL: CANCEL,
            ABORT: ABORT
        }
    )

    # pub FSM info
    YasminViewerPub("YASMIN_NAV_DEMO", sm)

    # execute FSM
    blackboard = Blackboard()
    blackboard.waypoints_num = 2
    outcome = sm(blackboard)
    print(outcome)

    # shutdown ROS 2
    rclpy.shutdown()


if __name__ == "__main__":
    main()
```

</details>

### Cpp

#### Vanilla Demo

```shell
$ ros2 run yasmin_demo yasmin_demo
```

<details>
<summary>Click to expand</summary>

```cpp
#include <chrono>
#include <iostream>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "yasmin/state.hpp"
#include "yasmin/state_machine.hpp"
#include "yasmin_viewer/yasmin_viewer_pub.hpp"

// define state Foo
class FooState : public yasmin::State {
public:
  int counter;

  FooState() : yasmin::State({"outcome1", "outcome2"}) { this->counter = 0; };

  std::string
  execute(std::shared_ptr<yasmin::blackboard::Blackboard> blackboard) {
    std::cout << "Executing state FOO\n";
    std::this_thread::sleep_for(std::chrono::seconds(3));

    if (this->counter < 3) {
      this->counter += 1;
      blackboard->set<std::string>("foo_str",
                                   "Counter: " + std::to_string(this->counter));
      return "outcome1";

    } else {
      return "outcome2";
    }
  }

  std::string to_string() { return "FooState"; }
};

// define state Bar
class BarState : public yasmin::State {
public:
  BarState() : yasmin::State({"outcome3"}){};

  std::string
  execute(std::shared_ptr<yasmin::blackboard::Blackboard> blackboard) {
    std::cout << "Executing state BAR\n";
    std::this_thread::sleep_for(std::chrono::seconds(3));

    std::cout << blackboard->get<std::string>("foo_str") << "\n";

    return "outcome3";
  }

  std::string to_string() { return "BarState"; }
};

int main(int argc, char *argv[]) {

  std::cout << "yasmin_demo\n";
  rclcpp::init(argc, argv);

  // create a state machine
  auto sm = std::make_shared<yasmin::StateMachine>(
      yasmin::StateMachine({"outcome4"}));

  // add states
  sm->add_state("FOO", std::make_shared<FooState>(),
                {{"outcome1", "BAR"}, {"outcome2", "outcome4"}});
  sm->add_state("BAR", std::make_shared<BarState>(), {{"outcome3", "FOO"}});

  // pub
  yasmin_viewer::YasminViewerPub yasmin_pub("YASMIN_ACTION_CLIENT_DEMO", sm);

  // execute
  std::string outcome = (*sm.get())();
  std::cout << outcome << "\n";

  rclcpp::shutdown();

  return 0;
}
```

</details>

#### Service Demo (FSM + ROS 2 Service Client)

```shell
$ ros2 run demo_nodes_py add_two_ints_server
```

```shell
$ ros2 run yasmin_demo service_client_demo
```

<details>
<summary>Click to expand</summary>

```cpp
#include <iostream>
#include <memory>
#include <string>

#include "example_interfaces/srv/add_two_ints.hpp"
#include "rclcpp/rclcpp.hpp"

#include "yasmin/cb_state.hpp"
#include "yasmin/state_machine.hpp"
#include "yasmin_ros/basic_outcomes.hpp"
#include "yasmin_ros/service_state.hpp"
#include "yasmin_viewer/yasmin_viewer_pub.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

std::string
set_ints(std::shared_ptr<yasmin::blackboard::Blackboard> blackboard) {
  blackboard->set<int>("a", 10);
  blackboard->set<int>("b", 5);
  return yasmin_ros::basic_outcomes::SUCCEED;
}

std::string
print_sum(std::shared_ptr<yasmin::blackboard::Blackboard> blackboard) {
  fprintf(stderr, "Sum: %d\n", blackboard->get<int>("sum"));
  return yasmin_ros::basic_outcomes::SUCCEED;
}

class AddTwoIntsState
    : public yasmin_ros::ServiceState<example_interfaces::srv::AddTwoInts> {

public:
  AddTwoIntsState()
      : yasmin_ros::ServiceState<example_interfaces::srv::AddTwoInts> // msg
        (                                                             // node
            "/add_two_ints", // srv name
            std::bind(&AddTwoIntsState::create_request_handler, this, _1),
            {"outcome1"},
            std::bind(&AddTwoIntsState::response_handler, this, _1, _2)){};

  example_interfaces::srv::AddTwoInts::Request::SharedPtr
  create_request_handler(
      std::shared_ptr<yasmin::blackboard::Blackboard> blackboard) {

    auto request =
        std::make_shared<example_interfaces::srv::AddTwoInts::Request>();

    request->a = blackboard->get<int>("a");
    request->b = blackboard->get<int>("b");

    return request;
  }

  std::string response_handler(
      std::shared_ptr<yasmin::blackboard::Blackboard> blackboard,
      example_interfaces::srv::AddTwoInts::Response::SharedPtr response) {

    blackboard->set<int>("sum", response->sum);

    return "outcome1";
  }

  std::string to_string() { return "AddTwoIntsState"; }
};

int main(int argc, char *argv[]) {

  std::cout << "yasmin_service_client_demo\n";
  rclcpp::init(argc, argv);

  // create a state machine
  auto sm = std::make_shared<yasmin::StateMachine>(
      yasmin::StateMachine({"outcome4"}));

  // add states
  sm->add_state("SETTING_INTS",
                std::make_shared<yasmin::CbState>(yasmin::CbState(
                    {yasmin_ros::basic_outcomes::SUCCEED}, set_ints)),
                {{yasmin_ros::basic_outcomes::SUCCEED, "ADD_TWO_INTS"}});
  sm->add_state("ADD_TWO_INTS", std::make_shared<AddTwoIntsState>(),
                {{"outcome1", "PRINTING_SUM"},
                 {yasmin_ros::basic_outcomes::SUCCEED, "outcome4"},
                 {yasmin_ros::basic_outcomes::ABORT, "outcome4"}});
  sm->add_state("PRINTING_SUM",
                std::make_shared<yasmin::CbState>(yasmin::CbState(
                    {yasmin_ros::basic_outcomes::SUCCEED}, print_sum)),
                {{yasmin_ros::basic_outcomes::SUCCEED, "outcome4"}});

  // pub
  yasmin_viewer::YasminViewerPub yasmin_pub("YASMIN_ACTION_CLIENT_DEMO", sm);

  // execute
  std::string outcome = (*sm.get())();
  std::cout << outcome << "\n";

  rclcpp::shutdown();

  return 0;
}
```

</details>

#### Action Demo (FSM + ROS 2 Action)

```shell
$ ros2 run action_tutorials_cpp fibonacci_action_server
```

```shell
$ ros2 run yasmin_demo action_client_demo
```

<details>
<summary>Click to expand</summary>

```cpp
#include <iostream>
#include <memory>
#include <string>

#include "action_tutorials_interfaces/action/fibonacci.hpp"

#include "yasmin/cb_state.hpp"
#include "yasmin/state_machine.hpp"
#include "yasmin_ros/action_state.hpp"
#include "yasmin_ros/basic_outcomes.hpp"
#include "yasmin_ros/yasmin_node.hpp"
#include "yasmin_viewer/yasmin_viewer_pub.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

std::string
set_int(std::shared_ptr<yasmin::blackboard::Blackboard> blackboard) {
  blackboard->set<int>("n", 3);
  return yasmin_ros::basic_outcomes::SUCCEED;
}

std::string
print_result(std::shared_ptr<yasmin::blackboard::Blackboard> blackboard) {

  auto fibo_res = blackboard->get<std::vector<int>>("sum");

  fprintf(stderr, "Sum:");

  for (auto ele : fibo_res) {
    fprintf(stderr, " %d,", ele);
  }

  fprintf(stderr, "\n");

  return yasmin_ros::basic_outcomes::SUCCEED;
}

class FibonacciState : public yasmin_ros::ActionState<
                           action_tutorials_interfaces::action::Fibonacci> {

public:
  FibonacciState()
      : yasmin_ros::ActionState<
            action_tutorials_interfaces::action::Fibonacci> // msg
        ("/fibonacci",                                      // action name
         std::bind(&FibonacciState::create_goal_handler, this, _1),
         std::bind(&FibonacciState::response_handler, this, _1, _2)){};

  action_tutorials_interfaces::action::Fibonacci::Goal create_goal_handler(
      std::shared_ptr<yasmin::blackboard::Blackboard> blackboard) {

    auto goal = action_tutorials_interfaces::action::Fibonacci::Goal();
    goal.order = blackboard->get<int>("n");

    return goal;
  }

  std::string response_handler(
      std::shared_ptr<yasmin::blackboard::Blackboard> blackboard,
      action_tutorials_interfaces::action::Fibonacci::Result::SharedPtr
          response) {

    blackboard->set<std::vector<int>>("sum", response->sequence);

    return yasmin_ros::basic_outcomes::SUCCEED;
  }

  std::string to_string() { return "FibonacciState"; }
};

int main(int argc, char *argv[]) {

  std::cout << "yasmin_action_client_demo\n";
  rclcpp::init(argc, argv);

  // create a state machine
  auto sm = std::make_shared<yasmin::StateMachine>(
      yasmin::StateMachine({"outcome4"}));

  // add states
  sm->add_state("SETTING_INT",
                std::make_shared<yasmin::CbState>(yasmin::CbState(
                    {yasmin_ros::basic_outcomes::SUCCEED}, set_int)),
                {{yasmin_ros::basic_outcomes::SUCCEED, "CALLING_FIBONACCI"}});
  sm->add_state("CALLING_FIBONACCI", std::make_shared<FibonacciState>(),
                {{yasmin_ros::basic_outcomes::SUCCEED, "PRINTING_RESULT"},
                 {yasmin_ros::basic_outcomes::CANCEL, "outcome4"},
                 {yasmin_ros::basic_outcomes::ABORT, "outcome4"}});
  sm->add_state("PRINTING_RESULT",
                std::make_shared<yasmin::CbState>(yasmin::CbState(
                    {yasmin_ros::basic_outcomes::SUCCEED}, print_result)),
                {{yasmin_ros::basic_outcomes::SUCCEED, "outcome4"}});

  // pub
  yasmin_viewer::YasminViewerPub yasmin_pub("YASMIN_ACTION_CLIENT_DEMO", sm);

  // execute
  std::string outcome = (*sm.get())();
  std::cout << outcome << "\n";

  rclcpp::shutdown();

  return 0;
}
```

</details>

#### Monitor Demo (FSM + ROS 2 Subscriber)

```shell
$ ros2 run yasmin_demo monitor_demo
```

<details>
<summary>Click to expand</summary>

```cpp
#include <iostream>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "nav_msgs/msg/odometry.hpp"

#include "yasmin/state_machine.hpp"
#include "yasmin_ros/basic_outcomes.hpp"
#include "yasmin_ros/monitor_state.hpp"
#include "yasmin_viewer/yasmin_viewer_pub.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

class PrintOdometryState
    : public yasmin_ros::MonitorState<nav_msgs::msg::Odometry> {

public:
  int times;

  PrintOdometryState(int times)
      : yasmin_ros::MonitorState<nav_msgs::msg::Odometry> // msg type
        ("odom",                                          // topic name
         {"outcome1", "outcome2"},                        // outcomes
         std::bind(&PrintOdometryState::monitor_handler, this, _1,
                   _2), // monitor handler callback
         10,            // qos for the topic sbscription
         10,            // queue of the monitor handler callback
         10             // timeout to wait for msgs in seconds
                        // if >0, CANCEL outcome is added
        ) {
    this->times = times;
  };

  std::string
  monitor_handler(std::shared_ptr<yasmin::blackboard::Blackboard> blackboard,
                  std::shared_ptr<nav_msgs::msg::Odometry> msg) {

    (void)blackboard;

    std::cout << "x: " << msg->pose.pose.position.x << "\n";
    std::cout << "y: " << msg->pose.pose.position.y << "\n";
    std::cout << "z: " << msg->pose.pose.position.z << "\n";
    std::cout << "\n";

    this->times--;

    if (this->times <= 0) {
      return "outcome2";
    }

    return "outcome1";
  }

  std::string to_string() { return "PrintOdometryState"; }
};

int main(int argc, char *argv[]) {

  std::cout << "yasmin_monitor_demo\n";
  rclcpp::init(argc, argv);

  // create a state machine
  auto sm = std::make_shared<yasmin::StateMachine>(
      yasmin::StateMachine({"outcome4"}));

  // add states
  sm->add_state("PRINTING_ODOM", std::make_shared<PrintOdometryState>(5),
                {{"outcome1", "PRINTING_ODOM"},
                 {"outcome2", "outcome4"},
                 {yasmin_ros::basic_outcomes::CANCEL, "outcome4"}});

  // pub
  yasmin_viewer::YasminViewerPub yasmin_pub("YASMIN_ACTION_CLIENT_DEMO", sm);

  // execute
  std::string outcome = (*sm.get())();
  std::cout << outcome << "\n";

  rclcpp::shutdown();

  return 0;
}
```

</details>

<a name="#YASMIN-Viewer"></a>

## YASMIN Viewer

This viewer allows monitoring YASMIN's FSM. It is implemented with Flask and ReactJS. A filter is provided to show only one FSM.

![](./docs/viewer.gif)

### Usage

```shell
$ ros2 run yasmin_viewer yasmin_viewer_node
```

http://localhost:5000/

### Custom host and port

```shell
$ ros2 run yasmin_viewer yasmin_viewer_node --ros-args -p host:=127.0.0.1 -p port:=5032
```

http://127.0.0.1:5032/

## Citations

```bibtex
@InProceedings{10.1007/978-3-031-21062-4_43,
author="Gonz{\'a}lez-Santamarta, Miguel {\'A}.
and Rodr{\'i}guez-Lera, Francisco J.
and Matell{\'a}n-Olivera, Vicente
and Fern{\'a}ndez-Llamas, Camino",
editor="Tardioli, Danilo
and Matell{\'a}n, Vicente
and Heredia, Guillermo
and Silva, Manuel F.
and Marques, Lino",
title="YASMIN: Yet Another State MachINe",
booktitle="ROBOT2022: Fifth Iberian Robotics Conference",
year="2023",
publisher="Springer International Publishing",
address="Cham",
pages="528--539",
abstract="State machines are a common mechanism for defining behaviors in robots where each behavior is based on identifiable stages. There are several libraries available for easing the implementation of state machines in ROS 1, however, the community was focused on SMACH or SMACC. Although these tools are still predominant, there are fewer alternatives for ROS 2. Besides, Behavior Trees are spreading fast, but there is a niche for using State Machines. Here, YASMIN is presented as yet another library specifically designed for ROS 2 for easing the design of robotic behaviors using state machines. It is available in C++ and Python, and provides some default states to speed up the development, in addition to a web viewer for monitoring the execution of the system and helping in the debugging.",
isbn="978-3-031-21062-4"
}

```

```bibtex
@misc{yasmin,
  doi = {10.48550/ARXIV.2205.13284},
  url = {https://arxiv.org/abs/2205.13284},
  author = {González-Santamarta, Miguel Ángel and Rodríguez-Lera, Francisco Javier and Llamas, Camino Fernández and Rico, Francisco Martín and Olivera, Vicente Matellán},
  keywords = {Robotics (cs.RO), FOS: Computer and information sciences, FOS: Computer and information sciences},
  title = {YASMIN: Yet Another State MachINe library for ROS 2},
  publisher = {arXiv},
  year = {2022},
  copyright = {Creative Commons Attribution Non Commercial No Derivatives 4.0 International}
}
```
