# YASMIN (Yet Another State MachINe)

<p align="center">
  <img src="./docs/logo.png" width="50%" />
</p>

YASMIN is a project focused on implementing robot behaviors using Finite State Machines (FSM). It is available for ROS 2, Python and C++.

[![License: MIT](https://img.shields.io/badge/GitHub-GPL--3.0-informational)](https://opensource.org/license/gpl-3-0) [![GitHub release](https://img.shields.io/github/release/uleroboticsgroup/yasmin.svg)](https://github.com/uleroboticsgroup/yasmin/releases) [![Code Size](https://img.shields.io/github/languages/code-size/uleroboticsgroup/yasmin.svg?branch=main)](https://github.com/uleroboticsgroup/yasmin?branch=main) [![Dependencies](https://img.shields.io/librariesio/github/uleroboticsgroup/yasmin?branch=main)](https://libraries.io/github/uleroboticsgroup/yasmin?branch=main) [![Last Commit](https://img.shields.io/github/last-commit/uleroboticsgroup/yasmin.svg)](https://github.com/uleroboticsgroup/yasmin/commits/main) [![GitHub issues](https://img.shields.io/github/issues/uleroboticsgroup/yasmin)](https://github.com/uleroboticsgroup/yasmin/issues) [![GitHub pull requests](https://img.shields.io/github/issues-pr/uleroboticsgroup/yasmin)](https://github.com/uleroboticsgroup/yasmin/pulls) [![Contributors](https://img.shields.io/github/contributors/uleroboticsgroup/yasmin.svg)](https://github.com/uleroboticsgroup/yasmin/graphs/contributors) [![Python Formatter Check](https://github.com/uleroboticsgroup/yasmin/actions/workflows/python_formatter.yml/badge.svg?branch=main)](https://github.com/uleroboticsgroup/yasmin/actions/workflows/python_formatter.yml?branch=main) [![C++ Formatter Check](https://github.com/uleroboticsgroup/yasmin/actions/workflows/cpp_formatter.yml/badge.svg?branch=main)](https://github.com/uleroboticsgroup/yasmin/actions/workflows/cpp_formatter.yml?branch=main)

<div align="center">

| ROS 2 Distro |                             Branch                             |                                                                                                             Build status                                                                                                              |                                                               Docker Image                                                                | Documentation                                                                                                                                               |
| :----------: | :------------------------------------------------------------: | :-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------: | :---------------------------------------------------------------------------------------------------------------------------------------: | ----------------------------------------------------------------------------------------------------------------------------------------------------------- |
|   **Foxy**   | [`main`](https://github.com/uleroboticsgroup/yasmin/tree/main) |       [![Foxy Build](https://github.com/uleroboticsgroup/yasmin/actions/workflows/foxy-docker-build.yml/badge.svg?branch=main)](https://github.com/uleroboticsgroup/yasmin/actions/workflows/foxy-docker-build.yml?branch=main)       |     [![Docker Image](https://img.shields.io/badge/Docker%20Image%20-foxy-blue)](https://hub.docker.com/r/mgons/yasmin/tags?name=foxy)     | [![Doxygen Deployment](https://github.com/uleroboticsgroup/yasmin/actions/workflows/doxygen-doc.yml/badge.svg)](https://uleroboticsgroup.github.io/yasmin/) |
| **Galatic**  | [`main`](https://github.com/uleroboticsgroup/yasmin/tree/main) | [![Galactic Build](https://github.com/uleroboticsgroup/yasmin/actions/workflows/galactic-docker-build.yml/badge.svg?branch=main)](https://github.com/uleroboticsgroup/yasmin/actions/workflows/galactic-docker-build.yml?branch=main) | [![Docker Image](https://img.shields.io/badge/Docker%20Image%20-galactic-blue)](https://hub.docker.com/r/mgons/yasmin/tags?name=galactic) | [![Doxygen Deployment](https://github.com/uleroboticsgroup/yasmin/actions/workflows/doxygen-doc.yml/badge.svg)](https://uleroboticsgroup.github.io/yasmin/) |
|  **Humble**  | [`main`](https://github.com/uleroboticsgroup/yasmin/tree/main) |    [![Humble Build](https://github.com/uleroboticsgroup/yasmin/actions/workflows/humble-docker-build.yml/badge.svg?branch=main)](https://github.com/uleroboticsgroup/yasmin/actions/workflows/humble-docker-build.yml?branch=main)    |   [![Docker Image](https://img.shields.io/badge/Docker%20Image%20-humble-blue)](https://hub.docker.com/r/mgons/yasmin/tags?name=humble)   | [![Doxygen Deployment](https://github.com/uleroboticsgroup/yasmin/actions/workflows/doxygen-doc.yml/badge.svg)](https://uleroboticsgroup.github.io/yasmin/) |
|   **Iron**   | [`main`](https://github.com/uleroboticsgroup/yasmin/tree/main) |       [![Iron Build](https://github.com/uleroboticsgroup/yasmin/actions/workflows/iron-docker-build.yml/badge.svg?branch=main)](https://github.com/uleroboticsgroup/yasmin/actions/workflows/iron-docker-build.yml?branch=main)       |     [![Docker Image](https://img.shields.io/badge/Docker%20Image%20-iron-blue)](https://hub.docker.com/r/mgons/yasmin/tags?name=iron)     | [![Doxygen Deployment](https://github.com/uleroboticsgroup/yasmin/actions/workflows/doxygen-doc.yml/badge.svg)](https://uleroboticsgroup.github.io/yasmin/) |
|  **Jazzy**   | [`main`](https://github.com/uleroboticsgroup/yasmin/tree/main) |     [![Jazzy Build](https://github.com/uleroboticsgroup/yasmin/actions/workflows/jazzy-docker-build.yml/badge.svg?branch=main)](https://github.com/uleroboticsgroup/yasmin/actions/workflows/jazzy-docker-build.yml?branch=main)      |    [![Docker Image](https://img.shields.io/badge/Docker%20Image%20-jazzy-blue)](https://hub.docker.com/r/mgons/yasmin/tags?name=jazzy)    | [![Doxygen Deployment](https://github.com/uleroboticsgroup/yasmin/actions/workflows/doxygen-doc.yml/badge.svg)](https://uleroboticsgroup.github.io/yasmin/) |
| **Rolling**  | [`main`](https://github.com/uleroboticsgroup/yasmin/tree/main) |  [![Rolling Build](https://github.com/uleroboticsgroup/yasmin/actions/workflows/rolling-docker-build.yml/badge.svg?branch=main)](https://github.com/uleroboticsgroup/yasmin/actions/workflows/rolling-docker-build.yml?branch=main)   |  [![Docker Image](https://img.shields.io/badge/Docker%20Image%20-rolling-blue)](https://hub.docker.com/r/mgons/yasmin/tags?name=rolling)  | [![Doxygen Deployment](https://github.com/uleroboticsgroup/yasmin/actions/workflows/doxygen-doc.yml/badge.svg)](https://uleroboticsgroup.github.io/yasmin/) |

</div>

## Table of Contents

1. [Features](#features)
2. [Installation](#installation)
3. [Docker](#docker)
4. [Demos](#demos)
   - [Python](#python)
   - [Cpp](#cpp)
5. [YASMIN Viewer](#yasmin-viewer)
6. [Citations](#citations)

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
$ cd ~/ros2_ws
$ rosdep install --from-paths src --ignore-src -r -y
$ cd src/yasmin
$ pip3 install -r requirements.txt

# colcon
$ cd ~/ros2_ws
$ colcon build
```

## Docker

If your operating system doesn't support ROS 2 humble, docker is a great alternative. First of all, you have to build the project and create an image like so:

```shell
## Assuming you are in the correct project directory
$ docker build -t yasmin .
```

To use a shortcut, you may use the following command:

```shell
## Assuming you are in the correct project directory
$ make docker_build
```

After the image is created, run a docker container with the following command:

```shell
## Assuming you are in the correct project directory
$ docker run -it --net=host --ipc=host --privileged --env="DISPLAY" --env="QT_X11_NO_MITSHM=1" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" --volume="${XAUTHORITY}:/root/.Xauthority" --entrypoint /bin/bash yasmin
```

To use a shortcut, you may use following command:

```shell
$ make docker_run
```

### Running the docker image

If you are in the docker image , this project is already sourced and the demo script can be run as the following command;

```shell
$ cd /root/ros2_ws/
$ ros2 run yasmin_demos yasmin_demo.py
```

## Demos

There are some examples, for both Python and C++, that can be found in [yasmin_demos](./yasmin_demos/).

### Python

#### Vanilla Demo (FSM)

```shell
$ ros2 run yasmin_demos yasmin_demo.py
```

<p align="center">
  <img src="./docs/demo.gif" width="65%" />
</p>

<details>
<summary>Click to expand</summary>

```python
import time
import rclpy

import yasmin
from yasmin import State
from yasmin import Blackboard
from yasmin import StateMachine
from yasmin_ros.ros_logs import set_ros_loggers
from yasmin_viewer import YasminViewerPub


# define state Foo
class FooState(State):
    def __init__(self) -> None:
        super().__init__(["outcome1", "outcome2"])
        self.counter = 0

    def execute(self, blackboard: Blackboard) -> str:
        yasmin.YASMIN_LOG_INFO("Executing state FOO")
        time.sleep(3)

        if self.counter < 3:
            self.counter += 1
            blackboard["foo_str"] = f"Counter: {self.counter}"
            return "outcome1"
        else:
            return "outcome2"


# define state Bar
class BarState(State):
    def __init__(self) -> None:
        super().__init__(outcomes=["outcome3"])

    def execute(self, blackboard: Blackboard) -> str:
        yasmin.YASMIN_LOG_INFO("Executing state BAR")
        time.sleep(3)

        yasmin.YASMIN_LOG_INFO(blackboard["foo_str"])
        return "outcome3"


# main
def main():

    yasmin.YASMIN_LOG_INFO("yasmin_demo")

    # init ROS 2
    rclpy.init()

    # set ROS 2 logs
    set_ros_loggers()

    # create a FSM
    sm = StateMachine(outcomes=["outcome4"])

    # add states
    sm.add_state(
        "FOO",
        FooState(),
        transitions={
            "outcome1": "BAR",
            "outcome2": "outcome4",
        },
    )
    sm.add_state(
        "BAR",
        BarState(),
        transitions={
            "outcome3": "FOO",
        },
    )
    # pub FSM info
    YasminViewerPub("yasmin_demo", sm)

    # execute FSM
    try:
        outcome = sm()
        yasmin.YASMIN_LOG_INFO(outcome)
    except KeyboardInterrupt:
        sm.cancel_state()

    # shutdown ROS 2
    if rclpy.ok():
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
$ ros2 run yasmin_demos service_client_demo.py
```

<details>
<summary>Click to expand</summary>

```python
import rclpy
from example_interfaces.srv import AddTwoInts

import yasmin
from yasmin import CbState
from yasmin import Blackboard
from yasmin import StateMachine
from yasmin_ros import ServiceState
from yasmin_ros.ros_logs import set_ros_loggers
from yasmin_ros.basic_outcomes import SUCCEED, ABORT
from yasmin_viewer import YasminViewerPub


class AddTwoIntsState(ServiceState):
    def __init__(self) -> None:
        super().__init__(
            AddTwoInts,  # srv type
            "/add_two_ints",  # service name
            self.create_request_handler,  # cb to create the request
            ["outcome1"],  # outcomes. Includes (SUCCEED, ABORT)
            self.response_handler,  # cb to process the response
        )

    def create_request_handler(self, blackboard: Blackboard) -> AddTwoInts.Request:

        req = AddTwoInts.Request()
        req.a = blackboard["a"]
        req.b = blackboard["b"]
        return req

    def response_handler(
        self, blackboard: Blackboard, response: AddTwoInts.Response
    ) -> str:

        blackboard["sum"] = response.sum
        return "outcome1"


def set_ints(blackboard: Blackboard) -> str:
    blackboard["a"] = 10
    blackboard["b"] = 5
    return SUCCEED


def print_sum(blackboard: Blackboard) -> str:
    yasmin.YASMIN_LOG_INFO(f"Sum: {blackboard['sum']}")
    return SUCCEED


# main
def main():

    yasmin.YASMIN_LOG_INFO("yasmin_service_client_demo")

    # init ROS 2
    rclpy.init()

    # set ROS 2 logs
    set_ros_loggers()

    # create a FSM
    sm = StateMachine(outcomes=["outcome4"])

    # add states
    sm.add_state(
        "SETTING_INTS",
        CbState([SUCCEED], set_ints),
        transitions={SUCCEED: "ADD_TWO_INTS"},
    )
    sm.add_state(
        "ADD_TWO_INTS",
        AddTwoIntsState(),
        transitions={
            "outcome1": "PRINTING_SUM",
            SUCCEED: "outcome4",
            ABORT: "outcome4",
        },
    )
    sm.add_state(
        "PRINTING_SUM",
        CbState([SUCCEED], print_sum),
        transitions={
            SUCCEED: "outcome4",
        },
    )

    # pub FSM info
    YasminViewerPub("YASMIN_SERVICE_CLIENT_DEMO", sm)

    # execute FSM
    try:
        outcome = sm()
        yasmin.YASMIN_LOG_INFO(outcome)
    except KeyboardInterrupt:
        sm.cancel_state()

    # shutdown ROS 2
    if rclpy.ok():
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
$ ros2 run yasmin_demos action_client_demo.py
```

<details>
<summary>Click to expand</summary>

```python
import rclpy
from action_tutorials_interfaces.action import Fibonacci

import yasmin
from yasmin import CbState
from yasmin import Blackboard
from yasmin import StateMachine
from yasmin_ros import ActionState
from yasmin_ros.ros_logs import set_ros_loggers
from yasmin_ros.basic_outcomes import SUCCEED, ABORT, CANCEL
from yasmin_viewer import YasminViewerPub


class FibonacciState(ActionState):
    def __init__(self) -> None:
        super().__init__(
            Fibonacci,  # action type
            "/fibonacci",  # action name
            self.create_goal_handler,  # cb to create the goal
            None,  # outcomes. Includes (SUCCEED, ABORT, CANCEL)
            self.response_handler,  # cb to process the response
            self.print_feedback,  # cb to process the feedback
        )

    def create_goal_handler(self, blackboard: Blackboard) -> Fibonacci.Goal:

        goal = Fibonacci.Goal()
        goal.order = blackboard["n"]
        return goal

    def response_handler(
        self, blackboard: Blackboard, response: Fibonacci.Result
    ) -> str:

        blackboard["fibo_res"] = response.sequence
        return SUCCEED

    def print_feedback(
        self, blackboard: Blackboard, feedback: Fibonacci.Feedback
    ) -> None:
        yasmin.YASMIN_LOG_INFO(f"Received feedback: {list(feedback.partial_sequence)}")


def print_result(blackboard: Blackboard) -> str:
    yasmin.YASMIN_LOG_INFO(f"Result: {blackboard['fibo_res']}")
    return SUCCEED


# main
def main():

    yasmin.YASMIN_LOG_INFO("yasmin_action_client_demo")

    # init ROS 2
    rclpy.init()

    # set ROS 2 logs
    set_ros_loggers()

    # create a FSM
    sm = StateMachine(outcomes=["outcome4"])

    # add states
    sm.add_state(
        "CALLING_FIBONACCI",
        FibonacciState(),
        transitions={
            SUCCEED: "PRINTING_RESULT",
            CANCEL: "outcome4",
            ABORT: "outcome4",
        },
    )
    sm.add_state(
        "PRINTING_RESULT",
        CbState([SUCCEED], print_result),
        transitions={
            SUCCEED: "outcome4",
        },
    )

    # pub FSM info
    YasminViewerPub("YASMIN_ACTION_CLIENT_DEMO", sm)

    # create an initial blackoard
    blackboard = Blackboard()
    blackboard["n"] = 10

    # execute FSM
    try:
        outcome = sm(blackboard)
        yasmin.YASMIN_LOG_INFO(outcome)
    except KeyboardInterrupt:
        sm.cancel_state()

    # shutdown ROS 2
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
```

</details>

#### Monitor Demo (FSM + ROS 2 Subscriber)

```shell
$ ros2 run yasmin_demos monitor_demo.py
```

<details>
<summary>Click to expand</summary>

```python
import rclpy
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry

import yasmin
from yasmin import Blackboard
from yasmin import StateMachine
from yasmin_ros import MonitorState
from yasmin_ros.ros_logs import set_ros_loggers
from yasmin_ros.basic_outcomes import TIMEOUT
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
            timeout=10,  # timeout to wait for msgs in seconds
            # if not None, CANCEL outcome is added
        )
        self.times = times

    def monitor_handler(self, blackboard: Blackboard, msg: Odometry) -> str:
        yasmin.YASMIN_LOG_INFO(msg)

        self.times -= 1

        if self.times <= 0:
            return "outcome2"

        return "outcome1"


# main
def main():

    yasmin.YASMIN_LOG_INFO("yasmin_monitor_demo")

    # init ROS 2
    rclpy.init()

    # set ROS 2 logs
    set_ros_loggers()

    # create a FSM
    sm = StateMachine(outcomes=["outcome4"])

    # add states
    sm.add_state(
        "PRINTING_ODOM",
        PrintOdometryState(5),
        transitions={
            "outcome1": "PRINTING_ODOM",
            "outcome2": "outcome4",
            TIMEOUT: "outcome4",
        },
    )

    # pub FSM info
    YasminViewerPub("YASMIN_MONITOR_DEMO", sm)

    # execute FSM
    try:
        outcome = sm()
        yasmin.YASMIN_LOG_INFO(outcome)
    except KeyboardInterrupt:
        sm.cancel_state()

    # shutdown ROS 2
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
```

</details>

#### Nav2 Demo (Hierarchical FSM + ROS 2 Action)

```shell
$ ros2 run yasmin_demos nav_demo.py
```

<details>
<summary>Click to expand</summary>

```python
import random

import rclpy
from geometry_msgs.msg import Pose
from nav2_msgs.action import NavigateToPose

import yasmin
from yasmin import CbState
from yasmin import Blackboard
from yasmin import StateMachine
from yasmin_ros import ActionState
from yasmin_ros.ros_logs import set_ros_loggers
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
            None,  # cb to process the response
        )

    def create_goal_handler(self, blackboard: Blackboard) -> NavigateToPose.Goal:

        goal = NavigateToPose.Goal()
        goal.pose.pose = blackboard["pose"]
        goal.pose.header.frame_id = "map"
        return goal


def create_waypoints(blackboard: Blackboard) -> str:
    blackboard["waypoints"] = {
        "entrance": [1.25, 6.30, -0.78, 0.67],
        "bathroom": [4.89, 1.64, 0.0, 1.0],
        "livingroom": [1.55, 4.03, -0.69, 0.72],
        "kitchen": [3.79, 6.77, 0.99, 0.12],
        "bedroom": [7.50, 4.89, 0.76, 0.65],
    }
    return SUCCEED


def take_random_waypoint(blackboard) -> str:
    blackboard["random_waypoints"] = random.sample(
        list(blackboard["waypoints"].keys()), blackboard["waypoints_num"]
    )
    return SUCCEED


def get_next_waypoint(blackboard: Blackboard) -> str:

    if not blackboard["random_waypoints"]:
        return END

    wp_name = blackboard["random_waypoints"].pop(0)
    wp = blackboard["waypoints"][wp_name]

    pose = Pose()
    pose.position.x = wp[0]
    pose.position.y = wp[1]

    pose.orientation.z = wp[2]
    pose.orientation.w = wp[3]

    blackboard["pose"] = pose
    blackboard["text"] = f"I have reached waypoint {wp_name}"

    return HAS_NEXT


# main
def main():

    yasmin.YASMIN_LOG_INFO("yasmin_nav2_demo")

    # init ROS 2
    rclpy.init()

    # set ROS 2 logs
    set_ros_loggers()

    # create state machines
    sm = StateMachine(outcomes=[SUCCEED, ABORT, CANCEL])
    nav_sm = StateMachine(outcomes=[SUCCEED, ABORT, CANCEL])

    # add states
    sm.add_state(
        "CREATING_WAYPOINTS",
        CbState([SUCCEED], create_waypoints),
        transitions={
            SUCCEED: "TAKING_RANDOM_WAYPOINTS",
        },
    )
    sm.add_state(
        "TAKING_RANDOM_WAYPOINTS",
        CbState([SUCCEED], take_random_waypoint),
        transitions={
            SUCCEED: "NAVIGATING",
        },
    )

    nav_sm.add_state(
        "GETTING_NEXT_WAYPOINT",
        CbState([END, HAS_NEXT], get_next_waypoint),
        transitions={
            END: SUCCEED,
            HAS_NEXT: "NAVIGATING",
        },
    )
    nav_sm.add_state(
        "NAVIGATING",
        Nav2State(),
        transitions={
            SUCCEED: "GETTING_NEXT_WAYPOINT",
            CANCEL: CANCEL,
            ABORT: ABORT,
        },
    )

    # pub FSM info
    YasminViewerPub("YASMIN_NAV_DEMO", sm)

    # execute FSM
    blackboard = Blackboard()
    blackboard["waypoints_num"] = 2

    try:
        outcome = sm(blackboard)
        yasmin.YASMIN_LOG_INFO(outcome)
    except KeyboardInterrupt:
        sm.cancel_state()

    # shutdown ROS 2
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
```

</details>

### Cpp

#### Vanilla Demo

```shell
$ ros2 run yasmin_demos yasmin_demo
```

<details>
<summary>Click to expand</summary>

```cpp
#include <chrono>
#include <iostream>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "yasmin/logs.hpp"
#include "yasmin/state.hpp"
#include "yasmin/state_machine.hpp"
#include "yasmin_ros/ros_logs.hpp"
#include "yasmin_viewer/yasmin_viewer_pub.hpp"

using namespace yasmin;

// define state Foo
class FooState : public yasmin::State {
public:
  int counter;

  FooState() : yasmin::State({"outcome1", "outcome2"}) { this->counter = 0; };

  std::string
  execute(std::shared_ptr<yasmin::blackboard::Blackboard> blackboard) {
    YASMIN_LOG_INFO("Executing state FOO");
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
};

// define state Bar
class BarState : public yasmin::State {
public:
  BarState() : yasmin::State({"outcome3"}){};

  std::string
  execute(std::shared_ptr<yasmin::blackboard::Blackboard> blackboard) {
    YASMIN_LOG_INFO("Executing state BAR");
    std::this_thread::sleep_for(std::chrono::seconds(3));

    YASMIN_LOG_INFO(blackboard->get<std::string>("foo_str").c_str());

    return "outcome3";
  }
};

int main(int argc, char *argv[]) {

  YASMIN_LOG_INFO("yasmin_demo");
  rclcpp::init(argc, argv);

  // set ROS 2 logs
  yasmin_ros::set_ros_loggers();

  // create a state machine
  auto sm = std::make_shared<yasmin::StateMachine>(
      std::initializer_list<std::string>{"outcome4"});

  // cancel state machine on ROS 2 shutdown
  rclcpp::on_shutdown([sm]() {
    if (sm->is_running()) {
      sm->cancel_state();
    }
  });

  // add states
  sm->add_state("FOO", std::make_shared<FooState>(),
                {
                    {"outcome1", "BAR"},
                    {"outcome2", "outcome4"},
                });
  sm->add_state("BAR", std::make_shared<BarState>(),
                {
                    {"outcome3", "FOO"},
                });

  // pub
  yasmin_viewer::YasminViewerPub yasmin_pub("yasmin_demo", sm);

  // execute
  try {
    std::string outcome = (*sm.get())();
    YASMIN_LOG_INFO(outcome.c_str());
  } catch (const std::exception &e) {
    YASMIN_LOG_WARN(e.what());
  }

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
$ ros2 run yasmin_demos service_client_demo
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
#include "yasmin/logs.hpp"
#include "yasmin/state_machine.hpp"
#include "yasmin_ros/basic_outcomes.hpp"
#include "yasmin_ros/ros_logs.hpp"
#include "yasmin_ros/service_state.hpp"
#include "yasmin_viewer/yasmin_viewer_pub.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
using namespace yasmin;

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
};

int main(int argc, char *argv[]) {

  YASMIN_LOG_INFO("yasmin_service_client_demo");
  rclcpp::init(argc, argv);

  // set ROS 2 logs
  yasmin_ros::set_ros_loggers();

  // create a state machine
  auto sm = std::make_shared<yasmin::StateMachine>(
      std::initializer_list<std::string>{"outcome4"});

  // cancel state machine on ROS 2 shutdown
  rclcpp::on_shutdown([sm]() {
    if (sm->is_running()) {
      sm->cancel_state();
    }
  });

  // add states
  sm->add_state("SETTING_INTS",
                std::make_shared<yasmin::CbState>(
                    std::initializer_list<std::string>{
                        yasmin_ros::basic_outcomes::SUCCEED},
                    set_ints),
                {
                    {yasmin_ros::basic_outcomes::SUCCEED, "ADD_TWO_INTS"},
                });
  sm->add_state("ADD_TWO_INTS", std::make_shared<AddTwoIntsState>(),
                {
                    {"outcome1", "PRINTING_SUM"},
                    {yasmin_ros::basic_outcomes::SUCCEED, "outcome4"},
                    {yasmin_ros::basic_outcomes::ABORT, "outcome4"},
                });
  sm->add_state("PRINTING_SUM",
                std::make_shared<yasmin::CbState>(
                    std::initializer_list<std::string>{
                        yasmin_ros::basic_outcomes::SUCCEED},
                    print_sum),
                {
                    {yasmin_ros::basic_outcomes::SUCCEED, "outcome4"},
                });

  // pub
  yasmin_viewer::YasminViewerPub yasmin_pub("YASMIN_ACTION_CLIENT_DEMO", sm);

  // execute
  try {
    std::string outcome = (*sm.get())();
    YASMIN_LOG_INFO(outcome.c_str());
  } catch (const std::exception &e) {
    YASMIN_LOG_WARN(e.what());
  }

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
$ ros2 run yasmin_demos action_client_demo
```

<details>
<summary>Click to expand</summary>

```cpp
#include <iostream>
#include <memory>
#include <string>

#include "action_tutorials_interfaces/action/fibonacci.hpp"

#include "yasmin/cb_state.hpp"
#include "yasmin/logs.hpp"
#include "yasmin/state_machine.hpp"
#include "yasmin_ros/action_state.hpp"
#include "yasmin_ros/basic_outcomes.hpp"
#include "yasmin_ros/ros_logs.hpp"
#include "yasmin_ros/yasmin_node.hpp"
#include "yasmin_viewer/yasmin_viewer_pub.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
using Fibonacci = action_tutorials_interfaces::action::Fibonacci;
using namespace yasmin;

std::string
print_result(std::shared_ptr<yasmin::blackboard::Blackboard> blackboard) {

  auto fibo_res = blackboard->get<std::vector<int>>("sum");

  fprintf(stderr, "Result received:");

  for (auto ele : fibo_res) {
    fprintf(stderr, " %d,", ele);
  }

  fprintf(stderr, "\n");

  return yasmin_ros::basic_outcomes::SUCCEED;
}

class FibonacciState : public yasmin_ros::ActionState<Fibonacci> {

public:
  FibonacciState()
      : yasmin_ros::ActionState<Fibonacci>(

            "/fibonacci", // action name

            // # cb to create the goal
            std::bind(&FibonacciState::create_goal_handler, this, _1),
            // # cb to process the response

            std::bind(&FibonacciState::response_handler, this, _1, _2),

            // cb to process the feedback
            std::bind(&FibonacciState::print_feedback, this, _1, _2)){};

  Fibonacci::Goal create_goal_handler(
      std::shared_ptr<yasmin::blackboard::Blackboard> blackboard) {

    auto goal = Fibonacci::Goal();
    goal.order = blackboard->get<int>("n");

    return goal;
  }

  std::string
  response_handler(std::shared_ptr<yasmin::blackboard::Blackboard> blackboard,
                   Fibonacci::Result::SharedPtr response) {

    blackboard->set<std::vector<int>>("sum", response->sequence);

    return yasmin_ros::basic_outcomes::SUCCEED;
  }

  void
  print_feedback(std::shared_ptr<yasmin::blackboard::Blackboard> blackboard,
                 std::shared_ptr<const Fibonacci::Feedback> feedback) {
    (void)blackboard;

    std::stringstream ss;
    ss << "Next number in sequence received: ";
    for (auto number : feedback->partial_sequence) {
      ss << number << " ";
    }

    fprintf(stderr, "%s\n", ss.str().c_str());
  }
};

int main(int argc, char *argv[]) {

  YASMIN_LOG_INFO("yasmin_action_client_demo");
  rclcpp::init(argc, argv);

  // set ROS 2 logs
  yasmin_ros::set_ros_loggers();

  // create a state machine
  auto sm = std::make_shared<yasmin::StateMachine>(
      std::initializer_list<std::string>{"outcome4"});

  // cancel state machine on ROS 2 shutdown
  rclcpp::on_shutdown([sm]() {
    if (sm->is_running()) {
      sm->cancel_state();
    }
  });

  // add states
  sm->add_state("CALLING_FIBONACCI", std::make_shared<FibonacciState>(),
                {
                    {yasmin_ros::basic_outcomes::SUCCEED, "PRINTING_RESULT"},
                    {yasmin_ros::basic_outcomes::CANCEL, "outcome4"},
                    {yasmin_ros::basic_outcomes::ABORT, "outcome4"},
                });
  sm->add_state("PRINTING_RESULT",
                std::make_shared<yasmin::CbState>(
                    std::initializer_list<std::string>{
                        yasmin_ros::basic_outcomes::SUCCEED},
                    print_result),
                {
                    {yasmin_ros::basic_outcomes::SUCCEED, "outcome4"},
                });

  // pub
  yasmin_viewer::YasminViewerPub yasmin_pub("YASMIN_ACTION_CLIENT_DEMO", sm);

  // create an initial blackboard
  std::shared_ptr<yasmin::blackboard::Blackboard> blackboard =
      std::make_shared<yasmin::blackboard::Blackboard>();
  blackboard->set<int>("n", 10);

  // execute
  try {
    std::string outcome = (*sm.get())(blackboard);
    YASMIN_LOG_INFO(outcome.c_str());
  } catch (const std::exception &e) {
    YASMIN_LOG_WARN(e.what());
  }

  rclcpp::shutdown();

  return 0;
}
```

</details>

#### Monitor Demo (FSM + ROS 2 Subscriber)

```shell
$ ros2 run yasmin_demos monitor_demo
```

<details>
<summary>Click to expand</summary>

```cpp
#include <iostream>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "nav_msgs/msg/odometry.hpp"

#include "yasmin/logs.hpp"
#include "yasmin/state_machine.hpp"
#include "yasmin_ros/basic_outcomes.hpp"
#include "yasmin_ros/monitor_state.hpp"
#include "yasmin_ros/ros_logs.hpp"
#include "yasmin_viewer/yasmin_viewer_pub.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
using namespace yasmin;

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

    YASMIN_LOG_INFO("x: %d", msg->pose.pose.position.x);
    YASMIN_LOG_INFO("y: %d", msg->pose.pose.position.y);
    YASMIN_LOG_INFO("z: %d", msg->pose.pose.position.z);

    this->times--;

    if (this->times <= 0) {
      return "outcome2";
    }

    return "outcome1";
  }
};

int main(int argc, char *argv[]) {

  YASMIN_LOG_INFO("yasmin_monitor_demo");
  rclcpp::init(argc, argv);

  // set ROS 2 logs
  yasmin_ros::set_ros_loggers();

  // create a state machine
  auto sm = std::make_shared<yasmin::StateMachine>(
      std::initializer_list<std::string>{"outcome4"});

  // cancel state machine on ROS 2 shutdown
  rclcpp::on_shutdown([sm]() {
    if (sm->is_running()) {
      sm->cancel_state();
    }
  });

  // add states
  sm->add_state("PRINTING_ODOM", std::make_shared<PrintOdometryState>(5),
                {
                    {"outcome1", "PRINTING_ODOM"},
                    {"outcome2", "outcome4"},
                    {yasmin_ros::basic_outcomes::TIMEOUT, "outcome4"},
                });

  // pub
  yasmin_viewer::YasminViewerPub yasmin_pub("YASMIN_MONITOR_DEMO", sm);

  // execute
  try {
    std::string outcome = (*sm.get())();
    YASMIN_LOG_INFO(outcome.c_str());
  } catch (const std::exception &e) {
    YASMIN_LOG_WARN(e.what());
  }

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
