# YASMIN (Yet Another State MachINe)

<p align="center">
  <img src="./docs/logo.png" width="50%" />
</p>

**YASMIN** is a project focused on implementing robot behaviors using Finite State Machines (FSM). It is available for ROS 2, Python and C++.

<div align="center">

[![License: MIT](https://img.shields.io/badge/GitHub-GPL--3.0-informational)](https://opensource.org/license/gpl-3-0) [![GitHub release](https://img.shields.io/github/release/uleroboticsgroup/yasmin.svg)](https://github.com/uleroboticsgroup/yasmin/releases) [![Code Size](https://img.shields.io/github/languages/code-size/uleroboticsgroup/yasmin.svg?branch=main)](https://github.com/uleroboticsgroup/yasmin?branch=main) [![Dependencies](https://img.shields.io/librariesio/github/uleroboticsgroup/yasmin?branch=main)](https://libraries.io/github/uleroboticsgroup/yasmin?branch=main) [![Last Commit](https://img.shields.io/github/last-commit/uleroboticsgroup/yasmin.svg)](https://github.com/uleroboticsgroup/yasmin/commits/main) [![GitHub issues](https://img.shields.io/github/issues/uleroboticsgroup/yasmin)](https://github.com/uleroboticsgroup/yasmin/issues) [![GitHub pull requests](https://img.shields.io/github/issues-pr/uleroboticsgroup/yasmin)](https://github.com/uleroboticsgroup/yasmin/pulls) [![Contributors](https://img.shields.io/github/contributors/uleroboticsgroup/yasmin.svg)](https://github.com/uleroboticsgroup/yasmin/graphs/contributors) [![Python Formatter Check](https://github.com/uleroboticsgroup/yasmin/actions/workflows/python-formatter.yml/badge.svg?branch=main)](https://github.com/uleroboticsgroup/yasmin/actions/workflows/python-formatter.yml?branch=main) [![C++ Formatter Check](https://github.com/uleroboticsgroup/yasmin/actions/workflows/cpp-formatter.yml/badge.svg?branch=main)](https://github.com/uleroboticsgroup/yasmin/actions/workflows/cpp-formatter.yml?branch=main)

| ROS 2 Distro |                             Branch                             |                                                                                                             Build status                                                                                                              |                                                               Docker Image                                                                | Documentation                                                                                                                                                            |
| :----------: | :------------------------------------------------------------: | :-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------: | :---------------------------------------------------------------------------------------------------------------------------------------: | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
|   **Foxy**   | [`main`](https://github.com/uleroboticsgroup/yasmin/tree/main) |       [![Foxy Build](https://github.com/uleroboticsgroup/yasmin/actions/workflows/foxy-docker-build.yml/badge.svg?branch=main)](https://github.com/uleroboticsgroup/yasmin/actions/workflows/foxy-docker-build.yml?branch=main)       |     [![Docker Image](https://img.shields.io/badge/Docker%20Image%20-foxy-blue)](https://hub.docker.com/r/mgons/yasmin/tags?name=foxy)     | [![Doxygen Deployment](https://github.com/uleroboticsgroup/yasmin/actions/workflows/doxygen-deployment.yml/badge.svg)](https://uleroboticsgroup.github.io/yasmin/latest) |
| **Galatic**  | [`main`](https://github.com/uleroboticsgroup/yasmin/tree/main) | [![Galactic Build](https://github.com/uleroboticsgroup/yasmin/actions/workflows/galactic-docker-build.yml/badge.svg?branch=main)](https://github.com/uleroboticsgroup/yasmin/actions/workflows/galactic-docker-build.yml?branch=main) | [![Docker Image](https://img.shields.io/badge/Docker%20Image%20-galactic-blue)](https://hub.docker.com/r/mgons/yasmin/tags?name=galactic) | [![Doxygen Deployment](https://github.com/uleroboticsgroup/yasmin/actions/workflows/doxygen-deployment.yml/badge.svg)](https://uleroboticsgroup.github.io/yasmin/latest) |
|  **Humble**  | [`main`](https://github.com/uleroboticsgroup/yasmin/tree/main) |    [![Humble Build](https://github.com/uleroboticsgroup/yasmin/actions/workflows/humble-docker-build.yml/badge.svg?branch=main)](https://github.com/uleroboticsgroup/yasmin/actions/workflows/humble-docker-build.yml?branch=main)    |   [![Docker Image](https://img.shields.io/badge/Docker%20Image%20-humble-blue)](https://hub.docker.com/r/mgons/yasmin/tags?name=humble)   | [![Doxygen Deployment](https://github.com/uleroboticsgroup/yasmin/actions/workflows/doxygen-deployment.yml/badge.svg)](https://uleroboticsgroup.github.io/yasmin/latest) |
|   **Iron**   | [`main`](https://github.com/uleroboticsgroup/yasmin/tree/main) |       [![Iron Build](https://github.com/uleroboticsgroup/yasmin/actions/workflows/iron-docker-build.yml/badge.svg?branch=main)](https://github.com/uleroboticsgroup/yasmin/actions/workflows/iron-docker-build.yml?branch=main)       |     [![Docker Image](https://img.shields.io/badge/Docker%20Image%20-iron-blue)](https://hub.docker.com/r/mgons/yasmin/tags?name=iron)     | [![Doxygen Deployment](https://github.com/uleroboticsgroup/yasmin/actions/workflows/doxygen-deployment.yml/badge.svg)](https://uleroboticsgroup.github.io/yasmin/latest) |
|  **Jazzy**   | [`main`](https://github.com/uleroboticsgroup/yasmin/tree/main) |     [![Jazzy Build](https://github.com/uleroboticsgroup/yasmin/actions/workflows/jazzy-docker-build.yml/badge.svg?branch=main)](https://github.com/uleroboticsgroup/yasmin/actions/workflows/jazzy-docker-build.yml?branch=main)      |    [![Docker Image](https://img.shields.io/badge/Docker%20Image%20-jazzy-blue)](https://hub.docker.com/r/mgons/yasmin/tags?name=jazzy)    | [![Doxygen Deployment](https://github.com/uleroboticsgroup/yasmin/actions/workflows/doxygen-deployment.yml/badge.svg)](https://uleroboticsgroup.github.io/yasmin/latest) |
|  **Kilted**  | [`main`](https://github.com/uleroboticsgroup/yasmin/tree/main) |    [![Kilted Build](https://github.com/uleroboticsgroup/yasmin/actions/workflows/kilted-docker-build.yml/badge.svg?branch=main)](https://github.com/uleroboticsgroup/yasmin/actions/workflows/kilted-docker-build.yml?branch=main)    |   [![Docker Image](https://img.shields.io/badge/Docker%20Image%20-kilted-blue)](https://hub.docker.com/r/mgons/yasmin/tags?name=kilted)   | [![Doxygen Deployment](https://github.com/uleroboticsgroup/yasmin/actions/workflows/doxygen-deployment.yml/badge.svg)](https://uleroboticsgroup.github.io/yasmin/latest) |
| **Rolling**  | [`main`](https://github.com/uleroboticsgroup/yasmin/tree/main) |  [![Rolling Build](https://github.com/uleroboticsgroup/yasmin/actions/workflows/rolling-docker-build.yml/badge.svg?branch=main)](https://github.com/uleroboticsgroup/yasmin/actions/workflows/rolling-docker-build.yml?branch=main)   |  [![Docker Image](https://img.shields.io/badge/Docker%20Image%20-rolling-blue)](https://hub.docker.com/r/mgons/yasmin/tags?name=rolling)  | [![Doxygen Deployment](https://github.com/uleroboticsgroup/yasmin/actions/workflows/doxygen-deployment.yml/badge.svg)](https://uleroboticsgroup.github.io/yasmin/latest) |

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

## Key Features

- **ROS 2 Integration**: Integrates with ROS 2 for easy deployment and interaction.
- **Support for Python and C++**: Available for both Python and C++, making it flexible for a variety of use cases.
- **Rapid Prototyping**: Designed for fast prototyping, allowing quick iteration of state machine behaviors.
- **Predefined States**: Includes states for interacting with ROS 2 action clients, service clients, and topics.
- **Data Sharing with Blackboards**: Utilizes blackboards for data sharing between states and state machines.
- **State Management**: Supports cancellation and stopping of state machines, including halting the current executing state.
- **Web Viewer**: Features an integrated web viewer for real-time monitoring of state machine execution.

## Installation

### Debian Packages

To install YASMIN and its packages, use the following command:

```shell
sudo apt install ros-$ROS_DISTRO-yasmin ros-$ROS_DISTRO-yasmin-*
```

### Building from Source

Follow these steps to build the source code from this repository:

```shell
cd ~/ros2_ws/src
git clone https://github.com/uleroboticsgroup/yasmin.git
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
cd ~/ros2_ws
colcon build
```

#### Note for Deprecated or Rolling ROS 2 Distros

If you are using a deprecated ROS 2 distribution (e.g., Foxy or Galactic) or the Rolling distribution, install the example interfaces:

```shell
sudo apt install -y ros-$ROS_DISTRO-example-interfaces
```

## Docker

If your operating system doesn't support ROS 2, docker is a great alternative. You can use an image from [Dockerhub](https://hub.docker.com/r/mgons/yasmin/) or create your own images. First of all, to build the image you have to use the following command:

```shell
## Assuming you are in the YASMIN project directory
docker build -t yasmin .
```

To use a shortcut the docker build, you may use the following command:

```shell
## Assuming you are in the YASMIN project directory
make docker_build
```

After the image is created, run a docker container with the following command:

```shell
docker run -it --net=host --ipc=host --privileged --env="DISPLAY" --env="QT_X11_NO_MITSHM=1" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" --volume="${XAUTHORITY}:/root/.Xauthority" --entrypoint /bin/bash yasmin
```

To use a shortcut the docker run, you may use following command:

```shell
## Assuming you are in the YASMIN project directory
make docker_run
```

## Demos

There are some examples, for both Python and C++, that can be found in [yasmin_demos](./yasmin_demos/).

### Python

#### Vanilla Demo (FSM)

```shell
ros2 run yasmin_demos yasmin_demo.py
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
from yasmin_ros import set_ros_loggers
from yasmin_viewer import YasminViewerPub


# Define the FooState class, inheriting from the State class
class FooState(State):
    """
    Represents the Foo state in the state machine.

    Attributes:
        counter (int): Counter to track the number of executions of this state.
    """

    def __init__(self) -> None:
        """
        Initializes the FooState instance, setting up the outcomes.

        Outcomes:
            outcome1: Indicates the state should continue to the Bar state.
            outcome2: Indicates the state should finish execution and return.
        """
        super().__init__(["outcome1", "outcome2"])
        self.counter = 0

    def execute(self, blackboard: Blackboard) -> str:
        """
        Executes the logic for the Foo state.

        Args:
            blackboard (Blackboard): The shared data structure for states.

        Returns:
            str: The outcome of the execution, which can be "outcome1" or "outcome2".

        Raises:
            Exception: May raise exceptions related to state execution.
        """
        yasmin.YASMIN_LOG_INFO("Executing state FOO")
        time.sleep(3)  # Simulate work by sleeping

        if self.counter < 3:
            self.counter += 1
            blackboard["foo_str"] = f"Counter: {self.counter}"
            return "outcome1"
        else:
            return "outcome2"


# Define the BarState class, inheriting from the State class
class BarState(State):
    """
    Represents the Bar state in the state machine.
    """

    def __init__(self) -> None:
        """
        Initializes the BarState instance, setting up the outcome.

        Outcomes:
            outcome3: Indicates the state should transition back to the Foo state.
        """
        super().__init__(outcomes=["outcome3"])

    def execute(self, blackboard: Blackboard) -> str:
        """
        Executes the logic for the Bar state.

        Args:
            blackboard (Blackboard): The shared data structure for states.

        Returns:
            str: The outcome of the execution, which will always be "outcome3".

        Raises:
            Exception: May raise exceptions related to state execution.
        """
        yasmin.YASMIN_LOG_INFO("Executing state BAR")
        time.sleep(3)  # Simulate work by sleeping

        yasmin.YASMIN_LOG_INFO(blackboard["foo_str"])
        return "outcome3"


# Main function to initialize and run the state machine
def main():
    """
    The main entry point of the application.

    Initializes the ROS 2 environment, sets up the state machine,
    and handles execution and termination.

    Raises:
        KeyboardInterrupt: If the execution is interrupted by the user.
    """
    yasmin.YASMIN_LOG_INFO("yasmin_demo")

    # Initialize ROS 2
    rclpy.init()

    # Set ROS 2 loggers
    set_ros_loggers()

    # Create a finite state machine (FSM)
    sm = StateMachine(outcomes=["outcome4"])

    # Add states to the FSM
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

    # Publish FSM information for visualization
    YasminViewerPub("YASMIN_DEMO", sm)

    # Execute the FSM
    try:
        outcome = sm()
        yasmin.YASMIN_LOG_INFO(outcome)
    except KeyboardInterrupt:
        if sm.is_running():
            sm.cancel_state()

    # Shutdown ROS 2 if it's running
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
```

</details>

#### Blackboard Remapping Demo

```shell
ros2 run yasmin_demos remap_demo.py
```

<details>
<summary>Click to expand</summary>

```python
import yasmin
from yasmin import State
from yasmin import Blackboard
from yasmin import StateMachine
from yasmin_ros.basic_outcomes import SUCCEED


class Foo(State):
    """
    Represents the Foo state in the state machine.
    """

    def __init__(self):
        """
        Initializes the FooState instance, setting up the outcomes.

        Outcomes:
            SUCCEED: Indicates the state should continue to the next state.
        """
        super().__init__(outcomes=[SUCCEED])

    def execute(self, blackboard: Blackboard):
        """
        Executes the logic for the Foo state.

        Args:
            blackboard (Blackboard): The shared data structure for states.

        Returns:
            str: The outcome of the execution, which can be SUCCEED.

        Raises:
            Exception: May raise exceptions related to state execution.
        """
        data = blackboard["foo_data"]
        yasmin.YASMIN_LOG_INFO(f"{data}")
        blackboard["foo_out_data"] = data
        return SUCCEED


class BarState(State):
    """
    Represents the Bar state in the state machine.

    """

    def __init__(self):
        """
        Initializes the BarState instance, setting up the outcomes.

        Outcomes:
            SUCCEDED: Indicates the state should continue to the next state.
        """
        super().__init__(outcomes=[SUCCEED])

    def execute(self, blackboard: Blackboard):
        """
        Executes the logic for the Bar state.

        Args:
            blackboard (Blackboard): The shared data structure for states.

        Returns:
            str: The outcome of the execution, which can be SUCCEED.

        Raises:
            Exception: May raise exceptions related to state execution.
        """
        data = blackboard["bar_data"]
        yasmin.YASMIN_LOG_INFO(f"{data}")
        return SUCCEED


if __name__ == "__main__":
    """
    The main entry point of the application.

    Initializes the ROS 2 environment, sets up the state machine,
    and handles execution and termination.

    Raises:
        KeyboardInterrupt: If the execution is interrupted by the user.
    """
    bb = Blackboard()
    bb["msg1"] = "test1"
    bb["msg2"] = "test2"

    sm = StateMachine(outcomes=[SUCCEED])
    sm.add_state(
        "STATE1",
        Foo(),
        transitions={SUCCEED: "STATE2"},
        remappings={"foo_data": "msg1"},
    )
    sm.add_state(
        "STATE2",
        Foo(),
        transitions={SUCCEED: "STATE3"},
        remappings={"foo_data": "msg2"},
    )
    sm.add_state(
        "STATE3",
        BarState(),
        transitions={SUCCEED: SUCCEED},
        remappings={"bar_data": "foo_out_data"},
    )

    sm.execute(bb)
```

</details>

#### Concurrence Demo

```shell
ros2 run yasmin_demos concurrence_demo.py
```

<details>
<summary>Click to expand</summary>

```python
import time
import rclpy

import yasmin
from yasmin import State
from yasmin import Concurrence
from yasmin import Blackboard
from yasmin import StateMachine
from yasmin_ros import set_ros_loggers
from yasmin_viewer import YasminViewerPub


# Define the FooState class, inheriting from the State class
class FooState(State):
    """
    Represents the Foo state in the state machine.

    Attributes:
        counter (int): Counter to track the number of executions of this state.
    """

    def __init__(self) -> None:
        """
        Initializes the FooState instance, setting up the outcomes.

        Outcomes:
            outcome1: Indicates the state should continue.
            outcome2: Indicates the state should cotninue.
            outcome3: Indicates the state should finish execution and return.
        """
        super().__init__(["outcome1", "outcome2", "outcome3"])
        self.counter = 0

    def execute(self, blackboard: Blackboard) -> str:
        """
        Executes the logic for the Foo state.

        Args:
            blackboard (Blackboard): The shared data structure for states.

        Returns:
            str: The outcome of the execution.

        Raises:
            Exception: May raise exceptions related to state execution.
        """
        yasmin.YASMIN_LOG_INFO("Executing state FOO")
        time.sleep(2)  # Simulate work by sleeping

        outcome = ""

        blackboard["foo_str"] = f"Counter: {self.counter}"

        if self.counter < 3:
            outcome = "outcome1"
        elif self.counter < 5:
            outcome = "outcome2"
        else:
            outcome = "outcome3"

        yasmin.YASMIN_LOG_INFO("Finishing state FOO")
        self.counter += 1
        return outcome


# Define the BarState class, inheriting from the State class
class BarState(State):
    """
    Represents the Bar state in the state machine.
    """

    def __init__(self) -> None:
        """
        Initializes the BarState instance, setting up the outcome.

        Outcomes:
            outcome3: This state will always return this outcome
        """
        super().__init__(outcomes=["outcome3"])

    def execute(self, blackboard: Blackboard) -> str:
        """
        Executes the logic for the Bar state.

        Args:
            blackboard (Blackboard): The shared data structure for states.

        Returns:
            str: The outcome of the execution, which will always be "outcome3".

        Raises:
            Exception: May raise exceptions related to state execution.
        """
        yasmin.YASMIN_LOG_INFO("Executing state BAR")
        time.sleep(4)  # Simulate work by sleeping

        if "foo_str" in blackboard:
            yasmin.YASMIN_LOG_INFO(blackboard["foo_str"])
        else:
            yasmin.YASMIN_LOG_INFO("Blackboard does not yet contain 'foo_str'")

        yasmin.YASMIN_LOG_INFO("Finishing state BAR")

        return "outcome3"


# Main function to initialize and run the state machine
def main():
    """
    The main entry point of the application.

    Initializes the ROS 2 environment, sets up the state machine,
    and handles execution and termination.

    Raises:
        KeyboardInterrupt: If the execution is interrupted by the user.
    """
    yasmin.YASMIN_LOG_INFO("yasmin_concurrence_demo")

    # Initialize ROS 2
    rclpy.init()

    # Set ROS 2 loggers
    set_ros_loggers()

    # Create a finite state machine (FSM)
    sm = StateMachine(outcomes=["outcome4"])

    # Create states to run concurrently
    foo_state: State = FooState()
    bar_state: State = BarState()

    # Add concurrence state
    concurrence_state = Concurrence(
        states=[foo_state, bar_state],
        default_outcome="defaulted",
        outcome_map={
            "outcome1": {foo_state: "outcome1", bar_state: "outcome3"},
            "outcome2": {foo_state: "outcome2", bar_state: "outcome3"},
        },
    )

    # Add concurrent state to the FSM
    sm.add_state(
        "CONCURRENCE",
        concurrence_state,
        transitions={
            "outcome1": "CONCURRENCE",
            "outcome2": "CONCURRENCE",
            "defaulted": "outcome4",
        },
    )

    # Publish FSM information for visualization
    YasminViewerPub("YASMIN_CONCURRENCE_DEMO", sm)

    # Execute the FSM
    try:
        outcome = sm()
        yasmin.YASMIN_LOG_INFO(outcome)
    except KeyboardInterrupt:
        if sm.is_running():
            sm.cancel_state()

    # Shutdown ROS 2 if it's running
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
```

</details>

#### Service Demo (FSM + ROS 2 Service Client)

```shell
ros2 run yasmin_demos add_two_ints_server
```

```shell
ros2 run yasmin_demos service_client_demo.py
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
from yasmin_ros import set_ros_loggers
from yasmin_ros.basic_outcomes import SUCCEED, ABORT
from yasmin_viewer import YasminViewerPub


class AddTwoIntsState(ServiceState):
    """
    A state that calls the AddTwoInts service to add two integers.

    This class is a state in a finite state machine that sends a request
    to the AddTwoInts service, retrieves the response, and updates the
    blackboard with the result.

    Attributes:
        service_type (type): The service type being used (AddTwoInts).
        service_name (str): The name of the service.
        outcomes (list): The list of possible outcomes for this state.
    """

    def __init__(self) -> None:
        """
        Initializes the AddTwoIntsState.

        Calls the parent constructor with the specific service type,
        service name, request handler, outcomes, and response handler.
        """
        super().__init__(
            AddTwoInts,  # srv type
            "/add_two_ints",  # service name
            self.create_request_handler,  # cb to create the request
            ["outcome1"],  # outcomes. Includes (SUCCEED, ABORT)
            self.response_handler,  # cb to process the response
        )

    def create_request_handler(self, blackboard: Blackboard) -> AddTwoInts.Request:
        """
        Creates the service request from the blackboard data.

        Args:
            blackboard (Blackboard): The blackboard containing the input values.

        Returns:
            AddTwoInts.Request: The request object populated with values from the blackboard.
        """
        req = AddTwoInts.Request()
        req.a = blackboard["a"]
        req.b = blackboard["b"]
        return req

    def response_handler(
        self, blackboard: Blackboard, response: AddTwoInts.Response
    ) -> str:
        """
        Processes the response from the AddTwoInts service.

        Updates the blackboard with the sum result from the response.

        Args:
            blackboard (Blackboard): The blackboard to update with the sum.
            response (AddTwoInts.Response): The response from the service call.

        Returns:
            str: The outcome of the operation, which is "outcome1".
        """
        blackboard["sum"] = response.sum
        return "outcome1"


def set_ints(blackboard: Blackboard) -> str:
    """
    Sets the integer values in the blackboard.

    This function initializes the blackboard with two integer values to be added.

    Args:
        blackboard (Blackboard): The blackboard to update with integer values.

    Returns:
        str: The outcome of the operation, which is SUCCEED.
    """
    blackboard["a"] = 10
    blackboard["b"] = 5
    return SUCCEED


def print_sum(blackboard: Blackboard) -> str:
    """
    Logs the sum value from the blackboard.

    This function retrieves the sum from the blackboard and logs it.

    Args:
        blackboard (Blackboard): The blackboard from which to retrieve the sum.

    Returns:
        str: The outcome of the operation, which is SUCCEED.
    """
    yasmin.YASMIN_LOG_INFO(f"Sum: {blackboard['sum']}")
    return SUCCEED


def main():
    """
    The main function to execute the finite state machine (FSM).

    This function initializes the ROS 2 environment, sets up logging,
    creates the FSM with defined states, and executes the FSM.

    Raises:
        KeyboardInterrupt: If the user interrupts the program.
    """
    yasmin.YASMIN_LOG_INFO("yasmin_service_client_demo")

    # Init ROS 2
    rclpy.init()

    # Set ROS 2 logs
    set_ros_loggers()

    # Create a FSM
    sm = StateMachine(outcomes=["outcome4"])

    # Add states
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

    # Publish FSM info
    YasminViewerPub("YASMIN_SERVICE_CLIENT_DEMO", sm)

    # Execute FSM
    try:
        outcome = sm()
        yasmin.YASMIN_LOG_INFO(outcome)
    except KeyboardInterrupt:
        if sm.is_running():
            sm.cancel_state()

    # Shutdown ROS 2
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
```

</details>

#### Action Demo (FSM + ROS 2 Action)

```shell
ros2 run yasmin_demos fibonacci_action_server
```

```shell
ros2 run yasmin_demos action_client_demo.py
```

<details>
<summary>Click to expand</summary>

```python
import rclpy
from example_interfaces.action import Fibonacci

import yasmin
from yasmin import CbState, Blackboard, StateMachine
from yasmin_ros import ActionState
from yasmin_ros import set_ros_loggers
from yasmin_ros.basic_outcomes import SUCCEED, ABORT, CANCEL
from yasmin_viewer import YasminViewerPub


class FibonacciState(ActionState):
    """
    Class representing the state of the Fibonacci action.

    Inherits from ActionState and implements methods to handle the
    Fibonacci action in a finite state machine.

    Attributes:
        None
    """

    def __init__(self) -> None:
        """
        Initializes the FibonacciState.

        Sets up the action type and the action name for the Fibonacci
        action. Initializes goal, response handler, and feedback
        processing callbacks.

        Parameters:
            None

        Returns:
            None
        """
        super().__init__(
            Fibonacci,  # action type
            "/fibonacci",  # action name
            self.create_goal_handler,  # callback to create the goal
            None,  # outcomes. Includes (SUCCEED, ABORT, CANCEL)
            self.response_handler,  # callback to process the response
            self.print_feedback,  # callback to process the feedback
        )

    def create_goal_handler(self, blackboard: Blackboard) -> Fibonacci.Goal:
        """
        Creates the goal for the Fibonacci action.

        This method retrieves the input value from the blackboard and
        populates the Fibonacci goal.

        Parameters:
            blackboard (Blackboard): The blackboard containing the state
            information.

        Returns:
            Fibonacci.Goal: The populated goal object for the Fibonacci action.

        Raises:
            KeyError: If the expected key is not present in the blackboard.
        """
        goal = Fibonacci.Goal()
        goal.order = blackboard["n"]  # Retrieve the input value 'n' from the blackboard
        return goal

    def response_handler(self, blackboard: Blackboard, response: Fibonacci.Result) -> str:
        """
        Handles the response from the Fibonacci action.

        This method processes the result of the Fibonacci action and
        stores it in the blackboard.

        Parameters:
            blackboard (Blackboard): The blackboard to store the result.
            response (Fibonacci.Result): The result object from the Fibonacci action.

        Returns:
            str: Outcome of the operation, typically SUCCEED.

        Raises:
            None
        """
        blackboard["fibo_res"] = (
            response.sequence
        )  # Store the result sequence in the blackboard
        return SUCCEED

    def print_feedback(
        self, blackboard: Blackboard, feedback: Fibonacci.Feedback
    ) -> None:
        """
        Prints feedback from the Fibonacci action.

        This method logs the partial sequence received during the action.

        Parameters:
            blackboard (Blackboard): The blackboard (not used in this method).
            feedback (Fibonacci.Feedback): The feedback object from the Fibonacci action.

        Returns:
            None

        Raises:
            None
        """
        yasmin.YASMIN_LOG_INFO(f"Received feedback: {list(feedback.sequence)}")


def print_result(blackboard: Blackboard) -> str:
    """
    Prints the result of the Fibonacci action.

    This function logs the final result stored in the blackboard.

    Parameters:
        blackboard (Blackboard): The blackboard containing the result.

    Returns:
        str: Outcome of the operation, typically SUCCEED.

    Raises:
        None
    """
    yasmin.YASMIN_LOG_INFO(f"Result: {blackboard['fibo_res']}")
    return SUCCEED


def main():
    """
    Main function to execute the ROS 2 action client demo.

    This function initializes the ROS 2 client, sets up the finite state
    machine, adds the states, and starts the action processing.

    Parameters:
        None

    Returns:
        None

    Raises:
        KeyboardInterrupt: If the user interrupts the execution.
    """
    yasmin.YASMIN_LOG_INFO("yasmin_action_client_demo")

    # Initialize ROS 2
    rclpy.init()

    # Set up ROS 2 logs
    set_ros_loggers()

    # Create a finite state machine (FSM)
    sm = StateMachine(outcomes=["outcome4"])

    # Add states to the FSM
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

    # Publish FSM information
    YasminViewerPub("YASMIN_ACTION_CLIENT_DEMO", sm)

    # Create an initial blackboard with the input value
    blackboard = Blackboard()
    blackboard["n"] = 10  # Set the Fibonacci order to 10

    # Execute the FSM
    try:
        outcome = sm(blackboard)
        yasmin.YASMIN_LOG_INFO(outcome)
    except KeyboardInterrupt:
        if sm.is_running():
            sm.cancel_state()  # Cancel the state if interrupted

    # Shutdown ROS 2
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
```

</details>

#### Monitor Demo (FSM + ROS 2 Subscriber)

```shell
ros2 run yasmin_demos monitor_demo.py
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
from yasmin_ros import set_ros_loggers
from yasmin_ros.basic_outcomes import TIMEOUT
from yasmin_viewer import YasminViewerPub


class PrintOdometryState(MonitorState):
    """
    MonitorState subclass to handle Odometry messages.

    This state monitors Odometry messages from the specified ROS topic,
    logging them and transitioning based on the number of messages received.

    Attributes:
        times (int): The number of messages to monitor before transitioning
                     to the next outcome.

    Parameters:
        times (int): The initial count of how many Odometry messages to
                     process before changing state.

    Methods:
        monitor_handler(blackboard: Blackboard, msg: Odometry) -> str:
            Handles incoming Odometry messages, logging the message and
            returning the appropriate outcome based on the remaining count.
    """

    def __init__(self, times: int) -> None:
        """
        Initializes the PrintOdometryState.

        Parameters:
            times (int): The number of Odometry messages to monitor before
                         transitioning to the next outcome.
        """
        super().__init__(
            Odometry,  # msg type
            "odom",  # topic name
            ["outcome1", "outcome2"],  # outcomes
            self.monitor_handler,  # monitor handler callback
            qos=qos_profile_sensor_data,  # qos for the topic subscription
            msg_queue=10,  # queue for the monitor handler callback
            timeout=10,  # timeout to wait for messages in seconds
        )
        self.times = times

    def monitor_handler(self, blackboard: Blackboard, msg: Odometry) -> str:
        """
        Handles incoming Odometry messages.

        This method is called whenever a new Odometry message is received.
        It logs the message, decrements the count of messages to process,
        and determines the next state outcome.

        Parameters:
            blackboard (Blackboard): The shared data storage for states.
            msg (Odometry): The incoming Odometry message.

        Returns:
            str: The next state outcome, either "outcome1" to continue
                 monitoring or "outcome2" to transition to the next state.

        Exceptions:
            None
        """
        yasmin.YASMIN_LOG_INFO(msg)

        self.times -= 1

        if self.times <= 0:
            return "outcome2"

        return "outcome1"


def main():
    """
    Main function to initialize and run the ROS 2 state machine.

    This function initializes ROS 2, sets up logging, creates a finite state
    machine (FSM), adds states to the FSM, and executes the FSM. It handles
    cleanup and shutdown of ROS 2 gracefully.

    Exceptions:
        KeyboardInterrupt: Caught to allow graceful cancellation of the
                          state machine during execution.
    """
    yasmin.YASMIN_LOG_INFO("yasmin_monitor_demo")

    # Initialize ROS 2
    rclpy.init()

    # Set ROS 2 logs
    set_ros_loggers()

    # Create a finite state machine (FSM)
    sm = StateMachine(outcomes=["outcome4"])

    # Add states to the FSM
    sm.add_state(
        "PRINTING_ODOM",
        PrintOdometryState(5),
        transitions={
            "outcome1": "PRINTING_ODOM",
            "outcome2": "outcome4",
            TIMEOUT: "outcome4",
            CANCEL: "outcome4",
        },
    )

    # Publish FSM information
    YasminViewerPub("YASMIN_MONITOR_DEMO", sm)

    # Execute FSM
    try:
        outcome = sm()
        yasmin.YASMIN_LOG_INFO(outcome)
    except KeyboardInterrupt:
        if sm.is_running():
            sm.cancel_state()

    # Shutdown ROS 2
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
```

</details>

#### Publisher Demo (FSM + ROS 2 Publisher)

```shell
ros2 run yasmin_demos publisher_demo.py
```

<details>
<summary>Click to expand</summary>

```python
#!/usr/bin/env python3

# Copyright (C) 2025 Miguel Ángel González Santamarta
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.

import time
import rclpy
from std_msgs.msg import Int32

import yasmin
from yasmin.cb_state import CbState
from yasmin.state_machine import StateMachine
from yasmin.blackboard import Blackboard

from yasmin_ros.basic_outcomes import SUCCEED
from yasmin_ros import PublisherState
from yasmin_ros.ros_logs import set_ros_loggers

from yasmin_viewer.yasmin_viewer_pub import YasminViewerPub


class PublishIntState(PublisherState):
    """
    PublishIntState is a YASMIN ROS publisher state that sends incrementing integers
    to the 'count' topic using std_msgs.msg.Int32 messages.

    This state increments a counter on the blackboard and publishes it.
    """

    def __init__(self):
        """
        Initializes the PublishIntState with the topic 'count' and a message creation callback.
        """
        super().__init__("count", self.create_int_msg)

    def create_int_msg(self, blackboard: Blackboard) -> Int32:
        """
        Generates a std_msgs.msg.Int32 message with an incremented counter value.

        Parameters:
            blackboard (Blackboard): The shared data store between states.

        Returns:
            Int32: A ROS message containing the updated counter.
        """
        # Get and increment the counter from the blackboard
        counter = blackboard.get("counter", 0)
        counter += 1
        blackboard.set("counter", counter)

        # Log the message creation
        yasmin.YASMIN_LOG_INFO(f"Creating message {counter}")

        # Create and return the message
        msg = Int32()
        msg.data = counter
        return msg


def check_count(blackboard: Blackboard) -> str:
    """
    Checks the current counter against a max threshold to determine state transition.

    Parameters:
        blackboard (Blackboard): The shared data store between states.

    Returns:
        str: The outcome string ('outcome1' or 'outcome2').
    """
    # Simulate processing time
    time.sleep(1)

    # Retrieve the counter and max value from blackboard
    count = blackboard.get("counter", 0)
    max_count = blackboard.get("max_count", 10)

    yasmin.YASMIN_LOG_INFO(f"Checking count: {count}")

    # Determine and return the outcome based on the counter value
    if count >= max_count:
        return "outcome1"
    else:
        return "outcome2"


def main(args=None):
    """
    Main function to initialize ROS 2, configure logging, build the YASMIN state machine,
    and execute it until the max_count is reached.

    Args:
        args (list, optional): Command-line arguments passed to rclpy.init().
    """
    yasmin.YASMIN_LOG_INFO("yasmin_monitor_demo")
    rclpy.init(args=args)

    # Configure YASMIN to use ROS-based logging
    set_ros_loggers()

    # Create the state machine with 'SUCCEED' as the terminal outcome
    sm = StateMachine([SUCCEED])

    # Ensure the state machine cancels on shutdown
    def on_shutdown():
        if sm.is_running():
            sm.cancel_state()

    rclpy.get_default_context().on_shutdown(on_shutdown)

    # Add the publishing state which loops until the condition is met
    sm.add_state(
        "PUBLISHING_INT",
        PublishIntState(),
        {
            SUCCEED: "CHECKING_COUNTS",
        },
    )

    # Add the conditional check state
    sm.add_state(
        "CHECKING_COUNTS",
        CbState(["outcome1", "outcome2"], check_count),
        {
            "outcome1": SUCCEED,
            "outcome2": "PUBLISHING_INT",
        },
    )

    # Launch YASMIN Viewer publisher for state visualization
    YasminViewerPub("YASMIN_PUBLISHER_DEMO", sm)

    # Initialize blackboard with counter values
    blackboard = Blackboard()
    blackboard.set("counter", 0)
    blackboard.set("max_count", 10)

    # Run the state machine and log the outcome
    try:
        outcome = sm(blackboard)
        yasmin.YASMIN_LOG_INFO(outcome)
    except Exception as e:
        yasmin.YASMIN_LOG_INFO(str(e))

    # Shutdown ROS
    rclpy.shutdown()


if __name__ == "__main__":
    main()
```

</details>


#### Parameters Demo (FSM + ROS 2 Parameters)

```shell
ros2 run yasmin_demos parameters_demo.py --ros-args -p max_counter:=5
```

<details>
<summary>Click to expand</summary>

```python
import time
import rclpy

import yasmin
from yasmin import State
from yasmin import Blackboard
from yasmin import StateMachine
from yasmin_ros import set_ros_loggers
from yasmin_ros import GetParametersState
from yasmin_ros.basic_outcomes import SUCCEED, ABORT
from yasmin_viewer import YasminViewerPub


# Define the FooState class, inheriting from the State class
class FooState(State):
    """
    Represents the Foo state in the state machine.

    Attributes:
        counter (int): Counter to track the number of executions of this state.
    """

    def __init__(self) -> None:
        """
        Initializes the FooState instance, setting up the outcomes.

        Outcomes:
            outcome1: Indicates the state should continue to the Bar state.
            outcome2: Indicates the state should finish execution and return.
        """
        super().__init__(["outcome1", "outcome2"])
        self.counter = 0

    def execute(self, blackboard: Blackboard) -> str:
        """
        Executes the logic for the Foo state.

        Args:
            blackboard (Blackboard): The shared data structure for states.

        Returns:
            str: The outcome of the execution, which can be "outcome1" or "outcome2".

        Raises:
            Exception: May raise exceptions related to state execution.
        """
        yasmin.YASMIN_LOG_INFO("Executing state FOO")
        time.sleep(3)  # Simulate work by sleeping

        if self.counter < blackboard["max_counter"]:
            self.counter += 1
            blackboard["foo_str"] = f"{blackboard['counter_str']}: {self.counter}"
            return "outcome1"
        else:
            return "outcome2"


# Define the BarState class, inheriting from the State class
class BarState(State):
    """
    Represents the Bar state in the state machine.
    """

    def __init__(self) -> None:
        """
        Initializes the BarState instance, setting up the outcome.

        Outcomes:
            outcome3: Indicates the state should transition back to the Foo state.
        """
        super().__init__(outcomes=["outcome3"])

    def execute(self, blackboard: Blackboard) -> str:
        """
        Executes the logic for the Bar state.

        Args:
            blackboard (Blackboard): The shared data structure for states.

        Returns:
            str: The outcome of the execution, which will always be "outcome3".

        Raises:
            Exception: May raise exceptions related to state execution.
        """
        yasmin.YASMIN_LOG_INFO("Executing state BAR")
        time.sleep(3)  # Simulate work by sleeping

        yasmin.YASMIN_LOG_INFO(blackboard["foo_str"])
        return "outcome3"


# Main function to initialize and run the state machine
def main():
    """
    The main entry point of the application.

    Initializes the ROS 2 environment, sets up the state machine,
    and handles execution and termination.

    Raises:
        KeyboardInterrupt: If the execution is interrupted by the user.
    """
    yasmin.YASMIN_LOG_INFO("yasmin_parameters_demo")

    # Initialize ROS 2
    rclpy.init()

    # Set ROS 2 loggers
    set_ros_loggers()

    # Create a finite state machine (FSM)
    sm = StateMachine(outcomes=["outcome4"])

    # Add states to the FSM
    sm.add_state(
        "GETTING_PARAMETERS",
        GetParametersState(
            parameters={
                "max_counter": 3,
                "counter_str": "Counter",
            },
        ),
        transitions={
            SUCCEED: "FOO",
            ABORT: "outcome4",
        },
    )

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

    # Publish FSM information for visualization
    YasminViewerPub("YASMIN_PARAMETERS_DEMO", sm)

    # Execute the FSM
    try:
        outcome = sm()
        yasmin.YASMIN_LOG_INFO(outcome)
    except KeyboardInterrupt:
        if sm.is_running():
            sm.cancel_state()

    # Shutdown ROS 2 if it's running
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()

```

</details>


#### Nav2 Demo (Hierarchical FSM + ROS 2 Action)

```shell
ros2 run yasmin_demos nav_demo.py
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
from yasmin_ros import set_ros_loggers
from yasmin_ros.basic_outcomes import SUCCEED, ABORT, CANCEL
from yasmin_viewer import YasminViewerPub

# Constants for state outcomes
HAS_NEXT = "has_next"  ##< Indicates there are more waypoints
END = "end"  ##< Indicates no more waypoints


class Nav2State(ActionState):
    """
    ActionState for navigating to a specified pose using ROS 2 Navigation.

    Attributes:
        None

    Methods:
        create_goal_handler(blackboard: Blackboard) -> NavigateToPose.Goal:
            Creates the navigation goal from the blackboard.
    """

    def __init__(self) -> None:
        """
        Initializes the Nav2State.

        Calls the parent constructor to set up the action with:
        - Action type: NavigateToPose
        - Action name: /navigate_to_pose
        - Callback for goal creation: create_goal_handler
        - Outcomes: None, since it will use default outcomes (SUCCEED, ABORT, CANCEL)
        """
        super().__init__(
            NavigateToPose,  # action type
            "/navigate_to_pose",  # action name
            self.create_goal_handler,  # callback to create the goal
            None,  # outcomes
            None,  # callback to process the response
        )

    def create_goal_handler(self, blackboard: Blackboard) -> NavigateToPose.Goal:
        """
        Creates a goal for navigation based on the current pose in the blackboard.

        Args:
            blackboard (Blackboard): The blackboard instance holding current state data.

        Returns:
            NavigateToPose.Goal: The constructed goal for the navigation action.
        """
        goal = NavigateToPose.Goal()
        goal.pose.pose = blackboard["pose"]
        goal.pose.header.frame_id = "map"  # Set the reference frame to 'map'
        return goal


def create_waypoints(blackboard: Blackboard) -> str:
    """
    Initializes waypoints in the blackboard for navigation.

    Args:
        blackboard (Blackboard): The blackboard instance to store waypoints.

    Returns:
        str: Outcome indicating success (SUCCEED).
    """
    blackboard["waypoints"] = {
        "entrance": [1.25, 6.30, -0.78, 0.67],
        "bathroom": [4.89, 1.64, 0.0, 1.0],
        "livingroom": [1.55, 4.03, -0.69, 0.72],
        "kitchen": [3.79, 6.77, 0.99, 0.12],
        "bedroom": [7.50, 4.89, 0.76, 0.65],
    }
    return SUCCEED


def take_random_waypoint(blackboard: Blackboard) -> str:
    """
    Selects a random set of waypoints from the available waypoints.

    Args:
        blackboard (Blackboard): The blackboard instance to store random waypoints.

    Returns:
        str: Outcome indicating success (SUCCEED).
    """
    blackboard["random_waypoints"] = random.sample(
        list(blackboard["waypoints"].keys()), blackboard["waypoints_num"]
    )
    return SUCCEED


def get_next_waypoint(blackboard: Blackboard) -> str:
    """
    Retrieves the next waypoint from the list of random waypoints.

    Updates the blackboard with the pose of the next waypoint.

    Args:
        blackboard (Blackboard): The blackboard instance holding current state data.

    Returns:
        str: Outcome indicating whether there is a next waypoint (HAS_NEXT) or if
             navigation is complete (END).
    """
    if not blackboard["random_waypoints"]:
        return END

    wp_name = blackboard["random_waypoints"].pop(0)  # Get the next waypoint name
    wp = blackboard["waypoints"][wp_name]  # Get the waypoint coordinates

    pose = Pose()
    pose.position.x = wp[0]
    pose.position.y = wp[1]
    pose.orientation.z = wp[2]
    pose.orientation.w = wp[3]

    blackboard["pose"] = pose  # Update blackboard with new pose
    blackboard["text"] = f"I have reached waypoint {wp_name}"

    return HAS_NEXT


# main function
def main() -> None:
    """
    Initializes the ROS 2 node, sets up state machines for navigation, and executes the FSM.

    Handles cleanup and shutdown of the ROS 2 node upon completion.
    """
    yasmin.YASMIN_LOG_INFO("yasmin_nav2_demo")

    # Initialize ROS 2
    rclpy.init()

    # Set ROS 2 loggers for debugging
    set_ros_loggers()

    # Create state machines
    sm = StateMachine(outcomes=[SUCCEED, ABORT, CANCEL])
    nav_sm = StateMachine(outcomes=[SUCCEED, ABORT, CANCEL])

    # Add states to the state machine
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

    sm.add_state(
        "NAVIGATING",
        nav_sm,
        transitions={
            SUCCEED: SUCCEED,
            CANCEL: CANCEL,
            ABORT: ABORT,
        },
    )

    # Publish FSM information for visualization
    YasminViewerPub("YASMIN_NAV2_DEMO", sm)

    # Execute the state machine
    blackboard = Blackboard()
    blackboard["waypoints_num"] = 2  # Set the number of waypoints to navigate

    try:
        outcome = sm(blackboard)  # Run the state machine with the blackboard
        yasmin.YASMIN_LOG_INFO(outcome)
    except KeyboardInterrupt:
        sm.cancel_state()  # Handle manual interruption

    # Shutdown ROS 2
    if rclpy.ok():
        if sm.is_running():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
```

</details>

### Cpp

#### Vanilla Demo

```shell
ros2 run yasmin_demos yasmin_demo
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

/**
 * @brief Represents the "Foo" state in the state machine.
 *
 * This state increments a counter each time it is executed and
 * communicates the current count via the blackboard.
 */
class FooState : public yasmin::State {
public:
  /// Counter to track the number of executions.
  int counter;

  /**
   * @brief Constructs a FooState object, initializing the counter.
   */
  FooState() : yasmin::State({"outcome1", "outcome2"}), counter(0){};

  /**
   * @brief Executes the Foo state logic.
   *
   * This method logs the execution, waits for 3 seconds,
   * increments the counter, and sets a string in the blackboard.
   * The state will transition to either "outcome1" or "outcome2"
   * based on the current value of the counter.
   *
   * @param blackboard Shared pointer to the blackboard for state communication.
   * @return std::string The outcome of the execution: "outcome1" or "outcome2".
   */
  std::string
  execute(std::shared_ptr<yasmin::blackboard::Blackboard> blackboard) override {
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
  };
};

/**
 * @brief Represents the "Bar" state in the state machine.
 *
 * This state logs the value from the blackboard and provides
 * a single outcome to transition.
 */
class BarState : public yasmin::State {
public:
  /**
   * @brief Constructs a BarState object.
   */
  BarState() : yasmin::State({"outcome3"}) {}

  /**
   * @brief Executes the Bar state logic.
   *
   * This method logs the execution, waits for 3 seconds,
   * retrieves a string from the blackboard, and logs it.
   *
   * @param blackboard Shared pointer to the blackboard for state communication.
   * @return std::string The outcome of the execution: "outcome3".
   */
  std::string
  execute(std::shared_ptr<yasmin::blackboard::Blackboard> blackboard) override {
    YASMIN_LOG_INFO("Executing state BAR");
    std::this_thread::sleep_for(std::chrono::seconds(3));

    YASMIN_LOG_INFO(blackboard->get<std::string>("foo_str").c_str());

    return "outcome3";
  }
};

/**
 * @brief Main function that initializes the ROS 2 node and state machine.
 *
 * This function sets up the state machine, adds states, and handles
 * the execution flow, including logging and cleanup.
 *
 * @param argc Argument count from the command line.
 * @param argv Argument vector from the command line.
 * @return int Exit status of the program. Returns 0 on success.
 *
 * @throws std::exception If there is an error during state machine execution.
 */
int main(int argc, char *argv[]) {
  YASMIN_LOG_INFO("yasmin_demo");
  rclcpp::init(argc, argv);

  // Set ROS 2 logs
  yasmin_ros::set_ros_loggers();

  // Create a state machine
  auto sm = std::make_shared<yasmin::StateMachine>(
      std::initializer_list<std::string>{"outcome4"});

  // Cancel state machine on ROS 2 shutdown
  rclcpp::on_shutdown([sm]() {
    if (sm->is_running()) {
      sm->cancel_state();
    }
  });

  // Add states to the state machine
  sm->add_state("FOO", std::make_shared<FooState>(),
                {
                    {"outcome1", "BAR"},
                    {"outcome2", "outcome4"},
                });
  sm->add_state("BAR", std::make_shared<BarState>(),
                {
                    {"outcome3", "FOO"},
                });

  // Publish state machine updates
  yasmin_viewer::YasminViewerPub yasmin_pub("YASMIN_DEMO", sm);

  // Execute the state machine
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

#### Blackboard Remapping Demo

```shell
ros2 run yasmin_demos remap_demo
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
#include "yasmin_ros/basic_outcomes.hpp"
#include "yasmin_ros/ros_logs.hpp"
#include "yasmin_viewer/yasmin_viewer_pub.hpp"

using namespace yasmin;

/**
 * @brief Represents the "Foo" state in the state machine.
 */
class FooState : public yasmin::State {
public:
  /**
   * @brief Constructs a FooState object, initializing the counter.
   */
  FooState() : yasmin::State({yasmin_ros::basic_outcomes::SUCCEED}){};

  /**
   * @brief Executes the Foo state logic.
   *
   * Executes the logic for the Foo state.
   *
   * @param blackboard Shared pointer to the blackboard for state communication.
   * @return std::string The outcome of the execution, which can be SUCCEED.
   */
  std::string
  execute(std::shared_ptr<yasmin::blackboard::Blackboard> blackboard) override {
    std::string data = blackboard->get<std::string>("foo_data");
    YASMIN_LOG_INFO("%s", data.c_str());
    blackboard->set<std::string>("foo_out_data", data);
    return yasmin_ros::basic_outcomes::SUCCEED;
  };
};

/**
 * @brief Represents the "Bar" state in the state machine.
 */
class BarState : public yasmin::State {
public:
  /**
   * @brief Constructs a BarState object.
   */
  BarState() : yasmin::State({yasmin_ros::basic_outcomes::SUCCEED}) {}

  /**
   * @brief Executes the Bar state logic.
   *
   * Executes the logic for the Bar state.
   *
   * @param blackboard Shared pointer to the blackboard for state communication.
   * @return std::string The outcome of the execution: "outcome3".
   */
  std::string
  execute(std::shared_ptr<yasmin::blackboard::Blackboard> blackboard) override {
    std::string datga = blackboard->get<std::string>("bar_data");
    YASMIN_LOG_INFO("%s", datga.c_str());
    return yasmin_ros::basic_outcomes::SUCCEED;
  }
};

/**
 * @brief Main function that initializes the ROS 2 node and state machine.
 *
 * This function sets up the state machine, adds states, and handles
 * the execution flow, including logging and cleanup.
 *
 * @param argc Argument count from the command line.
 * @param argv Argument vector from the command line.
 * @return int Exit status of the program. Returns 0 on success.
 *
 * @throws std::exception If there is an error during state machine execution.
 */
int main(int argc, char *argv[]) {
  YASMIN_LOG_INFO("yasmin_remapping_demo");
  rclcpp::init(argc, argv);

  // Set ROS 2 logs
  yasmin_ros::set_ros_loggers();

  // Create blackboard
  auto blackboard = std::make_shared<yasmin::blackboard::Blackboard>();
  blackboard->set<std::string>("msg1", "test1");
  blackboard->set<std::string>("msg2", "test2");

  // Create a state machine
  auto sm = std::make_shared<yasmin::StateMachine>(
      std::initializer_list<std::string>{yasmin_ros::basic_outcomes::SUCCEED});

  // Cancel state machine on ROS 2 shutdown
  rclcpp::on_shutdown([sm]() {
    if (sm->is_running()) {
      sm->cancel_state();
    }
  });

  // Add states to the state machine
  sm->add_state("STATE1", std::make_shared<FooState>(),
                {
                    {yasmin_ros::basic_outcomes::SUCCEED, "STATE2"},
                },
                {
                    {"foo_data", "msg1"},
                });
  sm->add_state("STATE2", std::make_shared<FooState>(),
                {
                    {yasmin_ros::basic_outcomes::SUCCEED, "STATE3"},
                },
                {
                    {"foo_data", "msg2"},
                });
  sm->add_state("STATE3", std::make_shared<BarState>(),
                {
                    {yasmin_ros::basic_outcomes::SUCCEED,
                     yasmin_ros::basic_outcomes::SUCCEED},
                },
                {
                    {"bar_data", "foo_out_data"},
                });

  // Publish state machine updates
  yasmin_viewer::YasminViewerPub yasmin_pub("YASMIN_REMAPPING_DEMO", sm);

  // Execute the state machine
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

#### Concurrence Demo

```shell
ros2 run yasmin_demos concurrence_demo
```

<details>
<summary>Click to expand</summary>

```cpp
#include <chrono>
#include <iostream>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "yasmin/concurrence.hpp"
#include "yasmin/logs.hpp"
#include "yasmin/state.hpp"
#include "yasmin/state_machine.hpp"
#include "yasmin_ros/ros_logs.hpp"
#include "yasmin_viewer/yasmin_viewer_pub.hpp"

using namespace yasmin;

/**
 * @brief Represents the "Foo" state in the state machine.
 *
 * This state increments a counter each time it is executed and
 * communicates the current count via the blackboard.
 */
class FooState : public yasmin::State {
public:
  /// Counter to track the number of executions.
  int counter;

  /**
   * @brief Constructs a FooState object, initializing the counter.
   */
  FooState()
      : yasmin::State({"outcome1", "outcome2", "outcome3"}), counter(0){};

  /**
   * @brief Executes the Foo state logic.
   *
   * This method logs the execution, waits for 3 seconds,
   * increments the counter, and sets a string in the blackboard.
   * The state will transition to either "outcome1" or "outcome2"
   * based on the current value of the counter.
   *
   * @param blackboard Shared pointer to the blackboard for state communication.
   * @return std::string The outcome of the execution: "outcome1" or "outcome2".
   */
  std::string
  execute(std::shared_ptr<yasmin::blackboard::Blackboard> blackboard) override {
    YASMIN_LOG_INFO("Executing state FOO");
    std::this_thread::sleep_for(std::chrono::seconds(2));

    std::string outcome;

    blackboard->set<std::string>("foo_str",
                                 "Counter: " + std::to_string(this->counter));

    if (this->counter < 3) {
      outcome = "outcome1";
    } else if (this->counter < 5) {
      outcome = "outcome2";
    } else {
      outcome = "outcome3";
    }

    YASMIN_LOG_INFO("Finishing state FOO");
    this->counter += 1;
    return outcome;
  };
};

/**
 * @brief Represents the "Bar" state in the state machine.
 *
 * This state logs the value from the blackboard and provides
 * a single outcome to transition.
 */
class BarState : public yasmin::State {
public:
  /**
   * @brief Constructs a BarState object.
   */
  BarState() : yasmin::State({"outcome3"}) {}

  /**
   * @brief Executes the Bar state logic.
   *
   * This method logs the execution, waits for 3 seconds,
   * retrieves a string from the blackboard, and logs it.
   *
   * @param blackboard Shared pointer to the blackboard for state communication.
   * @return std::string The outcome of the execution: "outcome3".
   */
  std::string
  execute(std::shared_ptr<yasmin::blackboard::Blackboard> blackboard) override {
    YASMIN_LOG_INFO("Executing state BAR");
    std::this_thread::sleep_for(std::chrono::seconds(4));

    if (blackboard->contains("foo_str")) {
      YASMIN_LOG_INFO(blackboard->get<std::string>("foo_str").c_str());
    } else {
      YASMIN_LOG_INFO("blackboard does not yet contains 'foo_str'");
    }

    YASMIN_LOG_INFO("Finishing state BAR");

    return "outcome3";
  }
};

/**
 * @brief Main function that initializes the ROS 2 node and state machine.
 *
 * This function sets up the state machine, adds states, and handles
 * the execution flow, including logging and cleanup.
 *
 * @param argc Argument count from the command line.
 * @param argv Argument vector from the command line.
 * @return int Exit status of the program. Returns 0 on success.
 *
 * @throws std::exception If there is an error during state machine execution.
 */
int main(int argc, char *argv[]) {
  YASMIN_LOG_INFO("yasmin_concurrence_demo");
  rclcpp::init(argc, argv);

  // Set ROS 2 logs
  yasmin_ros::set_ros_loggers();

  // Create a state machine
  auto sm = std::make_shared<yasmin::StateMachine>(
      std::initializer_list<std::string>{"outcome4"});

  // Cancel state machine on ROS 2 shutdown
  rclcpp::on_shutdown([sm]() {
    if (sm->is_running()) {
      sm->cancel_state();
    }
  });

  // Create states to run concurrently
  auto foo_state = std::make_shared<FooState>();
  auto bar_state = std::make_shared<BarState>();

  // Create concurrent state
  auto concurrent_state = std::make_shared<Concurrence>(
      std::set<std::shared_ptr<State>>{foo_state, bar_state}, "defaulted",
      Concurrence::OutcomeMap{
          {"outcome1", Concurrence::StateMap{{foo_state, "outcome1"},
                                             {bar_state, "outcome3"}}},
          {"outcome2", Concurrence::StateMap{{foo_state, "outcome2"},
                                             {bar_state, "outcome3"}}}});

  // Add concurrent state to the state machine
  sm->add_state("CONCURRENCE", concurrent_state,
                {
                    {"outcome1", "CONCURRENCE"},
                    {"outcome2", "CONCURRENCE"},
                    {"defaulted", "outcome4"},
                });

  // Publish state machine updates
  yasmin_viewer::YasminViewerPub yasmin_pub("YASMIN_CONCURRENCE_DEMO", sm);

  // Execute the state machine
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
ros2 run yasmin_demos add_two_ints_server
```

```shell
ros2 run yasmin_demos service_client_demo
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

/**
 * @brief Sets two integer values in the blackboard.
 *
 * Sets the integers "a" and "b" in the blackboard with values 10 and 5,
 * respectively.
 *
 * @param blackboard Shared pointer to the blackboard for setting values.
 * @return std::string Outcome indicating success or failure.
 */
std::string
set_ints(std::shared_ptr<yasmin::blackboard::Blackboard> blackboard) {
  blackboard->set<int>("a", 10);
  blackboard->set<int>("b", 5);
  return yasmin_ros::basic_outcomes::SUCCEED;
}

/**
 * @brief Prints the sum stored in the blackboard.
 *
 * Retrieves the integer "sum" from the blackboard and prints it.
 *
 * @param blackboard Shared pointer to the blackboard for getting values.
 * @return std::string Outcome indicating success.
 */
std::string
print_sum(std::shared_ptr<yasmin::blackboard::Blackboard> blackboard) {
  fprintf(stderr, "Sum: %d\n", blackboard->get<int>("sum"));
  return yasmin_ros::basic_outcomes::SUCCEED;
}

/**
 * @class AddTwoIntsState
 * @brief State for calling the AddTwoInts service in ROS 2.
 *
 * This state constructs and sends a service request to add two integers, and
 * processes the response to retrieve and store the result in the blackboard.
 */
class AddTwoIntsState
    : public yasmin_ros::ServiceState<example_interfaces::srv::AddTwoInts> {
public:
  /**
   * @brief Constructor for AddTwoIntsState.
   *
   * Initializes the service state with the specified service name and handlers
   * for request creation and response processing.
   */
  AddTwoIntsState()
      : yasmin_ros::ServiceState<example_interfaces::srv::AddTwoInts>(
            "/add_two_ints",
            std::bind(&AddTwoIntsState::create_request_handler, this, _1),
            {"outcome1"},
            std::bind(&AddTwoIntsState::response_handler, this, _1, _2)){};

  /**
   * @brief Creates a service request using values from the blackboard.
   *
   * Retrieves integers "a" and "b" from the blackboard and sets them in the
   * request.
   *
   * @param blackboard Shared pointer to the blackboard for retrieving values.
   * @return example_interfaces::srv::AddTwoInts::Request::SharedPtr The service
   * request.
   */
  example_interfaces::srv::AddTwoInts::Request::SharedPtr
  create_request_handler(
      std::shared_ptr<yasmin::blackboard::Blackboard> blackboard) {

    auto request =
        std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
    request->a = blackboard->get<int>("a");
    request->b = blackboard->get<int>("b");
    return request;
  };

  /**
   * @brief Handles the service response and stores the result in the
   * blackboard.
   *
   * Retrieves the sum from the service response and stores it in the
   * blackboard.
   *
   * @param blackboard Shared pointer to the blackboard for storing values.
   * @param response Shared pointer to the service response containing the sum.
   * @return std::string Outcome indicating success.
   */
  std::string response_handler(
      std::shared_ptr<yasmin::blackboard::Blackboard> blackboard,
      example_interfaces::srv::AddTwoInts::Response::SharedPtr response) {

    blackboard->set<int>("sum", response->sum);
    return "outcome1";
  };
};

/**
 * @brief Main function to initialize and run the state machine.
 *
 * Sets up logging, initializes ROS 2, and defines a state machine with three
 * states.
 *
 * @param argc Argument count.
 * @param argv Argument vector.
 * @return int Exit code indicating success or failure.
 */
int main(int argc, char *argv[]) {
  YASMIN_LOG_INFO("yasmin_service_client_demo");
  rclcpp::init(argc, argv);

  // Set up ROS 2 logging.
  yasmin_ros::set_ros_loggers();

  // Create a state machine with a specified outcome.
  auto sm = std::make_shared<yasmin::StateMachine>(
      std::initializer_list<std::string>{"outcome4"});

  // Cancel the state machine on ROS 2 shutdown.
  rclcpp::on_shutdown([sm]() {
    if (sm->is_running()) {
      sm->cancel_state();
    }
  });

  // Add states to the state machine.
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

  // Publish state machine visualization.
  yasmin_viewer::YasminViewerPub yasmin_pub("YASMIN_SERVICE_CLIENT_DEMO", sm);

  // Execute the state machine.
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
ros2 run yasmin_demos fibonacci_action_server
```

```shell
ros2 run yasmin_demos action_client_demo
```

<details>
<summary>Click to expand</summary>

```cpp
#include <iostream>
#include <memory>
#include <string>

#include "example_interfaces/action/fibonacci.hpp"

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
using Fibonacci = example_interfaces::action::Fibonacci;
using namespace yasmin;

/**
 * @brief Prints the result of the Fibonacci action.
 *
 * Retrieves the final Fibonacci sequence from the blackboard and outputs it to
 * stderr.
 *
 * @param blackboard Shared pointer to the blackboard storing the Fibonacci
 * sequence.
 * @return The outcome status indicating success.
 */
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

/**
 * @class FibonacciState
 * @brief Represents the action state for the Fibonacci action.
 *
 * This class manages goal creation, response handling, and feedback processing
 * for the Fibonacci action.
 */
class FibonacciState : public yasmin_ros::ActionState<Fibonacci> {

public:
  /**
   * @brief Constructs a new FibonacciState object and initializes callbacks.
   */
  FibonacciState()
      : yasmin_ros::ActionState<Fibonacci>(
            "/fibonacci",
            std::bind(&FibonacciState::create_goal_handler, this, _1),
            std::bind(&FibonacciState::response_handler, this, _1, _2),
            std::bind(&FibonacciState::print_feedback, this, _1, _2)){};

  /**
   * @brief Callback for creating the Fibonacci action goal.
   *
   * Reads the order of the Fibonacci sequence from the blackboard.
   *
   * @param blackboard Shared pointer to the blackboard.
   * @return The Fibonacci goal with the specified order.
   */
  Fibonacci::Goal create_goal_handler(
      std::shared_ptr<yasmin::blackboard::Blackboard> blackboard) {

    auto goal = Fibonacci::Goal();
    goal.order = blackboard->get<int>("n");
    return goal;
  };

  /**
   * @brief Callback for handling the action response.
   *
   * Stores the resulting Fibonacci sequence in the blackboard.
   *
   * @param blackboard Shared pointer to the blackboard.
   * @param response Shared pointer to the action result containing the
   * sequence.
   * @return The outcome status indicating success.
   */
  std::string
  response_handler(std::shared_ptr<yasmin::blackboard::Blackboard> blackboard,
                   Fibonacci::Result::SharedPtr response) {

    blackboard->set<std::vector<int>>("sum", response->sequence);
    return yasmin_ros::basic_outcomes::SUCCEED;
  };

  /**
   * @brief Callback for printing action feedback.
   *
   * Displays each new partial Fibonacci sequence number as it is received.
   *
   * @param blackboard Shared pointer to the blackboard (not used in this
   * method).
   * @param feedback Shared pointer to the feedback message with partial
   * sequence.
   */
  void
  print_feedback(std::shared_ptr<yasmin::blackboard::Blackboard> blackboard,
                 std::shared_ptr<const Fibonacci::Feedback> feedback) {
    (void)blackboard;

    std::stringstream ss;
    ss << "Next number in sequence received: ";
    for (auto number : feedback->sequence) {
      ss << number << " ";
    }

    fprintf(stderr, "%s\n", ss.str().c_str());
  };
};

/**
 * @brief Main function for the Fibonacci action client.
 *
 * Initializes ROS 2, sets up logging, creates a state machine to manage action
 * states, and executes the Fibonacci action.
 *
 * @param argc Argument count.
 * @param argv Argument values.
 * @return Execution status code.
 * @exception std::exception if there is an error during execution.
 */
int main(int argc, char *argv[]) {

  YASMIN_LOG_INFO("yasmin_action_client_demo");
  rclcpp::init(argc, argv);

  // Set ROS 2 logging
  yasmin_ros::set_ros_loggers();

  // Create the state machine
  auto sm = std::make_shared<yasmin::StateMachine>(
      std::initializer_list<std::string>{"outcome4"});

  // Cancel state machine on ROS 2 shutdown
  rclcpp::on_shutdown([sm]() {
    if (sm->is_running()) {
      sm->cancel_state();
    }
  });

  // Add states to the state machine
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

  // Publisher for visualizing the state machine
  yasmin_viewer::YasminViewerPub yasmin_pub("YASMIN_ACTION_CLIENT_DEMO", sm);

  // Create an initial blackboard and set the Fibonacci order
  std::shared_ptr<yasmin::blackboard::Blackboard> blackboard =
      std::make_shared<yasmin::blackboard::Blackboard>();
  blackboard->set<int>("n", 10);

  // Execute the state machine
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
ros2 run yasmin_demos monitor_demo
```

<details>
<summary>Click to expand</summary>

```cpp
#include <iostream>
#include <memory>
#include <string>

#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

#include "yasmin/logs.hpp"
#include "yasmin/state_machine.hpp"
#include "yasmin_ros/basic_outcomes.hpp"
#include "yasmin_ros/monitor_state.hpp"
#include "yasmin_ros/ros_logs.hpp"
#include "yasmin_viewer/yasmin_viewer_pub.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
using namespace yasmin;

/**
 * @class PrintOdometryState
 * @brief A state that monitors odometry data and transitions based on a
 * specified count.
 *
 * This class inherits from yasmin_ros::MonitorState and listens to the "odom"
 * topic for nav_msgs::msg::Odometry messages. The state transitions once a
 * specified number of messages has been received and processed.
 */
class PrintOdometryState
    : public yasmin_ros::MonitorState<nav_msgs::msg::Odometry> {

public:
  /// The number of times the state will process messages
  int times;

  /**
   * @brief Constructor for the PrintOdometryState class.
   * @param times Number of times to print odometry data before transitioning.
   */
  PrintOdometryState(int times)
      : yasmin_ros::MonitorState<nav_msgs::msg::Odometry>(
            "odom",                   // topic name
            {"outcome1", "outcome2"}, // possible outcomes
            std::bind(&PrintOdometryState::monitor_handler, this, _1,
                      _2), // monitor handler callback
            10,            // QoS for the topic subscription
            10,            // queue size for the callback
            10             // timeout for receiving messages
        ) {
    this->times = times;
  };

  /**
   * @brief Handler for processing odometry data.
   *
   * This function logs the x, y, and z positions from the odometry message.
   * After processing, it decreases the `times` counter. When the counter
   * reaches zero, the state transitions to "outcome2"; otherwise, it remains in
   * "outcome1".
   *
   * @param blackboard Shared pointer to the blackboard (unused in this
   * implementation).
   * @param msg Shared pointer to the received odometry message.
   * @return A string representing the outcome: "outcome1" to stay in the state,
   *         or "outcome2" to transition out of the state.
   */
  std::string
  monitor_handler(std::shared_ptr<yasmin::blackboard::Blackboard> blackboard,
                  std::shared_ptr<nav_msgs::msg::Odometry> msg) {

    (void)blackboard; // blackboard is not used in this implementation

    YASMIN_LOG_INFO("x: %f", msg->pose.pose.position.x);
    YASMIN_LOG_INFO("y: %f", msg->pose.pose.position.y);
    YASMIN_LOG_INFO("z: %f", msg->pose.pose.position.z);

    this->times--;

    // Transition based on remaining times
    if (this->times <= 0) {
      return "outcome2";
    }

    return "outcome1";
  };
};

/**
 * @brief Main function initializing ROS 2 and setting up the state machine.
 *
 * Initializes ROS 2, configures loggers, sets up the state machine with states
 * and transitions, and starts monitoring odometry data. The state machine will
 * cancel upon ROS 2 shutdown.
 *
 * @param argc Argument count.
 * @param argv Argument vector.
 * @return int Exit code.
 *
 * @exception std::exception Catches and logs any exceptions thrown by the state
 * machine.
 */
int main(int argc, char *argv[]) {

  YASMIN_LOG_INFO("yasmin_monitor_demo");
  rclcpp::init(argc, argv);

  // Set up ROS 2 loggers
  yasmin_ros::set_ros_loggers();

  // Create a state machine with a final outcome
  auto sm = std::make_shared<yasmin::StateMachine>(
      std::initializer_list<std::string>{"outcome4"});

  // Cancel state machine on ROS 2 shutdown
  rclcpp::on_shutdown([sm]() {
    if (sm->is_running()) {
      sm->cancel_state();
    }
  });

  // Add states to the state machine
  sm->add_state(
      "PRINTING_ODOM", std::make_shared<PrintOdometryState>(5),
      {
          {"outcome1",
           "PRINTING_ODOM"},        // Transition back to itself on outcome1
          {"outcome2", "outcome4"}, // Transition to outcome4 on outcome2
          {yasmin_ros::basic_outcomes::TIMEOUT,
           "outcome4"}, // Timeout transition
      });

  // Publisher for visualizing the state machine's status
  yasmin_viewer::YasminViewerPub yasmin_pub("YASMIN_MONITOR_DEMO", sm);

  // Execute the state machine
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

#### Publisher Demo (FSM + ROS 2 Publisher)

```shell
ros2 run yasmin_demos publisher_demo
```

<details>
<summary>Click to expand</summary>

```cpp
#include <iostream>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

#include "yasmin/cb_state.hpp"
#include "yasmin/logs.hpp"
#include "yasmin/state_machine.hpp"
#include "yasmin_ros/basic_outcomes.hpp"
#include "yasmin_ros/publisher_state.hpp"
#include "yasmin_ros/ros_logs.hpp"
#include "yasmin_viewer/yasmin_viewer_pub.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
using namespace yasmin;

/**
 * @class PublishIntState
 * @brief A state that publishes ints.
 *
 * This class inherits from yasmin_ros::PublisherState and publish ints
 * to the "count" topic.
 */
class PublishIntState
    : public yasmin_ros::PublisherState<std_msgs::msg::Int32> {

public:
  /**
   * @brief Constructor for the PublishIntState class.
   */
  PublishIntState()
      : yasmin_ros::PublisherState<std_msgs::msg::Int32>(
            "count", // topic name
            std::bind(&PublishIntState::create_int_msg, this,
                      _1) // create msg handler callback
        ){};

  /**
   * @brief Create a new Int message.
   *
   *
   * @param blackboard Shared pointer to the blackboard (unused in this
   * implementation).
   * @return A new Int message.
   */
  std_msgs::msg::Int32
  create_int_msg(std::shared_ptr<yasmin::blackboard::Blackboard> blackboard) {

    int counter = blackboard->get<int>("counter");
    counter++;
    blackboard->set<int>("counter", counter);

    YASMIN_LOG_INFO("Creating message %d", counter);
    std_msgs::msg::Int32 msg;
    msg.data = counter;
    return msg;
  };
};

/**
 * @brief Check the count in the blackboard and return an outcome based on it.
 *
 * This function checks the value of "counter" in the blackboard and compares it
 * with "max_count". If "counter" exceeds "max_count", it returns "outcome1",
 * otherwise it returns "outcome2".
 *
 * @param blackboard Shared pointer to the blackboard.
 * @return A string representing the outcome.
 */
std::string
check_count(std::shared_ptr<yasmin::blackboard::Blackboard> blackboard) {

  // Sleep for 1 second to simulate some processing time
  rclcpp::sleep_for(std::chrono::seconds(1));
  YASMIN_LOG_INFO("Checking count: %d", blackboard->get<int>("counter"));

  if (blackboard->get<int>("counter") >= blackboard->get<int>("max_count")) {
    return "outcome1";
  } else {
    return "outcome2";
  }
}

/**
 * @brief Main function initializing ROS 2 and setting up the state machine.
 *
 * Initializes ROS 2, configures loggers, sets up the state machine with states
 * and transitions, and starts monitoring odometry data. The state machine will
 * cancel upon ROS 2 shutdown.
 *
 * @param argc Argument count.
 * @param argv Argument vector.
 * @return int Exit code.
 *
 * @exception std::exception Catches and logs any exceptions thrown by the state
 * machine.
 */
int main(int argc, char *argv[]) {

  YASMIN_LOG_INFO("yasmin_publisher_demo");
  rclcpp::init(argc, argv);

  // Set up ROS 2 loggers
  yasmin_ros::set_ros_loggers();

  // Create a state machine with a final outcome
  auto sm = std::make_shared<yasmin::StateMachine>(
      std::initializer_list<std::string>{yasmin_ros::basic_outcomes::SUCCEED});

  // Cancel state machine on ROS 2 shutdown
  rclcpp::on_shutdown([sm]() {
    if (sm->is_running()) {
      sm->cancel_state();
    }
  });

  // Add states to the state machine
  sm->add_state("PUBLISHING_INT", std::make_shared<PublishIntState>(),
                {
                    {yasmin_ros::basic_outcomes::SUCCEED,
                     "CHECKINNG_COUNTS"}, // Transition back to itself
                });
  sm->add_state("CHECKINNG_COUNTS",
                std::make_shared<yasmin::CbState>(
                    std::initializer_list<std::string>{"outcome1", "outcome2"},
                    check_count),
                {{"outcome1", yasmin_ros::basic_outcomes::SUCCEED},
                 {"outcome2", "PUBLISHING_INT"}});

  // Publisher for visualizing the state machine's status
  yasmin_viewer::YasminViewerPub yasmin_pub("YASMIN_PUBLISHER_DEMO", sm);

  // Execute the state machine
  std::shared_ptr<yasmin::blackboard::Blackboard> blackboard =
      std::make_shared<yasmin::blackboard::Blackboard>();
  blackboard->set<int>("counter", 0);
  blackboard->set<int>("max_count", 10);
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

#### Parameters Demo (FSM + ROS 2 Parameters)

```shell
ros2 run yasmin_demos parameters_demo --ros-args -p max_counter:=5
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
#include "yasmin_ros/basic_outcomes.hpp"
#include "yasmin_ros/get_parameters_state.hpp"
#include "yasmin_ros/ros_logs.hpp"
#include "yasmin_viewer/yasmin_viewer_pub.hpp"

using namespace yasmin;

/**
 * @brief Represents the "Foo" state in the state machine.
 *
 * This state increments a counter each time it is executed and
 * communicates the current count via the blackboard.
 */
class FooState : public yasmin::State {
public:
  /// Counter to track the number of executions.
  int counter;

  /**
   * @brief Constructs a FooState object, initializing the counter.
   */
  FooState() : yasmin::State({"outcome1", "outcome2"}), counter(0){};

  /**
   * @brief Executes the Foo state logic.
   *
   * This method logs the execution, waits for 3 seconds,
   * increments the counter, and sets a string in the blackboard.
   * The state will transition to either "outcome1" or "outcome2"
   * based on the current value of the counter.
   *
   * @param blackboard Shared pointer to the blackboard for state communication.
   * @return std::string The outcome of the execution: "outcome1" or "outcome2".
   */
  std::string
  execute(std::shared_ptr<yasmin::blackboard::Blackboard> blackboard) override {
    YASMIN_LOG_INFO("Executing state FOO");
    std::this_thread::sleep_for(std::chrono::seconds(3));

    if (this->counter < blackboard->get<int>("max_counter")) {
      this->counter += 1;
      blackboard->set<std::string>("foo_str",
                                   blackboard->get<std::string>("counter_str") +
                                       ": " + std::to_string(this->counter));
      return "outcome1";

    } else {
      return "outcome2";
    }
  };
};

/**
 * @brief Represents the "Bar" state in the state machine.
 *
 * This state logs the value from the blackboard and provides
 * a single outcome to transition.
 */
class BarState : public yasmin::State {
public:
  /**
   * @brief Constructs a BarState object.
   */
  BarState() : yasmin::State({"outcome3"}) {}

  /**
   * @brief Executes the Bar state logic.
   *
   * This method logs the execution, waits for 3 seconds,
   * retrieves a string from the blackboard, and logs it.
   *
   * @param blackboard Shared pointer to the blackboard for state communication.
   * @return std::string The outcome of the execution: "outcome3".
   */
  std::string
  execute(std::shared_ptr<yasmin::blackboard::Blackboard> blackboard) override {
    YASMIN_LOG_INFO("Executing state BAR");
    std::this_thread::sleep_for(std::chrono::seconds(3));

    YASMIN_LOG_INFO(blackboard->get<std::string>("foo_str").c_str());

    return "outcome3";
  }
};

/**
 * @brief Main function that initializes the ROS 2 node and state machine.
 *
 * This function sets up the state machine, adds states, and handles
 * the execution flow, including logging and cleanup.
 *
 * @param argc Argument count from the command line.
 * @param argv Argument vector from the command line.
 * @return int Exit status of the program. Returns 0 on success.
 *
 * @throws std::exception If there is an error during state machine execution.
 */
int main(int argc, char *argv[]) {
  YASMIN_LOG_INFO("yasmin_parameters_demo");
  rclcpp::init(argc, argv);

  // Set ROS 2 logs
  yasmin_ros::set_ros_loggers();

  // Create a state machine
  auto sm = std::make_shared<yasmin::StateMachine>(
      std::initializer_list<std::string>{"outcome4"});

  // Cancel state machine on ROS 2 shutdown
  rclcpp::on_shutdown([sm]() {
    if (sm->is_running()) {
      sm->cancel_state();
    }
  });

  // Add states to the state machine
  sm->add_state("GETTING_PARAMETERS",
                std::make_shared<yasmin_ros::GetParametersState>(
                    std::map<std::string, std::any>{
                        {"max_counter", 3},
                        {"counter_str", std::string("Counter")},
                    }),
                {
                    {yasmin_ros::basic_outcomes::SUCCEED, "FOO"},
                    {yasmin_ros::basic_outcomes::ABORT, "outcome4"},
                });
  sm->add_state("FOO", std::make_shared<FooState>(),
                {
                    {"outcome1", "BAR"},
                    {"outcome2", "outcome4"},
                });
  sm->add_state("BAR", std::make_shared<BarState>(),
                {
                    {"outcome3", "FOO"},
                });

  // Publish state machine updates
  yasmin_viewer::YasminViewerPub yasmin_pub("YASMIN_PARAMETERS_DEMO", sm);

  // Execute the state machine
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

The **YASMIN Viewer** provides a convenient way to monitor **YASMIN**'s Finite State Machines (FSM). It is built using **Flask** and **ReactJS** and includes a filter to focus on a single FSM at a time.

![](./docs/viewer.gif)

### Getting Started

```shell
ros2 run yasmin_viewer yasmin_viewer_node
```

Once started, open http://localhost:5000/ in your browser to view your state machines.

### Custom host and port

You can specify a custom host and port by using the following command:

```shell
ros2 run yasmin_viewer yasmin_viewer_node --ros-args -p host:=127.0.0.1 -p port:=5032
```

After running the command, access your state machines at http://127.0.0.1:5032/.

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
