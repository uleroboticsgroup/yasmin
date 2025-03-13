#!/usr/bin/env python
import time
import rclpy

import yasmin
from yasmin import State
from yasmin import Blackboard
from yasmin import StateMachine
from yasmin_ros import set_ros_loggers
from yasmin_viewer import YasminViewerPub
from yasmin_ros.basic_outcomes import SUCCEED, CANCEL, ABORT

class Foo(State):
    def __init__(self):
        super().__init__(outcomes=[SUCCEED, CANCEL, ABORT])
    
    def execute(self, blackboard: Blackboard):
        print(blackboard["msg"])


if __name__ == "__main__":
    bb = Blackboard()
    bb["msg1"] = "teste1"
    bb["msg2"] = "teste2"
    sm = StateMachine(outcomes=[SUCCEED,CANCEL,ABORT])
    sm.add_state("STATE1",Foo(),
                 transitions={
                     SUCCEED:"STATE2",
                     ABORT:ABORT,
                     CANCEL:CANCEL
                 },
                 remmapings={
                     "msg":"msg1"
                 })
    sm.add_state("STATE2",Foo(),
                 transitions={
                     SUCCEED:SUCCEED,
                     ABORT:ABORT,
                     CANCEL:CANCEL
                 },
                 remmapings={
                     "msg":"msg2"
                 })
    sm.execute(bb)