int32 id                                    # state ID inside a state machine
int32 parent -1                             # ID of the parent of this state

string name                                 # state name inside the state machine
yasmin_msgs/Transition[] transitions        # transitions for this state
string[] outcomes                           # list of outcomes of this state

bool is_fsm                                 # whether this state is a state machine
int32 current_state -1                      # ID of the current state of this state if it is a state machine
