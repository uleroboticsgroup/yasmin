#!/usr/bin/env python3

"""
Example showing how concurrence transitions are handled in YasminViewerPub.

This example demonstrates how the concurrence outcome map is converted into
transition-like information for visualization purposes.
"""

from yasmin import State, Concurrence
from yasmin_viewer import YasminViewerPub


class SimpleState(State):
    def __init__(self, outcomes):
        super().__init__(outcomes)

    def execute(self, blackboard):
        return self.get_outcomes()[0]


def main():
    print("=== Concurrence Transitions Example ===")

    # Create some simple states
    foo_state = SimpleState(["outcome1", "outcome2", "outcome3"])
    bar_state = SimpleState(["outcome3"])

    # Create concurrence with outcome map
    concurrence = Concurrence(
        states={
            "FOO": foo_state,
            "BAR": bar_state,
        },
        default_outcome="defaulted",
        outcome_map={
            "success": {
                "FOO": "outcome1",
                "BAR": "outcome3",
            },
            "continue": {
                "FOO": "outcome2",
                "BAR": "outcome3",
            },
        },
    )

    # Create a YasminViewerPub instance to test the transition parsing
    viewer_pub = YasminViewerPub("test", None)  # Mock instance for testing

    # Test the parse_concurrence_transitions method
    transitions = viewer_pub.parse_concurrence_transitions(concurrence)

    print("Concurrence outcome map:")
    outcome_map = concurrence.get_outcome_map()
    for outcome, requirements in outcome_map.items():
        print(f"  {outcome}: {requirements}")

    print(f"\nDefault outcome: {concurrence._default_outcome}")

    print("\nGenerated transitions for visualization:")
    for transition in transitions:
        print(f"  Outcome: {transition.outcome} -> State: {transition.state}")

    print("\nExplanation:")
    print("- Each outcome in the outcome_map becomes a transition")
    print(
        "- The 'state' field shows the requirements (which child states must produce which outcomes)"
    )
    print("- The default outcome gets a simple 'default' state")
    print("- This allows the viewer to understand the concurrence logic visually")


if __name__ == "__main__":
    main()
