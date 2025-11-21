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

import unittest
import os
import tempfile
from lxml import etree as ET
from yasmin import Blackboard, StateMachine
from yasmin_factory.yasmin_factory import YasminFactory


class TestYasminFactory(unittest.TestCase):

    def setUp(self):
        """Set up test fixtures."""
        self.factory = YasminFactory()
        self.test_dir = os.path.dirname(os.path.abspath(__file__))

    def test_create_python_state_simple(self):
        """Test creating a simple Python state."""
        state_xml = '<State name="TestState" type="py" module="test.test_simple_state" class="TestSimpleState"/>'
        state_elem = ET.fromstring(state_xml)

        state = self.factory.create_state(state_elem)

        self.assertIsNotNone(state)
        self.assertEqual(len(state.get_outcomes()), 2)
        self.assertIn("outcome1", state.get_outcomes())
        self.assertIn("outcome2", state.get_outcomes())

    def test_create_cpp_state(self):
        """Test creating a C++ state using pybind bridge."""
        state_xml = (
            '<State name="CppState" type="cpp" class="yasmin_factory/TestSimpleState"/>'
        )
        state_elem = ET.fromstring(state_xml)

        state = self.factory.create_state(state_elem)
        self.assertIsNotNone(state)
        self.assertEqual(len(state.get_outcomes()), 2)

    def test_create_state_invalid_type(self):
        """Test creating a state with invalid type."""
        state_xml = '<State name="InvalidState" type="invalid" class="SomeClass"/>'
        state_elem = ET.fromstring(state_xml)

        with self.assertRaises(ValueError) as context:
            self.factory.create_state(state_elem)

        self.assertIn("Unknown state type", str(context.exception))

    def test_create_state_missing_class(self):
        """Test creating a state with missing class attribute."""
        state_xml = '<State name="NoClass" type="py" module="some.module"/>'
        state_elem = ET.fromstring(state_xml)

        with self.assertRaises(KeyError):
            self.factory.create_state(state_elem)

    def test_create_state_missing_module(self):
        """Test creating a Python state with missing module attribute."""
        state_xml = '<State name="NoModule" type="py" class="SomeClass"/>'
        state_elem = ET.fromstring(state_xml)

        with self.assertRaises(KeyError):
            self.factory.create_state(state_elem)

    def test_create_sm_from_xml(self):
        """Test creating a state machine from XML string."""
        sm_xml = """
        <StateMachine outcomes="end">
            <State name="State1" type="py" module="test.test_simple_state" class="TestSimpleState">
                <Transition from="outcome1" to="State1"/>
                <Transition from="outcome2" to="end"/>
            </State>
        </StateMachine>
        """
        root = ET.fromstring(sm_xml)

        sm = self.factory.create_sm(root)

        self.assertIsNotNone(sm)
        self.assertIsInstance(sm, StateMachine)
        self.assertIn("end", sm.get_outcomes())

    def test_create_sm_from_file_cpppy(self):
        """Test creating a state machine from XML file."""
        xml_file = os.path.join(self.test_dir, "test_simple_sm_1.xml")
        sm = self.factory.create_sm_from_file(xml_file)

        self.assertIsNotNone(sm)
        self.assertIsInstance(sm, StateMachine)
        self.assertIn("end", sm.get_outcomes())

    def test_create_sm_from_file_pycpp(self):
        """Test creating a state machine from XML file."""
        xml_file = os.path.join(self.test_dir, "test_simple_sm_2.xml")
        sm = self.factory.create_sm_from_file(xml_file)

        self.assertIsNotNone(sm)
        self.assertIsInstance(sm, StateMachine)
        self.assertIn("end", sm.get_outcomes())

    def test_create_sm_from_invalid_file(self):
        """Test creating a state machine from non-existent file."""
        with self.assertRaises(Exception):
            self.factory.create_sm_from_file("/non/existent/file.xml")

    def test_create_sm_from_invalid_xml(self):
        """Test creating a state machine from invalid XML."""
        with tempfile.NamedTemporaryFile(mode="w", suffix=".xml", delete=False) as f:
            f.write("This is not valid XML")
            temp_file = f.name

        try:
            with self.assertRaises(Exception):
                self.factory.create_sm_from_file(temp_file)
        finally:
            os.unlink(temp_file)

    def test_create_sm_invalid_root_element(self):
        """Test creating a state machine with invalid root element."""
        with tempfile.NamedTemporaryFile(mode="w", suffix=".xml", delete=False) as f:
            f.write('<?xml version="1.0"?><InvalidRoot/>')
            temp_file = f.name

        try:
            with self.assertRaises(ValueError) as context:
                self.factory.create_sm_from_file(temp_file)
            self.assertIn("Root element must be 'StateMachine'", str(context.exception))
        finally:
            os.unlink(temp_file)

    def test_state_execution(self):
        """Test executing a created state."""
        state_xml = '<State name="TestState" type="py" module="test.test_simple_state" class="TestSimpleState"/>'
        state_elem = ET.fromstring(state_xml)

        state = self.factory.create_state(state_elem)
        blackboard = Blackboard()

        # Execute state multiple times
        outcome1 = state.execute(blackboard)
        self.assertEqual(outcome1, "outcome1")
        self.assertEqual(blackboard["counter"], 1)

        outcome2 = state.execute(blackboard)
        self.assertEqual(outcome2, "outcome1")
        self.assertEqual(blackboard["counter"], 2)

        outcome3 = state.execute(blackboard)
        self.assertEqual(outcome3, "outcome2")
        self.assertEqual(blackboard["counter"], 3)

    def test_nested_state_machine(self):
        """Test creating a nested state machine."""
        xml_file = os.path.join(self.test_dir, "test_nested_sm.xml")
        sm = self.factory.create_sm_from_file(xml_file)

        self.assertIsNotNone(sm)
        self.assertIsInstance(sm, StateMachine)
        self.assertIn("final_outcome", sm.get_outcomes())

    def test_remapping(self):
        """Test remapping across multiple states that pass data through the chain."""
        xml_file = os.path.join(self.test_dir, "test_remapping.xml")
        sm = self.factory.create_sm_from_file(xml_file)

        self.assertIsNotNone(sm)

        # Set up initial input
        blackboard = Blackboard()
        blackboard["initial_data"] = "start"

        outcome = sm(blackboard)

        # Check the chain of processing
        self.assertEqual(outcome, "end")
        self.assertTrue(blackboard.contains("final_data"))
        self.assertEqual(blackboard.get("final_data"), "processed_start")
        self.assertTrue(blackboard.contains("final_data_2"))
        self.assertEqual(blackboard.get("final_data_2"), "processed_processed_start")

    def test_file_path_mechanism(self):
        """Test including external state machine via file_path attribute."""
        xml_file = os.path.join(self.test_dir, "test_file_path_sm.xml")
        sm = self.factory.create_sm_from_file(xml_file)

        self.assertIsNotNone(sm)
        self.assertIsInstance(sm, StateMachine)

        # Verify the state machine name is set
        self.assertEqual(sm.get_name(), "MainStateMachine")

        # Verify the outcomes include the final outcome
        self.assertIn("final_end", sm.get_outcomes())

        # Execute the state machine
        blackboard = Blackboard()
        outcome = sm(blackboard)

        # The state machine should execute through FirstState -> IncludedSM -> final_end
        # or directly to final_end depending on FirstState's execution
        self.assertIn(outcome, ["final_end"])


if __name__ == "__main__":
    unittest.main()
