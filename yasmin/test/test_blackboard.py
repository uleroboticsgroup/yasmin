# Copyright (C) 2023 Miguel Ángel González Santamarta

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


import copy
import unittest
from yasmin import Blackboard


class TestBlackboard(unittest.TestCase):

    def setUp(self):
        self.blackboard = Blackboard()

    def tearDown(self):
        self.blackboard.set_remappings({})

    def test_get(self):
        self.blackboard["foo"] = "foo"
        self.blackboard.bar = "bar"
        self.assertEqual("foo", self.blackboard["foo"])
        self.assertEqual("bar", self.blackboard.bar)

    def test_delete(self):
        self.blackboard["foo"] = "foo"
        del self.blackboard["foo"]
        self.assertFalse("foo" in self.blackboard)

    def test_contains(self):
        self.blackboard["foo"] = "foo"
        self.assertTrue("foo" in self.blackboard)

    def test_len(self):
        self.blackboard["foo"] = "foo"
        self.assertEqual(1, len(self.blackboard))

    def test_remapping(self):
        self.blackboard.set("foo", "foo")
        self.blackboard.set_remappings({"bar": "foo"})
        self.assertEqual("foo", self.blackboard.get("bar"))

    def test_copy_constructor_keeps_remappings_local_and_shares_values(self):
        self.blackboard["foo"] = "foo"
        self.blackboard["count"] = 1

        blackboard_copy = Blackboard(self.blackboard)

        self.blackboard.set_remappings({"alias": "foo"})
        self.assertEqual({"alias": "foo"}, self.blackboard.get_remappings())
        self.assertEqual({}, blackboard_copy.get_remappings())
        self.assertEqual("foo", self.blackboard["alias"])
        self.assertFalse("alias" in blackboard_copy)

        blackboard_copy.set_remappings({"copy_alias": "foo"})
        self.assertEqual({"alias": "foo"}, self.blackboard.get_remappings())
        self.assertEqual({"copy_alias": "foo"}, blackboard_copy.get_remappings())
        self.assertEqual("foo", blackboard_copy["copy_alias"])
        self.assertFalse("copy_alias" in self.blackboard)

        self.blackboard["count"] = 2
        self.assertEqual(2, self.blackboard["count"])
        self.assertEqual(2, blackboard_copy["count"])

    def test_copy_module_uses_blackboard_copy_constructor(self):
        self.blackboard["foo"] = "foo"
        self.blackboard.set_remappings({"alias": "foo"})

        blackboard_copy = copy.copy(self.blackboard)

        self.assertEqual({"alias": "foo"}, blackboard_copy.get_remappings())

        self.blackboard.set_remappings({"other_alias": "foo"})
        self.assertEqual({"alias": "foo"}, blackboard_copy.get_remappings())
        self.assertEqual({"other_alias": "foo"}, self.blackboard.get_remappings())
        self.assertEqual("foo", blackboard_copy["alias"])

    def test_set_get_string(self):
        """Test setting and getting string values"""
        self.blackboard["string_key"] = "test_string"
        self.assertEqual("test_string", self.blackboard["string_key"])
        self.assertIsInstance(self.blackboard["string_key"], str)

    def test_set_get_int(self):
        """Test setting and getting integer values"""
        self.blackboard["int_key"] = 42
        self.assertEqual(42, self.blackboard["int_key"])
        self.assertIsInstance(self.blackboard["int_key"], int)

    def test_set_get_negative_int(self):
        """Test setting and getting negative integer values"""
        self.blackboard["negative_int"] = -100
        self.assertEqual(-100, self.blackboard["negative_int"])

    def test_set_get_large_int(self):
        """Test setting and getting large integer values"""
        large_int = 9223372036854775807  # max int64_t
        self.blackboard["large_int"] = large_int
        self.assertEqual(large_int, self.blackboard["large_int"])

    def test_set_get_float(self):
        """Test setting and getting float values"""
        self.blackboard["float_key"] = 3.14159
        self.assertAlmostEqual(3.14159, self.blackboard["float_key"], places=5)
        self.assertIsInstance(self.blackboard["float_key"], float)

    def test_set_get_negative_float(self):
        """Test setting and getting negative float values"""
        self.blackboard["negative_float"] = -2.71828
        self.assertAlmostEqual(-2.71828, self.blackboard["negative_float"], places=5)

    def test_set_get_bool_true(self):
        """Test setting and getting boolean True value"""
        self.blackboard["bool_true"] = True
        self.assertTrue(self.blackboard["bool_true"])
        self.assertIsInstance(self.blackboard["bool_true"], bool)

    def test_set_get_bool_false(self):
        """Test setting and getting boolean False value"""
        self.blackboard["bool_false"] = False
        self.assertFalse(self.blackboard["bool_false"])
        self.assertIsInstance(self.blackboard["bool_false"], bool)

    def test_set_get_bytes(self):
        """Test setting and getting bytes values"""
        payload = bytes([0, 1, 2, 3, 127, 128, 255])
        self.blackboard["bytes_key"] = payload
        retrieved_payload = self.blackboard["bytes_key"]
        self.assertIsInstance(retrieved_payload, bytes)
        self.assertEqual(payload, retrieved_payload)

    def test_set_get_bytearray(self):
        """Test setting and getting bytearray values as bytes"""
        payload = bytearray([0, 1, 2, 3, 127, 128, 255])
        self.blackboard["bytearray_key"] = payload
        retrieved_payload = self.blackboard["bytearray_key"]
        self.assertIsInstance(retrieved_payload, bytes)
        self.assertEqual(bytes(payload), retrieved_payload)

    def test_set_get_none(self):
        """Test setting and getting None value"""
        self.blackboard["none_key"] = None
        self.assertIsNone(self.blackboard["none_key"])

    def test_set_get_mixed_list(self):
        """Test setting and getting mixed list values via py::object"""
        test_list = [1, 2, 3, "four", 5.0]
        self.blackboard["list_key"] = test_list
        retrieved_list = self.blackboard["list_key"]
        self.assertIsInstance(retrieved_list, list)
        self.assertEqual(test_list, retrieved_list)

    def test_set_get_empty_list(self):
        """Test setting and getting empty list"""
        self.blackboard["empty_list"] = []
        retrieved_list = self.blackboard["empty_list"]
        self.assertIsInstance(retrieved_list, list)
        self.assertEqual([], retrieved_list)

    def test_set_get_list_of_strings(self):
        """Test setting and getting homogeneous string lists"""
        test_list = ["one", "two", "three"]
        self.blackboard["string_list"] = test_list
        retrieved_list = self.blackboard["string_list"]
        self.assertIsInstance(retrieved_list, list)
        self.assertEqual(test_list, retrieved_list)
        for value in retrieved_list:
            self.assertIsInstance(value, str)

    def test_set_get_list_of_ints(self):
        """Test setting and getting homogeneous integer lists"""
        test_list = [1, 2, 3, 4]
        self.blackboard["int_list"] = test_list
        retrieved_list = self.blackboard["int_list"]
        self.assertIsInstance(retrieved_list, list)
        self.assertEqual(test_list, retrieved_list)
        for value in retrieved_list:
            self.assertIsInstance(value, int)
            self.assertNotIsInstance(value, bool)

    def test_set_get_list_of_floats(self):
        """Test setting and getting homogeneous float lists"""
        test_list = [1.5, 2.5, 3.5]
        self.blackboard["float_list"] = test_list
        retrieved_list = self.blackboard["float_list"]
        self.assertIsInstance(retrieved_list, list)
        self.assertEqual(test_list, retrieved_list)
        for value in retrieved_list:
            self.assertIsInstance(value, float)

    def test_set_get_list_of_bools(self):
        """Test setting and getting homogeneous boolean lists"""
        test_list = [True, False, True]
        self.blackboard["bool_list"] = test_list
        retrieved_list = self.blackboard["bool_list"]
        self.assertIsInstance(retrieved_list, list)
        self.assertEqual(test_list, retrieved_list)
        for value in retrieved_list:
            self.assertIsInstance(value, bool)

    def test_set_get_mixed_numeric_list(self):
        """Test mixed numeric lists normalized to float values"""
        test_list = [1, 2.5, 3]
        self.blackboard["mixed_numeric_list"] = test_list
        retrieved_list = self.blackboard["mixed_numeric_list"]
        self.assertIsInstance(retrieved_list, list)
        self.assertEqual([1.0, 2.5, 3.0], retrieved_list)
        for value in retrieved_list:
            self.assertIsInstance(value, float)

    def test_set_get_mixed_tuple(self):
        """Test setting and getting mixed tuple values via py::object"""
        test_tuple = (1, 2, 3, "four", 5.0)
        self.blackboard["tuple_key"] = test_tuple
        retrieved_value = self.blackboard["tuple_key"]
        self.assertIsInstance(retrieved_value, tuple)
        self.assertEqual(test_tuple, retrieved_value)

    def test_set_get_tuple_of_strings(self):
        """Test homogeneous string tuples round-trip as lists"""
        test_tuple = ("one", "two", "three")
        self.blackboard["string_tuple"] = test_tuple
        retrieved_value = self.blackboard["string_tuple"]
        self.assertIsInstance(retrieved_value, list)
        self.assertEqual(["one", "two", "three"], retrieved_value)

    def test_set_get_tuple_of_ints(self):
        """Test homogeneous integer tuples round-trip as lists"""
        test_tuple = (1, 2, 3, 4)
        self.blackboard["int_tuple"] = test_tuple
        retrieved_value = self.blackboard["int_tuple"]
        self.assertIsInstance(retrieved_value, list)
        self.assertEqual([1, 2, 3, 4], retrieved_value)
        for value in retrieved_value:
            self.assertIsInstance(value, int)
            self.assertNotIsInstance(value, bool)

    def test_set_get_tuple_of_floats(self):
        """Test homogeneous float tuples round-trip as lists"""
        test_tuple = (1.5, 2.5, 3.5)
        self.blackboard["float_tuple"] = test_tuple
        retrieved_value = self.blackboard["float_tuple"]
        self.assertIsInstance(retrieved_value, list)
        self.assertEqual([1.5, 2.5, 3.5], retrieved_value)
        for value in retrieved_value:
            self.assertIsInstance(value, float)

    def test_set_get_tuple_of_bools(self):
        """Test homogeneous boolean tuples round-trip as lists"""
        test_tuple = (True, False, True)
        self.blackboard["bool_tuple"] = test_tuple
        retrieved_value = self.blackboard["bool_tuple"]
        self.assertIsInstance(retrieved_value, list)
        self.assertEqual([True, False, True], retrieved_value)
        for value in retrieved_value:
            self.assertIsInstance(value, bool)

    def test_set_get_mixed_numeric_tuple(self):
        """Test mixed numeric tuples round-trip as float lists"""
        test_tuple = (1, 2.5, 3)
        self.blackboard["mixed_numeric_tuple"] = test_tuple
        retrieved_value = self.blackboard["mixed_numeric_tuple"]
        self.assertIsInstance(retrieved_value, list)
        self.assertEqual([1.0, 2.5, 3.0], retrieved_value)
        for value in retrieved_value:
            self.assertIsInstance(value, float)

    def test_set_get_mixed_dict(self):
        """Test setting and getting mixed dictionary values via py::object"""
        test_dict = {"a": 1, "b": "two", "c": 3.0, "d": True}
        self.blackboard["dict_key"] = test_dict
        retrieved_dict = self.blackboard["dict_key"]
        self.assertIsInstance(retrieved_dict, dict)
        self.assertEqual(test_dict, retrieved_dict)

    def test_set_get_empty_dict(self):
        """Test setting and getting empty dictionary"""
        self.blackboard["empty_dict"] = {}
        retrieved_dict = self.blackboard["empty_dict"]
        self.assertIsInstance(retrieved_dict, dict)
        self.assertEqual({}, retrieved_dict)

    def test_set_get_dict_of_strings(self):
        """Test setting and getting homogeneous string dictionaries"""
        test_dict = {"a": "one", "b": "two"}
        self.blackboard["string_dict"] = test_dict
        retrieved_dict = self.blackboard["string_dict"]
        self.assertIsInstance(retrieved_dict, dict)
        self.assertEqual(test_dict, retrieved_dict)
        for value in retrieved_dict.values():
            self.assertIsInstance(value, str)

    def test_set_get_dict_of_ints(self):
        """Test setting and getting homogeneous integer dictionaries"""
        test_dict = {"a": 1, "b": 2}
        self.blackboard["int_dict"] = test_dict
        retrieved_dict = self.blackboard["int_dict"]
        self.assertIsInstance(retrieved_dict, dict)
        self.assertEqual(test_dict, retrieved_dict)
        for value in retrieved_dict.values():
            self.assertIsInstance(value, int)
            self.assertNotIsInstance(value, bool)

    def test_set_get_dict_of_floats(self):
        """Test setting and getting homogeneous float dictionaries"""
        test_dict = {"a": 1.5, "b": 2.5}
        self.blackboard["float_dict"] = test_dict
        retrieved_dict = self.blackboard["float_dict"]
        self.assertIsInstance(retrieved_dict, dict)
        self.assertEqual(test_dict, retrieved_dict)
        for value in retrieved_dict.values():
            self.assertIsInstance(value, float)

    def test_set_get_dict_of_bools(self):
        """Test setting and getting homogeneous boolean dictionaries"""
        test_dict = {"a": True, "b": False}
        self.blackboard["bool_dict"] = test_dict
        retrieved_dict = self.blackboard["bool_dict"]
        self.assertIsInstance(retrieved_dict, dict)
        self.assertEqual(test_dict, retrieved_dict)
        for value in retrieved_dict.values():
            self.assertIsInstance(value, bool)

    def test_set_get_mixed_numeric_dict(self):
        """Test mixed numeric dictionaries normalized to float values"""
        test_dict = {"a": 1, "b": 2.5, "c": 3}
        self.blackboard["mixed_numeric_dict"] = test_dict
        retrieved_dict = self.blackboard["mixed_numeric_dict"]
        self.assertIsInstance(retrieved_dict, dict)
        self.assertEqual({"a": 1.0, "b": 2.5, "c": 3.0}, retrieved_dict)
        for value in retrieved_dict.values():
            self.assertIsInstance(value, float)

    def test_set_get_dict_with_non_string_keys(self):
        """Test dictionaries with non-string keys via py::object"""
        test_dict = {1: "one", 2: "two"}
        self.blackboard["non_string_key_dict"] = test_dict
        retrieved_dict = self.blackboard["non_string_key_dict"]
        self.assertIsInstance(retrieved_dict, dict)
        self.assertEqual(test_dict, retrieved_dict)

    def test_set_get_nested_list(self):
        """Test setting and getting nested list structures"""
        nested_list = [[1, 2], [3, 4], [5, [6, 7]]]
        self.blackboard["nested_list"] = nested_list
        retrieved_list = self.blackboard["nested_list"]
        self.assertIsInstance(retrieved_list, list)
        self.assertEqual(nested_list, retrieved_list)

    def test_set_get_nested_dict(self):
        """Test setting and getting nested dictionary structures"""
        nested_dict = {"outer": {"inner": "value", "number": 42}, "list": [1, 2, 3]}
        self.blackboard["nested_dict"] = nested_dict
        retrieved_dict = self.blackboard["nested_dict"]
        self.assertIsInstance(retrieved_dict, dict)
        self.assertEqual(nested_dict, retrieved_dict)

    def test_set_get_custom_object(self):
        """Test setting and getting custom Python objects"""

        class CustomObject:
            def __init__(self, value):
                self.value = value

        custom_obj = CustomObject(123)
        self.blackboard["custom_obj"] = custom_obj
        retrieved_obj = self.blackboard["custom_obj"]
        self.assertIsInstance(retrieved_obj, CustomObject)
        self.assertEqual(123, retrieved_obj.value)

    def test_overwrite_value(self):
        """Test overwriting existing values"""
        self.blackboard["key"] = "original"
        self.assertEqual("original", self.blackboard["key"])
        self.blackboard["key"] = "updated"
        self.assertEqual("updated", self.blackboard["key"])

    def test_overwrite_with_different_type(self):
        """Test overwriting values with different types"""
        self.blackboard["key"] = 42
        self.assertEqual(42, self.blackboard["key"])
        self.blackboard["key"] = "now_a_string"
        self.assertEqual("now_a_string", self.blackboard["key"])
        self.blackboard["key"] = [1, 2, 3]
        self.assertEqual([1, 2, 3], self.blackboard["key"])

    def test_multiple_keys_different_types(self):
        """Test storing multiple keys with different types simultaneously"""
        self.blackboard["int_val"] = 10
        self.blackboard["str_val"] = "hello"
        self.blackboard["float_val"] = 3.14
        self.blackboard["bool_val"] = True
        self.blackboard["list_val"] = [1, 2, 3]
        self.blackboard["dict_val"] = {"key": "value"}

        self.assertEqual(10, self.blackboard["int_val"])
        self.assertEqual("hello", self.blackboard["str_val"])
        self.assertAlmostEqual(3.14, self.blackboard["float_val"], places=2)
        self.assertTrue(self.blackboard["bool_val"])
        self.assertEqual([1, 2, 3], self.blackboard["list_val"])
        self.assertEqual({"key": "value"}, self.blackboard["dict_val"])

    def test_zero_values(self):
        """Test storing zero/empty values for different types"""
        self.blackboard["zero_int"] = 0
        self.blackboard["zero_float"] = 0.0
        self.blackboard["empty_string"] = ""

        self.assertEqual(0, self.blackboard["zero_int"])
        self.assertEqual(0.0, self.blackboard["zero_float"])
        self.assertEqual("", self.blackboard["empty_string"])

    def test_keys(self):
        self.blackboard["foo"] = "foo"
        self.blackboard["bar"] = 3
        self.assertEqual(["bar", "foo"], self.blackboard.keys())

    def test_iter(self):
        self.blackboard["foo"] = "foo"
        self.blackboard["bar"] = 3
        self.assertEqual(["bar", "foo"], list(self.blackboard))

    def test_values(self):
        self.blackboard["foo"] = "foo"
        self.blackboard["bar"] = 3
        self.assertEqual([3, "foo"], self.blackboard.values())

    def test_items(self):
        self.blackboard["foo"] = "foo"
        self.blackboard["bar"] = 3
        self.assertEqual([("bar", 3), ("foo", "foo")], self.blackboard.items())

    def test_keys_with_remappings(self):
        self.blackboard["shared"] = "value"
        self.blackboard["plain"] = 3
        self.blackboard.set_remappings({"first": "shared", "second": "shared"})
        self.assertEqual(["first", "plain", "second"], self.blackboard.keys())

    def test_iter_with_remappings(self):
        self.blackboard["shared"] = "value"
        self.blackboard["plain"] = 3
        self.blackboard.set_remappings({"first": "shared", "second": "shared"})
        self.assertEqual(["first", "plain", "second"], list(self.blackboard))

    def test_values_with_remappings(self):
        self.blackboard["shared"] = "value"
        self.blackboard["plain"] = 3
        self.blackboard.set_remappings({"first": "shared", "second": "shared"})
        self.assertEqual(["value", 3, "value"], self.blackboard.values())

    def test_items_with_remappings(self):
        self.blackboard["shared"] = "value"
        self.blackboard["plain"] = 3
        self.blackboard.set_remappings({"first": "shared", "second": "shared"})
        self.assertEqual(
            [("first", "value"), ("plain", 3), ("second", "value")],
            self.blackboard.items(),
        )


if __name__ == "__main__":
    unittest.main()
