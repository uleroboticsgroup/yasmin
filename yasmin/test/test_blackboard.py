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

    def test_set_get_none(self):
        """Test setting and getting None value"""
        self.blackboard["none_key"] = None
        self.assertIsNone(self.blackboard["none_key"])

    def test_set_get_list(self):
        """Test setting and getting list values"""
        test_list = [1, 2, 3, "four", 5.0]
        self.blackboard["list_key"] = test_list
        retrieved_list = self.blackboard["list_key"]
        self.assertIsInstance(retrieved_list, list)
        self.assertEqual(len(test_list), len(retrieved_list))
        for i in range(len(test_list)):
            self.assertEqual(test_list[i], retrieved_list[i])

    def test_set_get_empty_list(self):
        """Test setting and getting empty list"""
        self.blackboard["empty_list"] = []
        retrieved_list = self.blackboard["empty_list"]
        self.assertIsInstance(retrieved_list, list)
        self.assertEqual(0, len(retrieved_list))

    def test_set_get_tuple(self):
        """Test setting and getting tuple values (stored as list)"""
        test_tuple = (1, 2, 3, "four", 5.0)
        self.blackboard["tuple_key"] = test_tuple
        retrieved_value = self.blackboard["tuple_key"]
        # Tuples are stored as lists internally
        self.assertIsInstance(retrieved_value, tuple)
        self.assertEqual(len(test_tuple), len(retrieved_value))
        for i in range(len(test_tuple)):
            self.assertEqual(test_tuple[i], retrieved_value[i])

    def test_set_get_dict(self):
        """Test setting and getting dictionary values"""
        test_dict = {"a": 1, "b": "two", "c": 3.0, "d": True}
        self.blackboard["dict_key"] = test_dict
        retrieved_dict = self.blackboard["dict_key"]
        self.assertIsInstance(retrieved_dict, dict)
        self.assertEqual(len(test_dict), len(retrieved_dict))
        for key in test_dict:
            self.assertEqual(test_dict[key], retrieved_dict[key])

    def test_set_get_empty_dict(self):
        """Test setting and getting empty dictionary"""
        self.blackboard["empty_dict"] = {}
        retrieved_dict = self.blackboard["empty_dict"]
        self.assertIsInstance(retrieved_dict, dict)
        self.assertEqual(0, len(retrieved_dict))

    def test_set_get_nested_list(self):
        """Test setting and getting nested list structures"""
        nested_list = [[1, 2], [3, 4], [5, [6, 7]]]
        self.blackboard["nested_list"] = nested_list
        retrieved_list = self.blackboard["nested_list"]
        self.assertEqual(len(nested_list), len(retrieved_list))
        self.assertEqual(nested_list[0], retrieved_list[0])
        self.assertEqual(nested_list[2][1], retrieved_list[2][1])

    def test_set_get_nested_dict(self):
        """Test setting and getting nested dictionary structures"""
        nested_dict = {"outer": {"inner": "value", "number": 42}, "list": [1, 2, 3]}
        self.blackboard["nested_dict"] = nested_dict
        retrieved_dict = self.blackboard["nested_dict"]
        self.assertEqual(nested_dict["outer"]["inner"], retrieved_dict["outer"]["inner"])
        self.assertEqual(
            nested_dict["outer"]["number"], retrieved_dict["outer"]["number"]
        )

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


if __name__ == "__main__":
    unittest.main()
