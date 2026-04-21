# Copyright (C) 2026 Maik Knof
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
import unittest

from yasmin import Blackboard, CallbackSignal, CbState


class TestCallbackSignal(unittest.TestCase):
    def test_trigger_runs_python_callbacks(self):
        signal = CallbackSignal()
        calls = []

        signal.add_callback(lambda: calls.append("first"))
        signal.add_callback(lambda: calls.append("second"))
        signal.trigger()

        self.assertEqual(["first", "second"], calls)

    def test_remove_callback(self):
        signal = CallbackSignal()
        calls = []

        callback_id = signal.add_callback(lambda: calls.append("removed"))
        signal.add_callback(lambda: calls.append("kept"))

        self.assertTrue(signal.remove_callback(callback_id))
        self.assertFalse(signal.remove_callback(callback_id))

        signal.trigger()
        self.assertEqual(["kept"], calls)

    def test_blackboard_roundtrip_keeps_shared_signal(self):
        blackboard = Blackboard()
        signal = CallbackSignal()
        calls = []

        blackboard["signal"] = signal
        retrieved_signal = blackboard["signal"]
        retrieved_signal.add_callback(lambda: calls.append("shared"))

        signal.trigger()
        self.assertEqual(["shared"], calls)
        self.assertEqual(1, signal.callback_count())

    def test_mixed_python_and_cpp_callbacks_can_coexist(self):
        signal = CallbackSignal()
        cpp_state = CbState(["done"], lambda blackboard: "done")
        calls = []

        signal.add_callback(lambda: calls.append("python"))
        signal.add_cancel_callback(cpp_state)
        signal.trigger()

        self.assertEqual(["python"], calls)
        self.assertTrue(cpp_state.is_canceled())

    def test_trigger_async_returns_future(self):
        signal = CallbackSignal()
        calls = []

        def callback():
            time.sleep(0.05)
            calls.append("done")

        signal.add_callback(callback)
        future = signal.trigger_async()

        self.assertFalse(future.is_completed())
        future.wait()
        self.assertTrue(future.is_completed())
        self.assertFalse(future.has_exception())
        self.assertEqual(["done"], calls)

    def test_trigger_async_propagates_exceptions_via_future(self):
        signal = CallbackSignal()
        signal.add_callback(lambda: (_ for _ in ()).throw(RuntimeError("boom")))

        future = signal.trigger_async()

        with self.assertRaises(RuntimeError):
            future.wait()

        self.assertTrue(future.is_completed())
        self.assertTrue(future.has_exception())
        self.assertEqual("boom", future.get_exception_message())


if __name__ == "__main__":
    unittest.main()
