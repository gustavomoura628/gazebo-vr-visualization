import json
import sys
import threading
import time
from dataclasses import dataclass
from typing import Optional

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

from unitree_api.msg import Request, Response

from . import unitree_api
from .unitree_sport_mode_adapter import (
    TOPIC_SPORT_REQUEST,
    TOPIC_SPORT_RESPONSE,
)


@dataclass
class TestOption:
    name: Optional[str]
    id: Optional[int]


option_list = [
    TestOption(name="damp", id=0),
    TestOption(name="stand_up", id=1),
    TestOption(name="stand_down", id=2),
    TestOption(name="move", id=3),
    TestOption(name="stop_move", id=4),
    TestOption(name="speed_level", id=5),
    TestOption(name="switch_gait", id=6),
    TestOption(name="get_state", id=7),
    TestOption(name="recovery", id=8),
    TestOption(name="balance", id=9),
]


class UserInterface:
    def __init__(self) -> None:
        self.test_option_ = TestOption(name=None, id=None)

    def convert_to_int(self, input_str: str) -> Optional[int]:
        try:
            return int(input_str)
        except ValueError:
            return None

    def terminal_handle(self) -> None:
        input_str = input("Enter id or name: \n")

        if input_str == "list":
            self.test_option_.name = None
            self.test_option_.id = None
            for option in option_list:
                print(f"{option.name}, id: {option.id}")
            return

        for option in option_list:
            if (
                input_str == option.name
                or self.convert_to_int(input_str) == option.id
            ):
                self.test_option_.name = option.name
                self.test_option_.id = option.id
                print(
                    f"Test: {self.test_option_.name}, test_id: {self.test_option_.id}"
                )
                return

        print("No matching test option found.")


class UnitreeSportClientRosNode(Node):
    def __init__(self) -> None:
        super().__init__("unitree_sport_client_ros")

        self.declare_parameter("response_timeout_sec", 10.0)
        self._response_timeout_sec = (
            self.get_parameter("response_timeout_sec")
            .get_parameter_value()
            .double_value
        )
        if self._response_timeout_sec <= 0.0:
            self._response_timeout_sec = 10.0

        self._request_pub = self.create_publisher(
            Request, TOPIC_SPORT_REQUEST, 50
        )
        self._response_sub = self.create_subscription(
            Response, TOPIC_SPORT_RESPONSE, self._on_response, 50
        )

        self._pending_lock = threading.Lock()
        self._pending_events: dict[int, threading.Event] = {}
        self._pending_responses: dict[int, Response] = {}

        self.get_logger().info("Unitree sport ROS client node started.")

    def _on_response(self, msg: Response) -> None:
        request_id = int(msg.header.identity.id)

        with self._pending_lock:
            event = self._pending_events.get(request_id)
            if event is None:
                return
            self._pending_responses[request_id] = msg

        event.set()

    def _build_request(
        self, api_id: int, parameter: str, noreply: bool
    ) -> Request:
        req = Request()
        req.header.identity.id = time.monotonic_ns()
        req.header.identity.api_id = api_id
        req.header.lease.id = 0
        req.header.policy.priority = 0
        req.header.policy.noreply = noreply
        req.parameter = parameter
        return req

    def _call(
        self, api_id: int, payload: Optional[dict] = None, noreply: bool = False
    ) -> tuple[int, Optional[str]]:
        parameter = json.dumps(payload or {})
        req = self._build_request(api_id, parameter, noreply)
        request_id = int(req.header.identity.id)

        if noreply:
            self._request_pub.publish(req)
            return unitree_api.RPC_OK, None

        event = threading.Event()
        with self._pending_lock:
            self._pending_events[request_id] = event

        self._request_pub.publish(req)

        if not event.wait(timeout=self._response_timeout_sec):
            with self._pending_lock:
                self._pending_events.pop(request_id, None)
                self._pending_responses.pop(request_id, None)
            return unitree_api.RPC_ERR_CLIENT_API_TIMEOUT, None

        with self._pending_lock:
            self._pending_events.pop(request_id, None)
            response = self._pending_responses.pop(request_id, None)

        if response is None:
            return unitree_api.RPC_ERR_UNKNOWN, None

        if int(response.header.identity.api_id) != api_id:
            return unitree_api.RPC_ERR_CLIENT_API_NOT_MATCH, None

        return int(response.header.status.code), response.data or None

    def damp(self) -> tuple[int, Optional[str]]:
        return self._call(unitree_api.SPORT_API_ID_DAMP)

    def stand_up(self) -> tuple[int, Optional[str]]:
        return self._call(unitree_api.SPORT_API_ID_STANDUP)

    def stand_down(self) -> tuple[int, Optional[str]]:
        return self._call(unitree_api.SPORT_API_ID_STANDDOWN)

    def move(
        self, vx: float, vy: float, vyaw: float
    ) -> tuple[int, Optional[str]]:
        return self._call(
            unitree_api.SPORT_API_ID_MOVE,
            payload={"x": vx, "y": vy, "z": vyaw},
            noreply=True,
        )

    def stop_move(self) -> tuple[int, Optional[str]]:
        return self._call(unitree_api.SPORT_API_ID_STOPMOVE)

    def speed_level(self, level: int) -> tuple[int, Optional[str]]:
        return self._call(
            unitree_api.SPORT_API_ID_SPEEDLEVEL, payload={"data": level}
        )

    def switch_gait(self, gait: int) -> tuple[int, Optional[str]]:
        return self._call(
            unitree_api.SPORT_API_ID_SWITCHGAIT, payload={"data": gait}
        )

    def get_state(self) -> tuple[int, Optional[str]]:
        return self._call(unitree_api.SPORT_API_ID_GETSTATE)

    def recovery_stand(self) -> tuple[int, Optional[str]]:
        return self._call(unitree_api.SPORT_API_ID_RECOVERYSTAND)

    def balance_stand(self) -> tuple[int, Optional[str]]:
        return self._call(unitree_api.SPORT_API_ID_BALANCESTAND)


def _execute_option(
    node: UnitreeSportClientRosNode, test_option: TestOption
) -> tuple[int, Optional[str]]:
    if test_option.id == 0:
        return node.damp()
    if test_option.id == 1:
        return node.stand_up()
    if test_option.id == 2:
        return node.stand_down()
    if test_option.id == 3:
        return node.move(0.5, 0.0, 0.0)
    if test_option.id == 4:
        return node.stop_move()
    if test_option.id == 5:
        return node.speed_level(1)
    if test_option.id == 6:
        return node.switch_gait(1)
    if test_option.id == 7:
        return node.get_state()
    if test_option.id == 8:
        return node.recovery_stand()
    if test_option.id == 9:
        return node.balance_stand()
    return unitree_api.RPC_OK, None


def _run_terminal(node: UnitreeSportClientRosNode) -> None:
    test_option = TestOption(name=None, id=None)
    user_interface = UserInterface()
    user_interface.test_option_ = test_option

    print(
        "WARNING: Please ensure there are no obstacles around the robot while running this example."
    )
    input("Press Enter to continue...")

    while rclpy.ok():
        user_interface.terminal_handle()

        print(
            f"Updated Test Option: Name = {test_option.name}, ID = {test_option.id}\n"
        )

        code, data = _execute_option(node, test_option)
        if code != unitree_api.RPC_OK:
            print(f"Request failed with code: {code}")
        elif test_option.id == 7 and data is not None:
            print(f"State: {data}")

        time.sleep(1)


def main() -> None:
    rclpy.init(args=sys.argv)
    node = UnitreeSportClientRosNode()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        _run_terminal(node)
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        spin_thread.join(timeout=1.0)


if __name__ == "__main__":
    main()
