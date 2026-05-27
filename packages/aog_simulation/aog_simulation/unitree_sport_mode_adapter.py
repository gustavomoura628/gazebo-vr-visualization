import json
from typing import Callable, Dict, Optional

import rclpy
import rclpy.node
import geometry_msgs.msg as geometry_msgs

from unitree_api.msg import Request, Response
import unitree_go.msg as unitree_go_msgs

from . import unitree_api


TOPIC_SPORT_REQUEST = "/api/sport/request"
TOPIC_SPORT_RESPONSE = "/api/sport/response"
TOPIC_SPORT_STATE_HF = "/sportmodestate"
TOPIC_SPORT_STATE_LF = "/lf/sportmodestate"

# High-level gait values returned by GetState.
GAIT_MODE_DEFAULT = 0
GAIT_MODE_STAIR_CLIMBING = 1
GAIT_MODE_HEIGHT_CLIMBING = 2
GAIT_MODE_INVERTED = 3
GAIT_MODE_STANDING_LOCK = -1
GAIT_MODE_LIE_DOWN = -2
GAIT_MODE_DAMPING = -3

# SportModeState.mode values.
SPORT_STATE_MODE_DEFAULT_STAND = 0
SPORT_STATE_MODE_BALANCE_STAND = 1
SPORT_STATE_MODE_POSE = 2
SPORT_STATE_MODE_LOCOMOTION = 3
SPORT_STATE_MODE_LIE_DOWN = 5
SPORT_STATE_MODE_JOINT_LOCK = 6
SPORT_STATE_MODE_DAMPING = 7
SPORT_STATE_MODE_RECOVERY_STAND = 8
SPORT_STATE_MODE_SIT = 10
SPORT_STATE_MODE_FRONT_FLIP = 11
SPORT_STATE_MODE_FRONT_JUMP = 12
SPORT_STATE_MODE_FRONT_POUNCE = 13

# SportModeState.gait_type values.
SPORT_STATE_GAIT_TYPE_IDLE = 0
SPORT_STATE_GAIT_TYPE_TROT = 1
SPORT_STATE_GAIT_TYPE_RUN = 2
SPORT_STATE_GAIT_TYPE_CLIMB_STAIR = 3
SPORT_STATE_GAIT_TYPE_FORWARD_DOWN_STAIR = 4
SPORT_STATE_GAIT_TYPE_ADJUST = 9

SPEED_LEVEL_SLOW = -1
SPEED_LEVEL_NORMAL = 0
SPEED_LEVEL_FAST = 1


class AdapterNode(rclpy.node.Node):
    def __init__(self) -> None:
        super().__init__("unitree_sport_mode_adapter_node")

        # Node parameters
        self.declare_parameter("state_lf_rate_hz", 10.0)
        state_lf_rate_hz = (
            self.get_parameter("state_lf_rate_hz")
            .get_parameter_value()
            .double_value
        )
        if state_lf_rate_hz <= 0.0:
            state_lf_rate_hz = 10.0

        self.declare_parameter("cmd_vel_rate_hz", 10.0)
        cmd_vel_rate_hz = (
            self.get_parameter("cmd_vel_rate_hz")
            .get_parameter_value()
            .double_value
        )
        if cmd_vel_rate_hz <= 0.0:
            cmd_vel_rate_hz = 10.0

        # Internal state
        self.sport_state = unitree_go_msgs.SportModeState()
        self._selected_gait = GAIT_MODE_DEFAULT
        self._current_gait = GAIT_MODE_DEFAULT
        self._standing_mode = SPORT_STATE_MODE_DEFAULT_STAND
        self._current_mode = SPORT_STATE_MODE_DEFAULT_STAND
        self._speed_level = SPEED_LEVEL_NORMAL
        self._sync_sport_state()

        # Map API IDs to handler functions
        self._api_handlers: Dict[int, Callable[[Request], Response]] = {
            unitree_api.SPORT_API_ID_DAMP: self._handle_damp_request,
            unitree_api.SPORT_API_ID_BALANCESTAND: self._handle_balance_stand_request,
            unitree_api.SPORT_API_ID_STOPMOVE: self._handle_stop_move_request,
            unitree_api.SPORT_API_ID_STANDUP: self._handle_stand_up_request,
            unitree_api.SPORT_API_ID_STANDDOWN: self._handle_stand_down_request,
            unitree_api.SPORT_API_ID_RECOVERYSTAND: self._handle_recovery_stand_request,
            unitree_api.SPORT_API_ID_MOVE: self._handle_move_request,
            unitree_api.SPORT_API_ID_SWITCHGAIT: self._handle_switch_gait_request,
            unitree_api.SPORT_API_ID_SPEEDLEVEL: self._handle_speed_level_request,
            unitree_api.SPORT_API_ID_GETSTATE: self._handle_get_state_request,
        }

        # ROS2 interfaces
        self.sport_request_sub = self.create_subscription(
            Request, TOPIC_SPORT_REQUEST, self._on_sport_request, 50
        )

        self.sport_response_pub = self.create_publisher(
            Response, TOPIC_SPORT_RESPONSE, 50
        )

        self.sport_state_lf_pub = self.create_publisher(
            unitree_go_msgs.SportModeState, TOPIC_SPORT_STATE_LF, 1
        )

        self.cmd_vel_pub = self.create_publisher(
            geometry_msgs.Twist, "/cmd_vel", 1
        )

        # Timers
        self.state_lf_timer = self.create_timer(
            1.0 / state_lf_rate_hz, self._publish_sport_state
        )

        self.cmd_vel_timer = self.create_timer(
            1.0 / cmd_vel_rate_hz, self._publish_cmd_vel
        )

        self.get_logger().info("Unitree sport mode adapter node started.")

    def _on_sport_request(self, msg: Request) -> None:
        api_id = msg.header.identity.api_id

        self.get_logger().info(f"Received sport request with API ID: {api_id}")

        handler = self._api_handlers.get(api_id)
        if handler is None:
            self.get_logger().error(f"No handler found for API ID {api_id}.")

            if not msg.header.policy.noreply:
                self.sport_response_pub.publish(
                    self._build_response(
                        msg, code=unitree_api.RPC_ERR_SERVER_API_NOT_IMPL
                    )
                )
        else:
            res = handler(msg)
            if res is not None and not msg.header.policy.noreply:
                self.sport_response_pub.publish(res)

    def _build_response(
        self, req: Request, code: int = unitree_api.RPC_OK, data: str = ""
    ) -> Response:
        res = Response()
        res.header.identity = req.header.identity
        res.header.status.code = code
        res.data = data
        return res

    def _reject_request(self, req: Request, reason: str) -> Response:
        self.get_logger().warning(reason)
        return self._build_response(
            req, code=unitree_api.RPC_ERR_SERVER_LEASE_DENIED
        )

    def _invalid_parameter(self, req: Request, reason: str) -> Response:
        self.get_logger().warning(reason)
        return self._build_response(
            req, code=unitree_api.RPC_ERR_SERVER_API_PARAMETER
        )

    def _parse_parameter_dict(self, req: Request) -> Optional[dict]:
        if not req.parameter:
            return {}

        try:
            parameter = json.loads(req.parameter)
        except json.JSONDecodeError:
            self.get_logger().error(
                f"Failed to decode request parameter JSON: {req.parameter}"
            )
            return None

        if not isinstance(parameter, dict):
            self.get_logger().error(
                f"Expected request parameter object, got: {type(parameter).__name__}"
            )
            return None

        return parameter

    def _set_motion_velocities(self, vx: float, vy: float, vyaw: float) -> None:
        self.sport_state.velocity[0] = float(vx)
        self.sport_state.velocity[1] = float(vy)
        self.sport_state.velocity[2] = 0.0
        self.sport_state.yaw_speed = float(vyaw)

    def _stop_motion(self) -> None:
        self._set_motion_velocities(0.0, 0.0, 0.0)

    def _message_gait_type(self) -> int:
        if self._current_mode != SPORT_STATE_MODE_LOCOMOTION:
            return SPORT_STATE_GAIT_TYPE_IDLE

        if self._selected_gait == GAIT_MODE_STAIR_CLIMBING:
            return SPORT_STATE_GAIT_TYPE_CLIMB_STAIR

        if self._selected_gait == GAIT_MODE_HEIGHT_CLIMBING:
            return SPORT_STATE_GAIT_TYPE_FORWARD_DOWN_STAIR

        if self._speed_level == SPEED_LEVEL_FAST:
            return SPORT_STATE_GAIT_TYPE_RUN

        return SPORT_STATE_GAIT_TYPE_TROT

    def _sync_sport_state(self) -> None:
        self.sport_state.mode = self._current_mode
        self.sport_state.gait_type = self._message_gait_type()

    def _state_map(self) -> Dict[str, str]:
        return {
            "gait": str(self._current_gait),
            "speedLevel": str(self._speed_level),
        }

    def _requested_state_keys(
        self, req: Request
    ) -> tuple[Optional[list[str]], Optional[Response]]:
        if not req.parameter:
            return None, None

        try:
            parameter = json.loads(req.parameter)
        except json.JSONDecodeError:
            return None, self._invalid_parameter(
                req,
                f"GetState rejected due to invalid JSON payload: {req.parameter}",
            )

        if isinstance(parameter, list):
            return [str(item) for item in parameter], None

        if isinstance(parameter, dict):
            requested = parameter.get("keys", parameter.get("data"))
            if requested is None:
                return None, None
            if isinstance(requested, list):
                return [str(item) for item in requested], None

        return None, self._invalid_parameter(
            req,
            "GetState parameter did not contain a supported key list payload.",
        )

    def _handle_damp_request(self, req: Request) -> Response:
        self.get_logger().info(f"Handling DAMP request: {req}")

        self._current_gait = GAIT_MODE_DAMPING
        self._current_mode = SPORT_STATE_MODE_DAMPING
        self._stop_motion()
        self._sync_sport_state()

        return self._build_response(req)

    def _handle_balance_stand_request(self, req: Request) -> Response:
        self.get_logger().info(f"Handling BALANCESTAND request: {req}")

        self._current_gait = self._selected_gait
        self._standing_mode = SPORT_STATE_MODE_BALANCE_STAND
        self._current_mode = self._standing_mode
        self._stop_motion()
        self._sync_sport_state()

        return self._build_response(req)

    def _handle_stop_move_request(self, req: Request) -> Response:
        self.get_logger().info(f"Handling STOPMOVE request: {req}")

        self._speed_level = SPEED_LEVEL_NORMAL
        self._current_mode = self._standing_mode
        if self._standing_mode == SPORT_STATE_MODE_JOINT_LOCK:
            self._current_gait = GAIT_MODE_STANDING_LOCK
        else:
            self._current_gait = self._selected_gait
        self._stop_motion()
        self._sync_sport_state()

        return self._build_response(req)

    def _handle_stand_up_request(self, req: Request) -> Response:
        self.get_logger().info(f"Handling STANDUP request: {req}")

        self._current_gait = GAIT_MODE_STANDING_LOCK
        self._standing_mode = SPORT_STATE_MODE_JOINT_LOCK
        self._current_mode = self._standing_mode
        self._stop_motion()
        self._sync_sport_state()

        return self._build_response(req)

    def _handle_stand_down_request(self, req: Request) -> Response:
        self.get_logger().info(f"Handling STANDDOWN request: {req}")

        if self._current_gait not in (
            GAIT_MODE_STANDING_LOCK,
            GAIT_MODE_DAMPING,
        ):
            return self._reject_request(
                req,
                "StandDown rejected because the robot is not in standing lock or damping state.",
            )

        self._current_gait = GAIT_MODE_LIE_DOWN
        self._current_mode = SPORT_STATE_MODE_LIE_DOWN
        self._stop_motion()
        self._sync_sport_state()

        return self._build_response(req)

    def _handle_recovery_stand_request(self, req: Request) -> Response:
        self.get_logger().info(f"Handling RECOVERYSTAND request: {req}")

        if self._current_gait not in (GAIT_MODE_LIE_DOWN, GAIT_MODE_DAMPING):
            return self._reject_request(
                req,
                "RecoveryStand rejected because the robot is not in lie-down or damping state.",
            )

        self._current_gait = self._selected_gait
        self._standing_mode = SPORT_STATE_MODE_BALANCE_STAND
        self._current_mode = self._standing_mode
        self._stop_motion()
        self._sync_sport_state()

        return self._build_response(req)

    def _handle_switch_gait_request(self, req: Request) -> Response:
        self.get_logger().info(f"Handling SWITCHGAIT request: {req}")

        parameter = self._parse_parameter_dict(req)
        if parameter is None:
            return self._invalid_parameter(
                req, "SwitchGait rejected due to invalid JSON payload."
            )

        gait = parameter.get("data")
        if gait not in (
            GAIT_MODE_DEFAULT,
            GAIT_MODE_STAIR_CLIMBING,
            GAIT_MODE_HEIGHT_CLIMBING,
        ):
            return self._invalid_parameter(
                req,
                (
                    "SwitchGait rejected because gait value "
                    f"{gait!r} is outside the documented range [0, 2]."
                ),
            )

        self._selected_gait = int(gait)
        if self._current_mode == SPORT_STATE_MODE_LOCOMOTION:
            self._current_gait = self._selected_gait
        self._sync_sport_state()

        return self._build_response(req)

    def _handle_speed_level_request(self, req: Request) -> Response:
        self.get_logger().info(f"Handling SPEEDLEVEL request: {req}")

        if self._selected_gait != GAIT_MODE_DEFAULT:
            return self._reject_request(
                req,
                "SpeedLevel rejected because it is only effective in default gait mode.",
            )

        parameter = self._parse_parameter_dict(req)
        if parameter is None:
            return self._invalid_parameter(
                req, "SpeedLevel rejected due to invalid JSON payload."
            )

        level = parameter.get("data")
        if level not in (
            SPEED_LEVEL_SLOW,
            SPEED_LEVEL_NORMAL,
            SPEED_LEVEL_FAST,
        ):
            return self._invalid_parameter(
                req,
                (
                    "SpeedLevel rejected because level value "
                    f"{level!r} is outside the documented range [-1, 1]."
                ),
            )

        self._speed_level = int(level)
        return self._build_response(req)

    def _handle_move_request(self, req: Request) -> Response:
        self.get_logger().info(f"Handling MOVE request: {req}")

        if self._current_gait == GAIT_MODE_DAMPING:
            return self._reject_request(
                req,
                "Move rejected because the robot is currently in damping state.",
            )

        parameter = self._parse_parameter_dict(req)
        if parameter is None:
            return self._invalid_parameter(
                req, "Move rejected due to invalid JSON payload."
            )

        try:
            self._set_motion_velocities(
                float(parameter.get("x", 0.0)),
                float(parameter.get("y", 0.0)),
                float(parameter.get("z", 0.0)),
            )
        except (TypeError, ValueError):
            return self._invalid_parameter(
                req,
                "Move rejected because x, y, or z could not be converted to float values.",
            )

        self._current_gait = self._selected_gait
        self._current_mode = SPORT_STATE_MODE_LOCOMOTION
        self._sync_sport_state()

        return self._build_response(req)

    def _handle_get_state_request(self, req: Request) -> Response:
        self.get_logger().info(f"Handling GETSTATE request: {req}")

        state_map = self._state_map()
        requested_keys, error_response = self._requested_state_keys(req)
        if error_response is not None:
            return error_response

        if requested_keys is not None:
            state_map = {
                key: state_map[key]
                for key in requested_keys
                if key in state_map
            }

        return self._build_response(req, data=json.dumps(state_map))

    def _publish_sport_state(self) -> None:
        sec, nsec = self.get_clock().now().seconds_nanoseconds()
        self.sport_state.stamp.sec = sec
        self.sport_state.stamp.nanosec = nsec
        self.sport_state_lf_pub.publish(self.sport_state)

    def _publish_cmd_vel(self) -> None:
        cmd_vel = geometry_msgs.Twist()

        if self._current_mode == SPORT_STATE_MODE_LOCOMOTION:
            cmd_vel.linear.x = float(self.sport_state.velocity[0])
            cmd_vel.linear.y = float(self.sport_state.velocity[1])
            cmd_vel.angular.z = float(self.sport_state.yaw_speed)

        self.cmd_vel_pub.publish(cmd_vel)


def main() -> None:
    rclpy.init()
    node = AdapterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
