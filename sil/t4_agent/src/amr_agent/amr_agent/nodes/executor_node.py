"""Order executor — VDA 5050 order (JSON on `order`) → MovePrimitive goals → `state` JSON.

Per released node: [TURN to face the edge] → DRIVE (forward or reverse) → [TURN to node theta].
A zero-length edge (node at the same position) is a pure spot turn.
State is published on every change (order accepted/rejected, node reached, pause, error)
plus a slow heartbeat; the FMS barrier tick keys on state.lastNodeId.
"""
from __future__ import annotations

import json
import math

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String

from amr_agent_msgs.action import MovePrimitive

from ..vda5050.order import Edge, Node as VNode, Order, OrderBook, OrderError, wrap
from ..vda5050.state import StateBuilder, make_error


def yaw_of(q) -> float:
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class OrderExecutor(Node):
    def __init__(self) -> None:
        super().__init__("order_executor")
        p = self.declare_parameters("", [
            ("serial", "amr01"), ("manufacturer", "santa"), ("map_id", ""),
            ("pose_source", "odom"), ("tol_xy", 0.05), ("tol_theta", 0.03),
            ("turn_threshold", 0.12), ("zero_edge_eps", 0.05),
            ("state_rate_hz", 2.0), ("first_node_tol", 0.5),
        ])
        g = {x.name: x.value for x in p}
        self.serial, self.manufacturer = g["serial"], g["manufacturer"]
        self.pose_source = g["pose_source"]
        self.tol_xy, self.tol_theta = g["tol_xy"], g["tol_theta"]
        self.turn_threshold, self.zero_edge_eps = g["turn_threshold"], g["zero_edge_eps"]
        self.first_node_tol = g["first_node_tol"]

        self.book = OrderBook()
        self.sb = StateBuilder(self.manufacturer, self.serial, g["map_id"])
        self.pose = None
        self.vel = (0.0, 0.0, 0.0)
        self.paused = False
        self.driving = False
        self.errors: list = []
        self.action_states: list = []
        self.queue: list = []
        self.inflight = None          # None | "pending" | ClientGoalHandle

        self.create_subscription(String, "order", self._on_order, 10)
        self.create_subscription(String, "instant_actions", self._on_instant, 10)
        self.create_subscription(Odometry, "odom", self._on_odom, 20)
        qos_tl = QoSProfile(depth=5)
        qos_tl.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(PoseWithCovarianceStamped, "amcl_pose", self._on_amcl, qos_tl)
        self.state_pub = self.create_publisher(String, "state", 10)
        self.client = ActionClient(self, MovePrimitive, "move_primitive")
        self.create_timer(0.05, self._step)
        self.create_timer(1.0 / float(g["state_rate_hz"]), self._publish_state)
        self.get_logger().info(f"order executor up — serial={self.serial} pose_source={self.pose_source}")

    # ---- inputs ----------------------------------------------------------------
    def _on_odom(self, m: Odometry) -> None:
        t = m.twist.twist
        self.vel = (t.linear.x, t.linear.y, t.angular.z)
        if self.pose_source == "odom":
            p = m.pose.pose
            self.pose = (p.position.x, p.position.y, yaw_of(p.orientation))

    def _on_amcl(self, m: PoseWithCovarianceStamped) -> None:
        if self.pose_source == "amcl":
            p = m.pose.pose
            self.pose = (p.position.x, p.position.y, yaw_of(p.orientation))

    def _on_order(self, msg: String) -> None:
        try:
            d = json.loads(msg.data)
            order = Order.from_dict(d)
        except (ValueError, OrderError) as e:
            self._reject("validationError", str(e))
            return
        if d.get("serialNumber") and str(d["serialNumber"]) != self.serial:
            self._reject("validationError", f"serialNumber {d['serialNumber']} != {self.serial}")
            return
        try:
            how = self.book.receive(order, self.pose[:2] if self.pose else None, self.first_node_tol)
        except OrderError as e:
            self._reject("orderError", str(e), [("orderId", order.order_id),
                                                ("orderUpdateId", str(order.order_update_id))])
            return
        self.errors = []
        self.get_logger().info(f"order {order.order_id}#{order.order_update_id} {how}: "
                               f"base={order.base_len()} nodes={len(order.nodes)}")
        self._publish_state()

    def _reject(self, error_type: str, desc: str, refs=()) -> None:
        self.get_logger().warn(f"order rejected ({error_type}): {desc}")
        self.errors = [make_error(error_type, desc, "WARNING", refs)]
        self._publish_state()

    def _on_instant(self, msg: String) -> None:
        try:
            actions = json.loads(msg.data).get("actions") or []
        except ValueError as e:
            self.get_logger().warn(f"instantActions unparsable: {e}")
            return
        for a in actions:
            kind, aid = str(a.get("actionType", "")), str(a.get("actionId", ""))
            status = "FINISHED"
            if kind == "cancelOrder":
                self._cancel_inflight()
                self.queue.clear()
                self.book.cancel()
                self.driving = False
            elif kind == "startPause":
                self.paused = True
                self._cancel_inflight()
                self.queue.clear()
                self.driving = False
            elif kind == "stopPause":
                self.paused = False
            elif kind == "stateRequest":
                pass
            else:
                status = "FAILED"
                self.get_logger().warn(f"unsupported instantAction {kind}")
            self.action_states = [s for s in self.action_states if s["actionId"] != aid]
            self.action_states.append({"actionId": aid, "actionType": kind, "actionStatus": status})
            self.action_states = self.action_states[-20:]
        self._publish_state()

    # ---- execution ---------------------------------------------------------------
    def _step(self) -> None:
        if self.paused or self.inflight is not None or self.pose is None:
            return
        if self.queue:
            self._send(self.queue.pop(0))
            return
        tgt = self.book.next_target()
        if tgt is None:
            return
        edge, node = tgt
        self.queue = self._plan(edge, node, self.pose)
        if self.queue:
            self._send(self.queue.pop(0))
        else:
            self._node_done()

    def _plan(self, edge, node: VNode, pose) -> list:
        x, y, yaw = pose
        gx, gy = node.position.x, node.position.y
        goals = []
        tol_xy = min(node.position.allowed_deviation_xy or self.tol_xy, self.tol_xy)
        tol_th = min(node.position.allowed_deviation_theta or self.tol_theta, self.tol_theta)
        heading_after = yaw
        if math.hypot(gx - x, gy - y) > self.zero_edge_eps:
            path_dir = math.atan2(gy - y, gx - x)
            reverse = edge.is_reverse() if edge is not None else False
            face = wrap(path_dir + math.pi) if reverse else path_dir
            if abs(wrap(face - yaw)) > self.turn_threshold:
                goals.append(self._goal(MovePrimitive.Goal.TURN, theta=face, tol_theta=tol_th))
            goals.append(self._goal(MovePrimitive.Goal.DRIVE, x=gx, y=gy, theta=face, reverse=reverse,
                                    tol_xy=tol_xy, max_speed=(edge.max_speed if edge and edge.max_speed else 0.0)))
            heading_after = face
        if node.position.theta is not None and abs(wrap(node.position.theta - heading_after)) > tol_th:
            goals.append(self._goal(MovePrimitive.Goal.TURN, theta=node.position.theta, tol_theta=tol_th))
        return goals

    @staticmethod
    def _goal(kind, x=0.0, y=0.0, theta=0.0, reverse=False, tol_xy=0.0, tol_theta=0.0, max_speed=0.0):
        g = MovePrimitive.Goal()
        g.kind, g.x, g.y, g.theta, g.reverse = kind, float(x), float(y), float(theta), bool(reverse)
        g.tolerance_xy, g.tolerance_theta, g.max_speed = float(tol_xy), float(tol_theta), float(max_speed)
        return g

    def _send(self, goal) -> None:
        if not self.client.server_is_ready():
            self.queue.insert(0, goal)
            return
        self.driving = goal.kind == MovePrimitive.Goal.DRIVE
        self.inflight = "pending"
        self.client.send_goal_async(goal).add_done_callback(self._on_goal_response)

    def _on_goal_response(self, fut) -> None:
        gh = fut.result()
        if not gh.accepted:
            self.inflight = None
            self._fail("primitive goal rejected by controller")
            return
        self.inflight = gh
        gh.get_result_async().add_done_callback(self._on_result)

    def _on_result(self, fut) -> None:
        wrapped = fut.result()
        self.inflight = None
        if wrapped.status == GoalStatus.STATUS_CANCELED or not self.book.active:
            return
        if not wrapped.result.success:
            self._fail(f"primitive failed: {wrapped.result.message}")
            return
        if self.queue:
            self._send(self.queue.pop(0))
        else:
            self._node_done()

    def _node_done(self) -> None:
        node = self.book.node_reached()
        self.driving = False
        self.get_logger().info(f"node reached {node.node_id}/{node.sequence_id}")
        self._publish_state()

    def _fail(self, msg: str) -> None:
        self.get_logger().error(msg)
        self.queue.clear()
        self.driving = False
        self.errors = [make_error("primitiveError", msg, "FATAL", [("orderId", self.book.order_id)])]
        self.book.cancel()
        self._publish_state()

    def _cancel_inflight(self) -> None:
        if isinstance(self.inflight, ClientGoalHandle):
            self.inflight.cancel_goal_async()
        self.inflight = None

    def _publish_state(self) -> None:
        msg = self.sb.state(self.book, self.pose, self.vel, self.driving, self.paused,
                            self.errors, action_states=self.action_states)
        self.state_pub.publish(String(data=json.dumps(msg)))


def main() -> None:
    rclpy.init()
    n = OrderExecutor()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
