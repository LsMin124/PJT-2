"""MQTT ↔ ROS bridge for VDA 5050.

MQTT  uagv/v2/{manufacturer}/{serial}/order          → ROS `order`            (std_msgs/String, JSON)
MQTT  uagv/v2/{manufacturer}/{serial}/instantActions → ROS `instant_actions`  (std_msgs/String, JSON)
ROS   `state` (JSON)                                  → MQTT .../state
MQTT  .../connection: ONLINE (retained) on connect, OFFLINE on shutdown, CONNECTIONBROKEN as LWT.
"""
from __future__ import annotations

import json
import queue
import time

import paho.mqtt.client as mqtt
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from ..vda5050 import topics as T
from ..vda5050.state import StateBuilder


class Vda5050Bridge(Node):
    def __init__(self) -> None:
        super().__init__("vda5050_bridge")
        p = self.declare_parameters("", [
            ("serial", "amr01"), ("manufacturer", "santa"),
            ("mqtt_host", "localhost"), ("mqtt_port", 1883), ("mqtt_username", ""),
            ("mqtt_password", ""), ("mqtt_keepalive", 30), ("republish_s", 2.0),
        ])
        g = {x.name: x.value for x in p}
        self.serial, self.manufacturer = g["serial"], g["manufacturer"]
        self.t_order = T.topic(self.manufacturer, self.serial, T.ORDER)
        self.t_instant = T.topic(self.manufacturer, self.serial, T.INSTANT_ACTIONS)
        self.t_state = T.topic(self.manufacturer, self.serial, T.STATE)
        self.t_conn = T.topic(self.manufacturer, self.serial, T.CONNECTION)
        self.sb = StateBuilder(self.manufacturer, self.serial)
        self.inbox: "queue.Queue[tuple[str, str]]" = queue.Queue()
        self.last_state = None
        self.last_state_t = 0.0
        self.republish_s = float(g["republish_s"])

        self.order_pub = self.create_publisher(String, "order", 10)
        self.instant_pub = self.create_publisher(String, "instant_actions", 10)
        self.create_subscription(String, "state", self._on_state, 10)
        self.create_timer(0.01, self._drain)
        self.create_timer(0.5, self._heartbeat)

        self.client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2,
                                  client_id=f"{self.manufacturer}-{self.serial}-agent",
                                  protocol=mqtt.MQTTv311, clean_session=True)
        if g["mqtt_username"]:
            self.client.username_pw_set(g["mqtt_username"], g["mqtt_password"] or None)
        self.client.will_set(self.t_conn, json.dumps(self.sb.connection("CONNECTIONBROKEN")),
                             qos=1, retain=True)
        self.client.on_connect = self._on_connect
        self.client.on_disconnect = self._on_disconnect
        self.client.on_message = self._on_message
        self.client.reconnect_delay_set(1, 10)
        self.client.connect_async(g["mqtt_host"], int(g["mqtt_port"]), int(g["mqtt_keepalive"]))
        self.client.loop_start()
        self.get_logger().info(f"bridge up — {g['mqtt_host']}:{g['mqtt_port']} base={T.base(self.manufacturer, self.serial)}")

    # ---- MQTT thread -------------------------------------------------------------
    def _on_connect(self, client, _userdata, _flags, reason_code, _props=None) -> None:
        if reason_code != 0:
            self.get_logger().error(f"MQTT connect failed: {reason_code}")
            return
        client.publish(self.t_conn, json.dumps(self.sb.connection("ONLINE")), qos=1, retain=True)
        client.subscribe([(self.t_order, 0), (self.t_instant, 0)])
        self.get_logger().info("MQTT connected — ONLINE published, order/instantActions subscribed")

    def _on_disconnect(self, _client, _userdata, _flags, reason_code, _props=None) -> None:
        self.get_logger().warn(f"MQTT disconnected: {reason_code}")

    def _on_message(self, _client, _userdata, m) -> None:
        try:
            payload = m.payload.decode("utf-8")
            d = json.loads(payload)
        except (UnicodeDecodeError, ValueError) as e:
            self.get_logger().warn(f"bad JSON on {m.topic}: {e}")
            return
        if str(d.get("serialNumber", self.serial)) != self.serial:
            self.get_logger().warn(f"message for serial {d.get('serialNumber')} ignored")
            return
        self.inbox.put((m.topic, payload))

    # ---- ROS thread --------------------------------------------------------------
    def _drain(self) -> None:
        while True:
            try:
                topic, payload = self.inbox.get_nowait()
            except queue.Empty:
                return
            pub = self.order_pub if topic == self.t_order else self.instant_pub
            pub.publish(String(data=payload))

    def _on_state(self, msg: String) -> None:
        self.last_state, self.last_state_t = msg.data, time.monotonic()
        self.client.publish(self.t_state, msg.data, qos=0)

    def _heartbeat(self) -> None:
        if self.last_state and time.monotonic() - self.last_state_t > self.republish_s:
            self.last_state_t = time.monotonic()
            self.client.publish(self.t_state, self.last_state, qos=0)

    def shutdown(self) -> None:
        try:
            info = self.client.publish(self.t_conn, json.dumps(self.sb.connection("OFFLINE")), qos=1, retain=True)
            info.wait_for_publish(timeout=2.0)
        except Exception as e:  # noqa: BLE001 — best effort on shutdown
            self.get_logger().warn(f"OFFLINE publish failed: {e}")
        self.client.loop_stop()
        self.client.disconnect()


def main() -> None:
    rclpy.init()
    n = Vda5050Bridge()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.shutdown()
        n.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
