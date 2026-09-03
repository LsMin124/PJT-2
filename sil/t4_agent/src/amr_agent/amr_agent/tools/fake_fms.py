"""Fake FMS — drives one agent through a grid plan with the barrier-tick protocol.

Plan letters (cell = 1 m, heading snapped to a multiple of 90°):
  F = forward one cell   B = reverse one cell   L / R = spot turn 90°
Each tick = one order update that releases exactly one more node; the next tick is
sent only after state.lastNodeId acknowledges the previous one. Prints per-tick latency
and final position error. Exit code 0 = plan completed within tolerance.

  python3 -m amr_agent.tools.fake_fms --host localhost --serial amr01 --plan F,F,L,F,B
"""
from __future__ import annotations

import argparse
import json
import math
import sys
import threading
import time

import paho.mqtt.client as mqtt

from ..vda5050 import topics as T
from ..vda5050.order import Edge, Node, NodePosition, Order
from ..vda5050.state import now_iso


def snap_quarter(theta: float) -> float:
    return round(theta / (math.pi / 2)) * (math.pi / 2)


def build_plan(start, plan: str, cell: float, map_id: str, dev_xy: float, dev_th: float):
    x, y, h = start[0], start[1], snap_quarter(start[2])
    nodes = [Node("n0", 0, True, NodePosition(x, y, h, dev_xy, dev_th, map_id))]
    edges = []
    for i, step in enumerate([s.strip().upper() for s in plan.split(",") if s.strip()], start=1):
        rev = False
        if step == "F":
            x, y = x + cell * math.cos(h), y + cell * math.sin(h)
        elif step == "B":
            x, y = x - cell * math.cos(h), y - cell * math.sin(h)
            rev = True
        elif step == "L":
            h = math.atan2(math.sin(h + math.pi / 2), math.cos(h + math.pi / 2))
        elif step == "R":
            h = math.atan2(math.sin(h - math.pi / 2), math.cos(h - math.pi / 2))
        else:
            raise SystemExit(f"unknown plan step {step!r}")
        nodes.append(Node(f"n{i}", 2 * i, False, NodePosition(x, y, h, dev_xy, dev_th, map_id)))
        edges.append(Edge(f"e{i - 1}", 2 * i - 1, False, f"n{i - 1}", f"n{i}",
                          orientation=math.pi if rev else None, orientation_type="TANGENTIAL",
                          length=0.0 if step in ("L", "R") else cell))
    return nodes, edges


def release_upto(nodes, edges, k):
    """Order view where nodes[0..k] (and their edges) are released, the rest horizon."""
    ns = tuple(Node(n.node_id, n.sequence_id, i <= k, n.position, n.actions) for i, n in enumerate(nodes))
    es = tuple(Edge(e.edge_id, e.sequence_id, i + 1 <= k, e.start_node_id, e.end_node_id, e.orientation,
                    e.orientation_type, e.max_speed, e.rotation_allowed, e.length, e.actions)
               for i, e in enumerate(edges))
    return ns, es


class FakeFms:
    def __init__(self, args) -> None:
        self.a = args
        self.t_state = T.topic(args.manufacturer, args.serial, T.STATE)
        self.t_order = T.topic(args.manufacturer, args.serial, T.ORDER)
        self.t_conn = T.topic(args.manufacturer, args.serial, T.CONNECTION)
        self.state = None
        self.conn = None
        self.cv = threading.Condition()
        self.header = 0
        self.c = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=f"fake-fms-{args.serial}",
                             protocol=mqtt.MQTTv311)
        self.c.on_connect = lambda c, u, f, rc, p=None: c.subscribe([(self.t_state, 0), (self.t_conn, 1)])
        self.c.on_message = self._on_message
        self.c.connect(args.host, args.port, 30)
        self.c.loop_start()

    def _on_message(self, _c, _u, m) -> None:
        try:
            d = json.loads(m.payload.decode("utf-8"))
        except ValueError:
            return
        with self.cv:
            if m.topic == self.t_state:
                self.state = d
            else:
                self.conn = d
            self.cv.notify_all()

    def wait_state(self, pred, timeout: float):
        t0 = time.monotonic()
        with self.cv:
            while True:
                if self.state is not None:
                    if self.state.get("errors"):
                        return None, self.state["errors"]
                    if pred(self.state):
                        return self.state, None
                left = timeout - (time.monotonic() - t0)
                if left <= 0:
                    return None, [{"errorType": "timeout", "errorDescription": f"no ack in {timeout}s"}]
                self.cv.wait(min(left, 0.5))

    def send_order(self, order: Order) -> None:
        self.header += 1
        self.c.publish(self.t_order, json.dumps(order.to_dict(self.a.manufacturer, self.a.serial,
                                                              self.header, now_iso())), qos=0)

    def run(self) -> int:
        st, err = self.wait_state(lambda s: "agvPosition" in s, self.a.timeout)
        if st is None:
            print(f"[fake_fms] no agent state: {err}")
            return 2
        pos = st["agvPosition"]
        start = (pos["x"], pos["y"], pos["theta"])
        print(f"[fake_fms] agent at ({start[0]:.2f}, {start[1]:.2f}, {math.degrees(start[2]):.0f}°) "
              f"connection={self.conn.get('connectionState') if self.conn else '?'}")
        nodes, edges = build_plan(start, self.a.plan, self.a.cell, self.a.map_id, self.a.dev_xy, self.a.dev_theta)
        N = len(nodes) - 1
        ns, es = release_upto(nodes, edges, 0)
        self.send_order(Order(self.a.order_id, 0, ns, es))
        st, err = self.wait_state(lambda s: s.get("orderId") == self.a.order_id and s.get("lastNodeId") == "n0",
                                  self.a.timeout)
        if st is None:
            print(f"[fake_fms] order not accepted: {err}")
            return 1
        print(f"[fake_fms] order {self.a.order_id} accepted, {N} ticks to go")
        print(f"{'tick':>4} {'step':>4} {'target':>16} {'ack s':>7} {'pos err m':>9} {'θ err °':>7}")
        steps = [s.strip().upper() for s in self.a.plan.split(",") if s.strip()]
        worst_xy = worst_th = 0.0
        t_all = time.monotonic()
        for k in range(1, N + 1):
            ns, es = release_upto(nodes, edges, k)
            t0 = time.monotonic()
            self.send_order(Order(self.a.order_id, k, ns[k - 1:], es[k - 1:]))   # starts at last base node
            st, err = self.wait_state(lambda s, k=k: s.get("lastNodeId") == f"n{k}"
                                      and int(s.get("orderUpdateId", -1)) == k, self.a.timeout)
            if st is None:
                print(f"[fake_fms] tick {k} failed: {err}")
                self.c.loop_stop()
                return 1
            dt = time.monotonic() - t0
            tgt = nodes[k].position
            p = st.get("agvPosition", {})
            exy = math.hypot(p.get("x", 0) - tgt.x, p.get("y", 0) - tgt.y)
            eth = abs(math.degrees(math.atan2(math.sin(p.get("theta", 0) - tgt.theta),
                                              math.cos(p.get("theta", 0) - tgt.theta))))
            worst_xy, worst_th = max(worst_xy, exy), max(worst_th, eth)
            print(f"{k:>4} {steps[k - 1]:>4} ({tgt.x:6.2f},{tgt.y:6.2f}) {dt:7.2f} {exy:9.3f} {eth:7.1f}")
        total = time.monotonic() - t_all
        ok = worst_xy <= self.a.dev_xy and worst_th <= math.degrees(self.a.dev_theta)
        print(f"[fake_fms] done: {N} ticks in {total:.1f}s — worst pos err {worst_xy:.3f} m, "
              f"worst θ err {worst_th:.1f}° → {'PASS' if ok else 'FAIL'}")
        self.c.loop_stop()
        return 0 if ok else 1


def main(argv=None) -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default="localhost")
    ap.add_argument("--port", type=int, default=1883)
    ap.add_argument("--manufacturer", default="santa")
    ap.add_argument("--serial", default="amr01")
    ap.add_argument("--map-id", default="")
    ap.add_argument("--plan", default="F,F,L,F,B")
    ap.add_argument("--cell", type=float, default=1.0)
    ap.add_argument("--dev-xy", type=float, default=0.1, help="allowedDeviationXY per node [m]")
    ap.add_argument("--dev-theta", type=float, default=0.087, help="allowedDeviationTheta per node [rad]")
    ap.add_argument("--timeout", type=float, default=30.0, help="per-tick ack timeout [s]")
    ap.add_argument("--order-id", default=f"demo-{int(time.time()) % 100000}")
    sys.exit(FakeFms(ap.parse_args(argv)).run())


if __name__ == "__main__":
    main()
