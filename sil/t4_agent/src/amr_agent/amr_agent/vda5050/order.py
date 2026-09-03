"""VDA 5050 order model and the executor-side order book (pure Python, no ROS).

Mapping used by this project (FMS = PIBT tick loop):
- one FMS tick  = one order update that releases exactly one more node
- forward move  = edge with no orientation (or orientation 0, TANGENTIAL)
- reverse move  = edge with orientation = pi, orientationType TANGENTIAL
- spot turn     = node at the same position with a new theta (zero-length edge)
- ack           = state.lastNodeId == released node (published on change)
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Tuple


class OrderError(ValueError):
    """Order rejected: malformed or violates VDA 5050 acceptance rules."""


def wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def _opt_float(v) -> Optional[float]:
    if v is None:
        return None
    try:
        return float(v)
    except (TypeError, ValueError) as e:
        raise OrderError(f"expected number, got {v!r}") from e


@dataclass(frozen=True)
class NodePosition:
    x: float
    y: float
    theta: Optional[float] = None
    allowed_deviation_xy: Optional[float] = None
    allowed_deviation_theta: Optional[float] = None
    map_id: str = ""

    @classmethod
    def from_dict(cls, d: dict) -> "NodePosition":
        if not isinstance(d, dict):
            raise OrderError("nodePosition missing")
        try:
            x, y = float(d["x"]), float(d["y"])
        except (KeyError, TypeError, ValueError) as e:
            raise OrderError(f"nodePosition x/y invalid: {d}") from e
        return cls(x, y, _opt_float(d.get("theta")), _opt_float(d.get("allowedDeviationXY")),
                   _opt_float(d.get("allowedDeviationTheta")), str(d.get("mapId", "")))

    def to_dict(self) -> dict:
        out = {"x": self.x, "y": self.y, "mapId": self.map_id}
        if self.theta is not None:
            out["theta"] = self.theta
        if self.allowed_deviation_xy is not None:
            out["allowedDeviationXY"] = self.allowed_deviation_xy
        if self.allowed_deviation_theta is not None:
            out["allowedDeviationTheta"] = self.allowed_deviation_theta
        return out


@dataclass(frozen=True)
class Node:
    node_id: str
    sequence_id: int
    released: bool
    position: NodePosition
    actions: tuple = ()

    @classmethod
    def from_dict(cls, d: dict) -> "Node":
        try:
            node_id = str(d["nodeId"])
            seq = int(d["sequenceId"])
            released = bool(d["released"])
        except (KeyError, TypeError, ValueError) as e:
            raise OrderError(f"node fields invalid: {d}") from e
        return cls(node_id, seq, released, NodePosition.from_dict(d.get("nodePosition")),
                   tuple(d.get("actions") or ()))

    def to_state_dict(self) -> dict:
        return {"nodeId": self.node_id, "sequenceId": self.sequence_id,
                "released": self.released, "nodePosition": self.position.to_dict()}

    def to_order_dict(self) -> dict:
        d = self.to_state_dict()
        d["actions"] = list(self.actions)
        return d


@dataclass(frozen=True)
class Edge:
    edge_id: str
    sequence_id: int
    released: bool
    start_node_id: str
    end_node_id: str
    orientation: Optional[float] = None
    orientation_type: str = "TANGENTIAL"
    max_speed: Optional[float] = None
    rotation_allowed: Optional[bool] = None
    length: Optional[float] = None
    actions: tuple = ()

    @classmethod
    def from_dict(cls, d: dict) -> "Edge":
        try:
            edge_id = str(d["edgeId"])
            seq = int(d["sequenceId"])
            released = bool(d["released"])
            start, end = str(d["startNodeId"]), str(d["endNodeId"])
        except (KeyError, TypeError, ValueError) as e:
            raise OrderError(f"edge fields invalid: {d}") from e
        ra = d.get("rotationAllowed")
        return cls(edge_id, seq, released, start, end, _opt_float(d.get("orientation")),
                   str(d.get("orientationType", "TANGENTIAL")).upper(), _opt_float(d.get("maxSpeed")),
                   None if ra is None else bool(ra), _opt_float(d.get("length")),
                   tuple(d.get("actions") or ()))

    def is_reverse(self) -> bool:
        """VDA 5050: TANGENTIAL orientation 0 = forwards, pi = backwards."""
        return (self.orientation is not None and self.orientation_type == "TANGENTIAL"
                and abs(wrap(self.orientation)) > math.pi / 2)

    def to_state_dict(self) -> dict:
        return {"edgeId": self.edge_id, "sequenceId": self.sequence_id, "released": self.released}

    def to_order_dict(self) -> dict:
        d = {**self.to_state_dict(), "startNodeId": self.start_node_id, "endNodeId": self.end_node_id,
             "orientationType": self.orientation_type, "actions": list(self.actions)}
        if self.orientation is not None:
            d["orientation"] = self.orientation
        if self.max_speed is not None:
            d["maxSpeed"] = self.max_speed
        if self.rotation_allowed is not None:
            d["rotationAllowed"] = self.rotation_allowed
        if self.length is not None:
            d["length"] = self.length
        return d


@dataclass(frozen=True)
class Order:
    order_id: str
    order_update_id: int
    nodes: Tuple[Node, ...]
    edges: Tuple[Edge, ...]
    header_id: int = 0
    timestamp: str = ""
    zone_set_id: str = ""

    @classmethod
    def from_dict(cls, d: dict) -> "Order":
        if not isinstance(d, dict):
            raise OrderError("order is not an object")
        order_id = str(d.get("orderId", "")).strip()
        if not order_id:
            raise OrderError("orderId missing")
        try:
            upd = int(d.get("orderUpdateId", 0))
        except (TypeError, ValueError) as e:
            raise OrderError("orderUpdateId invalid") from e
        if upd < 0:
            raise OrderError("orderUpdateId must be >= 0")
        nodes = tuple(Node.from_dict(n) for n in (d.get("nodes") or ()))
        edges = tuple(Edge.from_dict(e) for e in (d.get("edges") or ()))
        o = cls(order_id, upd, nodes, edges, int(d.get("headerId", 0) or 0),
                str(d.get("timestamp", "")), str(d.get("zoneSetId", "")))
        o.validate()
        return o

    def validate(self) -> None:
        if not self.nodes:
            raise OrderError("order has no nodes")
        if len(self.edges) != len(self.nodes) - 1:
            raise OrderError(f"edges ({len(self.edges)}) must be nodes-1 ({len(self.nodes) - 1})")
        prev_seq = -1
        released_phase = True
        for i, n in enumerate(self.nodes):
            if n.sequence_id % 2 != 0:
                raise OrderError(f"node {n.node_id} sequenceId {n.sequence_id} must be even")
            if n.sequence_id <= prev_seq:
                raise OrderError(f"node {n.node_id} sequenceId not increasing")
            prev_seq = n.sequence_id
            if n.released and not released_phase:
                raise OrderError(f"node {n.node_id}: released node after an unreleased one")
            if not n.released:
                released_phase = False
        for i, e in enumerate(self.edges):
            a, b = self.nodes[i], self.nodes[i + 1]
            if e.sequence_id != a.sequence_id + 1:
                raise OrderError(f"edge {e.edge_id} sequenceId must be {a.sequence_id + 1}")
            if e.start_node_id != a.node_id or e.end_node_id != b.node_id:
                raise OrderError(f"edge {e.edge_id} does not connect {a.node_id}->{b.node_id}")
            if e.released != b.released:
                raise OrderError(f"edge {e.edge_id} released flag must match its end node")

    def base_len(self) -> int:
        n = 0
        for node in self.nodes:
            if not node.released:
                break
            n += 1
        return n

    def to_dict(self, manufacturer: str, serial: str, header_id: int, timestamp: str,
                version: str = "2.0.0") -> dict:
        return {"headerId": header_id, "timestamp": timestamp, "version": version,
                "manufacturer": manufacturer, "serialNumber": serial,
                "orderId": self.order_id, "orderUpdateId": self.order_update_id,
                "nodes": [n.to_order_dict() for n in self.nodes],
                "edges": [e.to_order_dict() for e in self.edges]}


class OrderBook:
    """Active order + traversal cursor + VDA 5050 acceptance/stitching rules. No I/O."""

    def __init__(self) -> None:
        self.order: Optional[Order] = None
        self.cursor = 0            # index of the next node to reach
        self.last_node: Optional[Node] = None

    # ---- queries -------------------------------------------------------------
    @property
    def active(self) -> bool:
        return self.order is not None and self.cursor < self.order.base_len()

    def next_target(self) -> Optional[Tuple[Optional[Edge], Node]]:
        if not self.active:
            return None
        node = self.order.nodes[self.cursor]
        edge = self.order.edges[self.cursor - 1] if self.cursor >= 1 else None
        return edge, node

    def node_states(self) -> list:
        if self.order is None:
            return []
        return [n.to_state_dict() for n in self.order.nodes[self.cursor:]]

    def edge_states(self) -> list:
        if self.order is None or self.cursor == 0:
            return [e.to_state_dict() for e in (self.order.edges if self.order else ())]
        return [e.to_state_dict() for e in self.order.edges[self.cursor - 1:]]

    @property
    def order_id(self) -> str:
        return self.order.order_id if self.order else ""

    @property
    def order_update_id(self) -> int:
        return self.order.order_update_id if self.order else 0

    # ---- transitions ---------------------------------------------------------
    def receive(self, incoming: Order, at_xy: Optional[Tuple[float, float]] = None,
                first_node_tol: float = 0.5) -> str:
        """Apply an incoming order. Returns 'started' | 'stitched'. Raises OrderError."""
        cur = self.order
        if cur is not None and incoming.order_id == cur.order_id:
            if incoming.order_update_id <= cur.order_update_id:
                raise OrderError(f"orderUpdateId {incoming.order_update_id} not greater than "
                                 f"{cur.order_update_id}")
            self._stitch(incoming)
            return "stitched"
        if self.active:
            raise OrderError(f"order {cur.order_id} still active; cancelOrder first")
        first = incoming.nodes[0]
        if at_xy is not None:
            tol = first.position.allowed_deviation_xy or first_node_tol
            d = math.hypot(first.position.x - at_xy[0], first.position.y - at_xy[1])
            if d > tol:
                raise OrderError(f"first node {first.node_id} is {d:.2f} m away (> {tol:.2f})")
        self.order = incoming
        self.last_node = first
        self.cursor = 1
        return "started"

    def _stitch(self, upd: Order) -> None:
        cur = self.order
        lb = cur.base_len()
        last = cur.nodes[lb - 1]
        first = upd.nodes[0]
        if (first.node_id, first.sequence_id) != (last.node_id, last.sequence_id):
            raise OrderError(f"update must start at last base node {last.node_id}/{last.sequence_id}, "
                             f"got {first.node_id}/{first.sequence_id}")
        new = Order(cur.order_id, upd.order_update_id, cur.nodes[:lb] + upd.nodes[1:],
                    cur.edges[:lb - 1] + upd.edges, upd.header_id, upd.timestamp, upd.zone_set_id)
        new.validate()
        self.order = new

    def node_reached(self) -> Node:
        if not self.active:
            raise OrderError("no active node to reach")
        node = self.order.nodes[self.cursor]
        self.last_node = node
        self.cursor += 1
        return node

    def cancel(self) -> None:
        """Drop everything not yet reached. lastNode stays so an update can stitch onto it."""
        if self.order is None:
            return
        keep = max(self.cursor, 1)
        self.order = Order(self.order.order_id, self.order.order_update_id,
                           self.order.nodes[:keep], self.order.edges[:keep - 1],
                           self.order.header_id, self.order.timestamp, self.order.zone_set_id)
        self.cursor = keep
