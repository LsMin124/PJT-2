"""VDA 5050 state / connection message builders (pure Python)."""
from __future__ import annotations

from datetime import datetime, timezone
from typing import Iterable, Optional, Tuple

from .order import OrderBook
from .topics import PROTOCOL_VERSION


def now_iso() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds").replace("+00:00", "Z")


def make_error(error_type: str, description: str, level: str = "WARNING",
               references: Iterable[Tuple[str, str]] = ()) -> dict:
    return {"errorType": error_type, "errorLevel": level, "errorDescription": description,
            "errorReferences": [{"referenceKey": k, "referenceValue": v} for k, v in references]}


class StateBuilder:
    def __init__(self, manufacturer: str, serial: str, map_id: str = "") -> None:
        self.manufacturer = manufacturer
        self.serial = serial
        self.map_id = map_id
        self._header = 0

    def _header_block(self) -> dict:
        self._header += 1
        return {"headerId": self._header, "timestamp": now_iso(), "version": PROTOCOL_VERSION,
                "manufacturer": self.manufacturer, "serialNumber": self.serial}

    def state(self, book: OrderBook, pose: Optional[Tuple[float, float, float]],
              velocity: Tuple[float, float, float] = (0.0, 0.0, 0.0), driving: bool = False,
              paused: bool = False, errors: Iterable[dict] = (), information: Iterable[dict] = (),
              battery_charge: float = 100.0, operating_mode: str = "AUTOMATIC",
              localization_score: Optional[float] = None, new_base_request: bool = False,
              distance_since_last_node: float = 0.0, action_states: Iterable[dict] = ()) -> dict:
        last = book.last_node
        pos = None
        if pose is not None:
            pos = {"x": pose[0], "y": pose[1], "theta": pose[2], "mapId": self.map_id,
                   "positionInitialized": True}
            if localization_score is not None:
                pos["localizationScore"] = localization_score
        msg = {**self._header_block(),
               "orderId": book.order_id, "orderUpdateId": book.order_update_id, "zoneSetId": "",
               "lastNodeId": last.node_id if last else "",
               "lastNodeSequenceId": last.sequence_id if last else 0,
               "nodeStates": book.node_states(), "edgeStates": book.edge_states(),
               "driving": driving, "paused": paused, "newBaseRequest": new_base_request,
               "distanceSinceLastNode": distance_since_last_node,
               "actionStates": list(action_states),
               "batteryState": {"batteryCharge": battery_charge, "charging": False},
               "operatingMode": operating_mode,
               "errors": list(errors), "information": list(information),
               "safetyState": {"eStop": "NONE", "fieldViolation": False},
               "velocity": {"vx": velocity[0], "vy": velocity[1], "omega": velocity[2]},
               "loads": []}
        if pos is not None:
            msg["agvPosition"] = pos
        return msg

    def connection(self, connection_state: str) -> dict:
        assert connection_state in ("ONLINE", "OFFLINE", "CONNECTIONBROKEN")
        return {**self._header_block(), "connectionState": connection_state}
