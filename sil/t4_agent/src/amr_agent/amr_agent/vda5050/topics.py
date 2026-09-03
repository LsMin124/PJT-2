"""VDA 5050 topic layout.

{interfaceName}/{majorVersion}/{manufacturer}/{serialNumber}/{topic}
"""

INTERFACE_NAME = "uagv"
MAJOR_VERSION = "v2"
PROTOCOL_VERSION = "2.0.0"

ORDER = "order"
INSTANT_ACTIONS = "instantActions"
STATE = "state"
CONNECTION = "connection"
VISUALIZATION = "visualization"
FACTSHEET = "factsheet"


def base(manufacturer: str, serial: str) -> str:
    return f"{INTERFACE_NAME}/{MAJOR_VERSION}/{manufacturer}/{serial}"


def topic(manufacturer: str, serial: str, name: str) -> str:
    return f"{base(manufacturer, serial)}/{name}"
