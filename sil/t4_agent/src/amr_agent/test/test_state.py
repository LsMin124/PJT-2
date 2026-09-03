import json

from amr_agent.vda5050.order import Edge, Node, NodePosition, Order, OrderBook
from amr_agent.vda5050.state import StateBuilder, make_error


def test_state_shape_and_progress():
    b = OrderBook()
    o = Order("o1", 0, (Node("n0", 0, True, NodePosition(0, 0)), Node("n1", 2, True, NodePosition(1, 0))),
              (Edge("e0", 1, True, "n0", "n1"),))
    b.receive(o)
    sb = StateBuilder("santa", "amr01", "wallA")
    s = sb.state(b, (0.0, 0.0, 0.0), driving=True, errors=[make_error("x", "y")])
    json.dumps(s)
    assert s["headerId"] == 1 and s["serialNumber"] == "amr01" and s["version"] == "2.0.0"
    assert s["lastNodeId"] == "n0" and [n["nodeId"] for n in s["nodeStates"]] == ["n1"]
    assert s["agvPosition"]["mapId"] == "wallA" and s["driving"] and s["errors"][0]["errorType"] == "x"
    b.node_reached()
    s2 = sb.state(b, None)
    assert s2["headerId"] == 2 and s2["lastNodeId"] == "n1" and s2["nodeStates"] == [] and "agvPosition" not in s2
    assert sb.connection("ONLINE")["connectionState"] == "ONLINE"
