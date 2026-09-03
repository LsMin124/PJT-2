import math

import pytest

from amr_agent.vda5050.order import Edge, Node, NodePosition, Order, OrderBook, OrderError


def mk(order_id, upd, spec, reverse=(), seq0=0):
    """spec: list of (node_id, released, x, y, theta). Edges auto-generated; sequenceIds start at seq0."""
    nodes = tuple(Node(nid, seq0 + 2 * i, rel, NodePosition(x, y, th)) for i, (nid, rel, x, y, th) in enumerate(spec))
    edges = tuple(Edge(f"e{i}", seq0 + 2 * i + 1, nodes[i + 1].released, nodes[i].node_id, nodes[i + 1].node_id,
                       orientation=math.pi if i in reverse else None)
                  for i in range(len(nodes) - 1))
    return Order(order_id, upd, nodes, edges)


def test_parse_and_validate_roundtrip():
    o = mk("o1", 0, [("n0", True, 0, 0, 0), ("n1", True, 1, 0, 0), ("n2", False, 2, 0, None)])
    d = o.to_dict("santa", "amr01", 1, "t")
    o2 = Order.from_dict(d)
    assert o2.base_len() == 2 and len(o2.nodes) == 3 and o2.nodes[2].position.theta is None


@pytest.mark.parametrize("bad", [
    {"orderId": "", "orderUpdateId": 0, "nodes": [], "edges": []},
    {"orderId": "o", "orderUpdateId": 0, "nodes": [{"nodeId": "a", "sequenceId": 1, "released": True,
                                                    "nodePosition": {"x": 0, "y": 0}}], "edges": []},
    {"orderId": "o", "orderUpdateId": 0,
     "nodes": [{"nodeId": "a", "sequenceId": 0, "released": False, "nodePosition": {"x": 0, "y": 0}},
               {"nodeId": "b", "sequenceId": 2, "released": True, "nodePosition": {"x": 1, "y": 0}}],
     "edges": [{"edgeId": "e", "sequenceId": 1, "released": True, "startNodeId": "a", "endNodeId": "b"}]},
])
def test_reject_malformed(bad):
    with pytest.raises(OrderError):
        Order.from_dict(bad)


def test_start_and_traverse():
    b = OrderBook()
    o = mk("o1", 0, [("n0", True, 0, 0, 0), ("n1", True, 1, 0, 0), ("n2", False, 2, 0, 0)])
    assert b.receive(o, at_xy=(0.02, 0.0)) == "started"
    assert b.last_node.node_id == "n0" and b.active
    edge, node = b.next_target()
    assert edge.edge_id == "e0" and node.node_id == "n1"
    assert [s["nodeId"] for s in b.node_states()] == ["n1", "n2"]
    b.node_reached()
    assert b.last_node.node_id == "n1" and not b.active   # n2 is horizon
    assert b.next_target() is None


def test_first_node_too_far():
    b = OrderBook()
    with pytest.raises(OrderError):
        b.receive(mk("o1", 0, [("n0", True, 0, 0, 0), ("n1", True, 1, 0, 0)]), at_xy=(3.0, 0.0))
    assert b.order is None


def test_stitch_one_node_per_tick():
    b = OrderBook()
    plan = [("n0", True, 0, 0, 0), ("n1", False, 1, 0, 0), ("n2", False, 2, 0, 0), ("n3", False, 2, 0, math.pi / 2)]
    b.receive(mk("o1", 0, plan))
    assert not b.active
    for k in range(1, 4):
        spec = [(nid, i <= k, x, y, th) for i, (nid, _, x, y, th) in enumerate(plan)]
        assert b.receive(mk("o1", k, spec[k - 1:], seq0=2 * (k - 1))) == "stitched"   # starts at last base node
        assert b.active and b.next_target()[1].node_id == f"n{k}"
        b.node_reached()
    assert b.last_node.node_id == "n3" and b.node_states() == []


def test_stitch_rejections():
    b = OrderBook()
    b.receive(mk("o1", 0, [("n0", True, 0, 0, 0), ("n1", True, 1, 0, 0), ("n2", False, 2, 0, 0)]))
    with pytest.raises(OrderError):   # same updateId
        b.receive(mk("o1", 0, [("n1", True, 1, 0, 0), ("n2", True, 2, 0, 0)]))
    with pytest.raises(OrderError):   # does not start at last base node (n1)
        b.receive(mk("o1", 1, [("n0", True, 0, 0, 0), ("n2", True, 2, 0, 0)]))
    with pytest.raises(OrderError):   # different order while base active
        b.receive(mk("o2", 0, [("m0", True, 1, 0, 0), ("m1", True, 2, 0, 0)]))
    # stitched update must carry matching sequenceIds
    good = Order("o1", 1, (Node("n1", 2, True, NodePosition(1, 0, 0)), Node("n2", 4, True, NodePosition(2, 0, 0))),
                 (Edge("e1", 3, True, "n1", "n2"),))
    assert b.receive(good) == "stitched"
    assert b.order.base_len() == 3


def test_cancel_then_new_order():
    b = OrderBook()
    b.receive(mk("o1", 0, [("n0", True, 0, 0, 0), ("n1", True, 1, 0, 0), ("n2", True, 2, 0, 0)]))
    b.node_reached()
    b.cancel()
    assert not b.active and b.last_node.node_id == "n1" and b.node_states() == []
    assert b.receive(mk("o2", 0, [("m0", True, 1, 0, 0), ("m1", True, 2, 0, 0)]), at_xy=(1.0, 0.0)) == "started"


def test_reverse_edge_detection():
    o = mk("o1", 0, [("n0", True, 0, 0, 0), ("n1", True, -1, 0, 0)], reverse={0})
    assert o.edges[0].is_reverse()
    assert not Edge("e", 1, True, "a", "b", orientation=0.0).is_reverse()
    assert not Edge("e", 1, True, "a", "b", orientation=math.pi, orientation_type="GLOBAL").is_reverse()
