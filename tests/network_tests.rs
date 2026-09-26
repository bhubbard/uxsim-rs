use glam::Vec2;
use uxsim_rs::network::{Link, Node, RoadNetwork, SignalState};
use uxsim_rs::flow_model::{FlowModel, NewellTriangular};

#[test]
fn test_network_creation_and_topology() {
    let mut net = RoadNetwork::new();

    // Add nodes
    net.add_node(1, Vec2::new(0.0, 0.0)).unwrap();
    net.add_node(2, Vec2::new(100.0, 0.0)).unwrap();
    net.add_node(3, Vec2::new(200.0, 0.0)).unwrap();

    // Cannot add duplicate node
    assert!(net.add_node(1, Vec2::new(0.0, 0.0)).is_err());

    // Add links
    let model = FlowModel::NewellTriangular(NewellTriangular::new(15.0, 5.0, 0.20));
    let link1 = Link::new(101, 1, 2, 100.0, 2, model);
    net.add_link(link1).unwrap();

    // Cannot add duplicate link
    let link_dup = Link::new(101, 1, 2, 100.0, 2, model);
    assert!(net.add_link(link_dup).is_err());

    // Add bidirectional link between 2 and 3
    net.add_bidirectional_link(102, 103, 2, 3, 100.0, 2, model).unwrap();

    // Verify adjacency
    let node1 = net.get_node(1).unwrap();
    assert_eq!(node1.outgoing_links, vec![101]);
    assert!(node1.incoming_links.is_empty());

    let node2 = net.get_node(2).unwrap();
    assert_eq!(node2.incoming_links, vec![101, 103]);
    assert_eq!(node2.outgoing_links, vec![102]);

    // Check link storage capacity and free flow travel time
    let l101 = net.get_link(101).unwrap();
    // Storage capacity = kj * length * lanes = 0.20 * 100 * 2 = 40 vehicles
    assert!((l101.storage_capacity() - 40.0).abs() < 1e-4);
    // Free flow travel time = 100 / 15 = 6.6667 s
    assert!((l101.free_flow_travel_time() - (100.0 / 15.0)).abs() < 1e-4);
    assert_eq!(l101.available_space(), 40.0);
}

#[test]
fn test_link_dynamics_update() {
    let mut net = RoadNetwork::new();
    net.add_node(1, Vec2::new(0.0, 0.0)).unwrap();
    net.add_node(2, Vec2::new(1000.0, 0.0)).unwrap();

    let model = FlowModel::NewellTriangular(NewellTriangular::new(20.0, 5.0, 0.20));
    let link = Link::new(1, 1, 2, 1000.0, 1, model);
    net.add_link(link).unwrap();

    let l = net.get_link_mut(1).unwrap();
    assert_eq!(l.density(), 0.0);
    assert!((l.current_speed - 20.0).abs() < 1e-4);

    // Place 100 vehicles on the 1000m 1-lane link => density = 0.1 veh/m
    l.vehicle_count = 100.0;
    assert!((l.density() - 0.10).abs() < 1e-4);

    l.update_dynamics();
    // Critical density kc = 5 * 0.2 / (20 + 5) = 0.04
    // Since k = 0.10 > kc, congested: q = 5 * (0.2 - 0.1) = 0.50 veh/s
    // speed = q / k = 0.50 / 0.10 = 5.0 m/s
    assert!((l.current_speed - 5.0).abs() < 1e-4);
    // Travel time = 1000m / 5m/s = 200 s
    assert!((l.estimated_travel_time - 200.0).abs() < 1e-4);
}

#[test]
fn test_node_signals() {
    let mut node = Node::new(1, Vec2::ZERO);
    assert_eq!(node.signal_state, SignalState::Uncontrolled);
    node.signal_state = SignalState::Red;
    assert_eq!(node.signal_state, SignalState::Red);
}
