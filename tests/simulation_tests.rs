use glam::Vec2;
use uxsim_rs::network::{Link, RoadNetwork, SignalState};
use uxsim_rs::simulation::{TrafficSimulator, Vehicle, VehicleState};

fn create_corridor_network() -> RoadNetwork {
    // 1 ---> 2 ---> 3
    let mut net = RoadNetwork::new();
    net.add_node(1, Vec2::new(0.0, 0.0)).unwrap();
    net.add_node(2, Vec2::new(300.0, 0.0)).unwrap();
    net.add_node(3, Vec2::new(600.0, 0.0)).unwrap();

    net.add_link(Link::standard_urban(12, 1, 2, 300.0, 1)).unwrap();
    net.add_link(Link::standard_urban(23, 2, 3, 300.0, 1)).unwrap();

    net
}

#[test]
fn test_vehicle_full_trip_propagation() {
    let net = create_corridor_network();
    let mut sim = TrafficSimulator::new(net, 1.0);

    // Add vehicle departing at t = 0 from node 1 to node 3
    let veh = Vehicle::new(1, 1, 3, 0.0, vec![12, 23]);
    sim.add_vehicle(veh).unwrap();

    // At t = 0, state is WaitingToDepart
    assert_eq!(sim.vehicles.get(&1).unwrap().state, VehicleState::WaitingToDepart);

    // Step 1: departs onto link 12
    sim.step();
    assert_eq!(sim.vehicles.get(&1).unwrap().state, VehicleState::EnRoute);
    assert_eq!(sim.vehicles.get(&1).unwrap().current_link(), Some(12));

    // Travel along link 12: length = 300m, vf = 15 m/s => takes ~20 seconds
    sim.run_for(22.0);

    // Vehicle should transition through node 2 to link 23 or arrive
    sim.run_for(25.0);

    let v = sim.vehicles.get(&1).unwrap();
    assert_eq!(v.state, VehicleState::Arrived);
    assert!(v.arrival_time.is_some());
    assert!(v.arrival_time.unwrap() > 30.0);

    let stats = sim.stats();
    assert_eq!(stats.completed_vehicles, 1);
    assert_eq!(stats.active_vehicles, 0);
}

#[test]
fn test_flow_packet_simulation() {
    let net = create_corridor_network();
    let mut sim = TrafficSimulator::new(net, 1.0);

    // Single vehicle entity representing a platoon of 10 cars
    let platoon = Vehicle::new(100, 1, 3, 0.0, vec![12, 23]).with_packet_size(10);
    sim.add_vehicle(platoon).unwrap();

    sim.step();
    // Link 12 should now reflect 10 vehicles
    assert_eq!(sim.network.get_link(12).unwrap().vehicle_count, 10.0);
}

#[test]
fn test_traffic_signal_queue_blocking() {
    let mut net = create_corridor_network();
    // Turn node 2 red
    if let Some(n) = net.get_node_mut(2) {
        n.signal_state = SignalState::Red;
    }

    let mut sim = TrafficSimulator::new(net, 1.0);
    let veh = Vehicle::new(1, 1, 3, 0.0, vec![12, 23]);
    sim.add_vehicle(veh).unwrap();

    // Run until vehicle reaches end of link 12
    sim.run_for(25.0);

    // Because node 2 is red, vehicle cannot enter link 23 and must be queued at intersection
    let v = sim.vehicles.get(&1).unwrap();
    assert_eq!(v.state, VehicleState::QueuedAtIntersection);
    assert_eq!(v.current_link(), Some(12));

    // Turn node 2 green!
    if let Some(n) = sim.network.get_node_mut(2) {
        n.signal_state = SignalState::Green;
    }

    // Step simulation
    sim.step();

    // Vehicle moves onto link 23
    let v2 = sim.vehicles.get(&1).unwrap();
    assert_eq!(v2.state, VehicleState::EnRoute);
    assert_eq!(v2.current_link(), Some(23));
}

#[test]
fn test_bottleneck_and_spillback() {
    let mut net = RoadNetwork::new();
    net.add_node(1, Vec2::new(0.0, 0.0)).unwrap();
    net.add_node(2, Vec2::new(200.0, 0.0)).unwrap();
    net.add_node(3, Vec2::new(300.0, 0.0)).unwrap(); // short downstream link

    // Link 12: 200m, 2 lanes
    net.add_link(Link::standard_urban(12, 1, 2, 200.0, 2)).unwrap();
    // Link 23: tiny bottleneck link: length 10m, 1 lane, jam density 0.14 => storage capacity = 1.4 vehicles
    net.add_link(Link::standard_urban(23, 2, 3, 10.0, 1)).unwrap();

    let mut sim = TrafficSimulator::new(net, 1.0);

    // Send 5 vehicles departing at t = 0
    for id in 1..=5 {
        sim.add_vehicle(Vehicle::new(id, 1, 3, 0.0, vec![12, 23])).unwrap();
    }

    // Step simulation until vehicles reach node 2
    sim.run_for(20.0);

    // Downstream link 23 has max storage capacity ~ 1 vehicle.
    // So link 23 cannot fit all 5 vehicles at once; spillback forces remaining vehicles to queue on link 12.
    let queued_on_12 = sim
        .vehicles
        .values()
        .filter(|v| v.state == VehicleState::QueuedAtIntersection && v.current_link() == Some(12))
        .count();
    let on_23_or_arrived = sim
        .vehicles
        .values()
        .filter(|v| v.current_link() == Some(23) || v.state == VehicleState::Arrived)
        .count();

    assert!(queued_on_12 > 0, "Spillback should keep vehicles queued on upstream link 12");
    assert_eq!(queued_on_12 + on_23_or_arrived, 5);
}

