use glam::Vec2;
use uxsim_rs::handoff::RenderBubble;
use uxsim_rs::network::{Link, RoadNetwork};
use uxsim_rs::simulation::{TrafficSimulator, Vehicle, VehicleState};

fn setup_sim() -> TrafficSimulator {
    let mut net = RoadNetwork::new();
    net.add_node(1, Vec2::new(0.0, 0.0)).unwrap();
    net.add_node(2, Vec2::new(500.0, 0.0)).unwrap();
    net.add_node(3, Vec2::new(1000.0, 0.0)).unwrap();

    net.add_link(Link::standard_urban(12, 1, 2, 500.0, 2)).unwrap();
    net.add_link(Link::standard_urban(23, 2, 3, 500.0, 2)).unwrap();

    let mut sim = TrafficSimulator::new(net, 1.0);
    sim.add_vehicle(Vehicle::new(1, 1, 3, 0.0, vec![12, 23])).unwrap();
    sim
}

#[test]
fn test_render_bubble_query_and_handoff() {
    let mut sim = setup_sim();

    // Player render bubble at (250, 0) with radius 100m
    let bubble = RenderBubble::new(Vec2::new(250.0, 0.0), 100.0, 20.0);

    // Step once to place vehicle at start (0, 0)
    sim.step();
    assert!(sim.query_vehicles_in_bubble(&bubble).is_empty());

    // Advance vehicle to middle of link 12 (around 250m)
    // 250m / 15m/s ~ 16-17s
    sim.run_for(16.0);

    let in_bubble = sim.query_vehicles_in_bubble(&bubble);
    assert_eq!(in_bubble, vec![1]);

    // Perform handoff to microscopic simulation
    let handoff = sim.handoff_to_micro(1).unwrap();
    assert_eq!(handoff.vehicle_id, 1);
    assert_eq!(handoff.link_id, 12);
    assert!(handoff.world_position.x > 200.0 && handoff.world_position.x < 300.0);
    assert_eq!(handoff.forward_direction, Vec2::X);

    // In macroscopic simulator, vehicle state is HandedOffToMicro
    assert_eq!(sim.vehicles.get(&1).unwrap().state, VehicleState::HandedOffToMicro);
    // Link vehicle count should have decreased by 1
    assert_eq!(sim.network.get_link(12).unwrap().vehicle_count, 0.0);

    // Later, vehicle leaves micro bubble near node 2 (e.g. at x = 450m on link 12)
    let mut return_handoff = handoff;
    return_handoff.distance_on_link = 450.0;
    return_handoff.world_position = Vec2::new(450.0, 0.0);

    // Assimilate back into macroscopic simulation
    sim.assimilate_from_micro(return_handoff).unwrap();
    assert_eq!(sim.vehicles.get(&1).unwrap().state, VehicleState::EnRoute);
    assert_eq!(sim.network.get_link(12).unwrap().vehicle_count, 1.0);
    assert_eq!(sim.vehicles.get(&1).unwrap().distance_on_link, 450.0);
}
