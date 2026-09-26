use glam::Vec2;
use uxsim_rs::network::{Link, RoadNetwork};
use uxsim_rs::routing::{Router, RoutingCostMetric};

fn build_diamond_network() -> RoadNetwork {
    // 1 ---> 2 ---> 4
    //  \           ^
    //   \---> 3 --/
    // Link 1: 1->2 (short, length 500m)
    // Link 2: 2->4 (short, length 500m) => total path 1->2->4 = 1000m
    // Link 3: 1->3 (long, length 800m)
    // Link 4: 3->4 (long, length 800m) => total path 1->3->4 = 1600m
    let mut net = RoadNetwork::new();
    net.add_node(1, Vec2::new(0.0, 0.0)).unwrap();
    net.add_node(2, Vec2::new(500.0, 200.0)).unwrap();
    net.add_node(3, Vec2::new(500.0, -400.0)).unwrap();
    net.add_node(4, Vec2::new(1000.0, 0.0)).unwrap();

    net.add_link(Link::standard_urban(12, 1, 2, 500.0, 2)).unwrap();
    net.add_link(Link::standard_urban(24, 2, 4, 500.0, 2)).unwrap();
    net.add_link(Link::standard_urban(13, 1, 3, 800.0, 2)).unwrap();
    net.add_link(Link::standard_urban(34, 3, 4, 800.0, 2)).unwrap();

    net
}

#[test]
fn test_dijkstra_shortest_path() {
    let net = build_diamond_network();
    let router = Router::new(RoutingCostMetric::Distance);

    let route = router.dijkstra(&net, 1, 4).unwrap();
    assert_eq!(route.nodes, vec![1, 2, 4]);
    assert_eq!(route.links, vec![12, 24]);
    assert_eq!(route.total_cost, 1000.0);
}

#[test]
fn test_a_star_shortest_path() {
    let net = build_diamond_network();
    let router = Router::new(RoutingCostMetric::Distance);

    let route = router.a_star(&net, 1, 4).unwrap();
    assert_eq!(route.nodes, vec![1, 2, 4]);
    assert_eq!(route.links, vec![12, 24]);
    assert_eq!(route.total_cost, 1000.0);
}

#[test]
fn test_dynamic_rerouting_around_congestion() {
    let mut net = build_diamond_network();
    let router = Router::new(RoutingCostMetric::DynamicTravelTime);

    // Initially, path 1->2->4 is faster (1000m at ~15 m/s = 66.6s vs 1600m at ~15 m/s = 106.6s)
    let route_init = router.dijkstra(&net, 1, 4).unwrap();
    assert_eq!(route_init.links, vec![12, 24]);

    // Heavily congest link 12 (set travel time artificially to 300 seconds)
    if let Some(l) = net.get_link_mut(12) {
        l.estimated_travel_time = 300.0;
    }

    // Now path 1->3->4 should be chosen!
    let route_congested = router.dijkstra(&net, 1, 4).unwrap();
    assert_eq!(route_congested.nodes, vec![1, 3, 4]);
    assert_eq!(route_congested.links, vec![13, 34]);
}

#[test]
fn test_no_path_found() {
    let mut net = RoadNetwork::new();
    net.add_node(1, Vec2::ZERO).unwrap();
    net.add_node(2, Vec2::X).unwrap();
    // No links added

    let router = Router::default();
    assert!(router.dijkstra(&net, 1, 2).is_err());
    assert!(router.a_star(&net, 1, 2).is_err());
}
