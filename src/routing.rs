//! Routing algorithms (Dijkstra and A*) on macroscopic road networks.
//!
//! Computes optimal routes based on dynamic link congestion delays,
//! free-flow travel time, or physical distance.

use std::cmp::Ordering;
use std::collections::{BinaryHeap, HashMap};

use crate::error::{Result, UxsimError};
use crate::network::{Link, LinkId, NodeId, RoadNetwork};

use serde::{Deserialize, Serialize};

/// Cost metric used for route calculations.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
pub enum RoutingCostMetric {
    /// Free-flow travel time: $L / v_f$ (seconds).
    FreeFlowTravelTime,
    /// Dynamic travel time: $L / v(k)$ taking congestion into account (seconds).
    #[default]
    DynamicTravelTime,
    /// Physical link length (meters).
    Distance,
}

impl RoutingCostMetric {
    /// Evaluates the cost of traversing a link according to this metric.
    #[inline]
    pub fn cost(&self, link: &Link) -> f32 {
        match self {
            Self::FreeFlowTravelTime => link.free_flow_travel_time(),
            Self::DynamicTravelTime => link.estimated_travel_time,
            Self::Distance => link.length,
        }
    }
}

/// A computed route through the road network.
#[derive(Debug, Clone, PartialEq, Default, Serialize, Deserialize)]
pub struct Route {
    /// Sequence of nodes from origin to destination.
    pub nodes: Vec<NodeId>,
    /// Sequence of directed links traversed.
    pub links: Vec<LinkId>,
    /// Total evaluated cost according to the metric used.
    pub total_cost: f32,
}

impl Route {
    /// Checks if the route is empty.
    #[inline]
    pub fn is_empty(&self) -> bool {
        self.links.is_empty()
    }

    /// Number of links in the route.
    #[inline]
    pub fn len(&self) -> usize {
        self.links.len()
    }
}

/// Priority queue item for Dijkstra / A*.
#[derive(Copy, Clone, PartialEq)]
struct HeapNode {
    cost: f32,
    node: NodeId,
}

impl Eq for HeapNode {}

impl Ord for HeapNode {
    fn cmp(&self, other: &Self) -> Ordering {
        // Reverse ordering for min-heap
        other.cost.partial_cmp(&self.cost).unwrap_or(Ordering::Equal)
    }
}

impl PartialOrd for HeapNode {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

/// Router engine for pathfinding across the road network.
#[derive(Debug, Clone, Copy, Default, Serialize, Deserialize)]
pub struct Router {
    pub metric: RoutingCostMetric,
}

impl Router {
    /// Creates a router with a specific cost metric.
    pub fn new(metric: RoutingCostMetric) -> Self {
        Self { metric }
    }

    /// Computes the shortest path using Dijkstra's algorithm.
    pub fn dijkstra(&self, network: &RoadNetwork, origin: NodeId, destination: NodeId) -> Result<Route> {
        if !network.nodes().contains_key(&origin) {
            return Err(UxsimError::NodeNotFound(origin));
        }
        if !network.nodes().contains_key(&destination) {
            return Err(UxsimError::NodeNotFound(destination));
        }

        if origin == destination {
            return Ok(Route {
                nodes: vec![origin],
                links: Vec::new(),
                total_cost: 0.0,
            });
        }

        let mut distances: HashMap<NodeId, f32> = HashMap::new();
        let mut predecessors: HashMap<NodeId, (NodeId, LinkId)> = HashMap::new();
        let mut heap = BinaryHeap::new();

        distances.insert(origin, 0.0);
        heap.push(HeapNode {
            cost: 0.0,
            node: origin,
        });

        while let Some(HeapNode { cost, node }) = heap.pop() {
            if node == destination {
                return Ok(reconstruct_route(origin, destination, &predecessors, cost));
            }

            if let Some(&best) = distances.get(&node)
                && cost > best {
                    continue;
                }

            let current_node = match network.get_node(node) {
                Some(n) => n,
                None => continue,
            };

            for &link_id in &current_node.outgoing_links {
                let link = match network.get_link(link_id) {
                    Some(l) => l,
                    None => continue,
                };

                let step_cost = self.metric.cost(link);
                let next_cost = cost + step_cost;
                let next_node = link.to_node;

                let is_better = match distances.get(&next_node) {
                    Some(&prev_cost) => next_cost < prev_cost,
                    None => true,
                };

                if is_better {
                    distances.insert(next_node, next_cost);
                    predecessors.insert(next_node, (node, link_id));
                    heap.push(HeapNode {
                        cost: next_cost,
                        node: next_node,
                    });
                }
            }
        }

        Err(UxsimError::NoPathFound(origin, destination))
    }

    /// Computes the shortest path using A* search with Euclidean distance heuristic.
    pub fn a_star(&self, network: &RoadNetwork, origin: NodeId, destination: NodeId) -> Result<Route> {
        if !network.nodes().contains_key(&origin) {
            return Err(UxsimError::NodeNotFound(origin));
        }
        let dest_node = network
            .get_node(destination)
            .ok_or(UxsimError::NodeNotFound(destination))?;

        if origin == destination {
            return Ok(Route {
                nodes: vec![origin],
                links: Vec::new(),
                total_cost: 0.0,
            });
        }

        // Speed used for admissible heuristic travel time estimation
        let max_speed = network
            .links()
            .values()
            .map(|l| l.free_flow_speed())
            .fold(1.0f32, f32::max);

        let dest_pos = dest_node.position;

        let heuristic = |node_id: NodeId| -> f32 {
            if let Some(node) = network.get_node(node_id) {
                let dist = node.position.distance(dest_pos);
                match self.metric {
                    RoutingCostMetric::Distance => dist,
                    RoutingCostMetric::FreeFlowTravelTime | RoutingCostMetric::DynamicTravelTime => {
                        dist / max_speed
                    }
                }
            } else {
                0.0
            }
        };

        let mut g_score: HashMap<NodeId, f32> = HashMap::new();
        let mut predecessors: HashMap<NodeId, (NodeId, LinkId)> = HashMap::new();
        let mut heap = BinaryHeap::new();

        g_score.insert(origin, 0.0);
        heap.push(HeapNode {
            cost: heuristic(origin),
            node: origin,
        });

        while let Some(HeapNode { node, .. }) = heap.pop() {
            if node == destination {
                let total_cost = *g_score.get(&destination).unwrap_or(&0.0);
                return Ok(reconstruct_route(origin, destination, &predecessors, total_cost));
            }

            let current_g = match g_score.get(&node) {
                Some(&g) => g,
                None => continue,
            };

            let current_node = match network.get_node(node) {
                Some(n) => n,
                None => continue,
            };

            for &link_id in &current_node.outgoing_links {
                let link = match network.get_link(link_id) {
                    Some(l) => l,
                    None => continue,
                };

                let step_cost = self.metric.cost(link);
                let tentative_g = current_g + step_cost;
                let next_node = link.to_node;

                let is_better = match g_score.get(&next_node) {
                    Some(&prev_g) => tentative_g < prev_g,
                    None => true,
                };

                if is_better {
                    g_score.insert(next_node, tentative_g);
                    predecessors.insert(next_node, (node, link_id));
                    let f_score = tentative_g + heuristic(next_node);
                    heap.push(HeapNode {
                        cost: f_score,
                        node: next_node,
                    });
                }
            }
        }

        Err(UxsimError::NoPathFound(origin, destination))
    }
}

fn reconstruct_route(
    origin: NodeId,
    destination: NodeId,
    predecessors: &HashMap<NodeId, (NodeId, LinkId)>,
    total_cost: f32,
) -> Route {
    let mut nodes = Vec::new();
    let mut links = Vec::new();

    let mut curr = destination;
    nodes.push(curr);

    while curr != origin {
        if let Some(&(prev_node, link_id)) = predecessors.get(&curr) {
            links.push(link_id);
            nodes.push(prev_node);
            curr = prev_node;
        } else {
            break;
        }
    }

    nodes.reverse();
    links.reverse();

    Route {
        nodes,
        links,
        total_cost,
    }
}
