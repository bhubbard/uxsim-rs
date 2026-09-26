//! Road network definitions: nodes (intersections) and links (road segments).

use std::collections::HashMap;
use glam::Vec2;
use serde::{Deserialize, Serialize};

use crate::error::{Result, UxsimError};
use crate::flow_model::{FlowModel, FundamentalDiagram, NewellTriangular};

/// Unique identifier for a road network node (intersection).
pub type NodeId = u64;

/// Unique identifier for a road network link (road segment).
pub type LinkId = u64;

/// Signal state for controlled intersections.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[derive(Default)]
pub enum SignalState {
    /// Uncontrolled / priority intersection.
    #[default]
    Uncontrolled,
    /// Green light / permitted flow.
    Green,
    /// Red light / stopped flow.
    Red,
    /// Yellow / clearance interval.
    Yellow,
}


/// A node in the macroscopic road network, typically representing an intersection.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Node {
    /// Unique identifier for this node.
    pub id: NodeId,
    /// 2D world position (e.g. game world coordinates in meters).
    pub position: Vec2,
    /// Incoming link IDs terminating at this node.
    pub incoming_links: Vec<LinkId>,
    /// Outgoing link IDs originating from this node.
    pub outgoing_links: Vec<LinkId>,
    /// Current signal state for incoming traffic control.
    pub signal_state: SignalState,
}

impl Node {
    /// Creates a new network node at the given 2D coordinates.
    pub fn new(id: NodeId, position: Vec2) -> Self {
        Self {
            id,
            position,
            incoming_links: Vec::new(),
            outgoing_links: Vec::new(),
            signal_state: SignalState::Uncontrolled,
        }
    }
}

/// A directed link (road segment) connecting two nodes in the road network.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Link {
    /// Unique identifier for this link.
    pub id: LinkId,
    /// Origin node ID.
    pub from_node: NodeId,
    /// Destination node ID.
    pub to_node: NodeId,
    /// Length of the link in meters.
    pub length: f32,
    /// Number of lanes on this link.
    pub num_lanes: u32,
    /// Fundamental diagram model governing traffic dynamics on this link.
    pub flow_model: FlowModel,
    /// Number of vehicles currently traveling or queued on this link.
    pub vehicle_count: f32,
    /// Dynamic link flow speed (m/s).
    pub current_speed: f32,
    /// Dynamic estimated travel time through this link (seconds).
    pub estimated_travel_time: f32,
    /// Cumulative inflow count or rate (veh/s).
    pub inflow_rate: f32,
    /// Cumulative outflow count or rate (veh/s).
    pub outflow_rate: f32,
}

impl Link {
    /// Creates a new link between two nodes.
    pub fn new(
        id: LinkId,
        from_node: NodeId,
        to_node: NodeId,
        length: f32,
        num_lanes: u32,
        flow_model: FlowModel,
    ) -> Self {
        assert!(length > 0.0, "Link length must be positive");
        assert!(num_lanes > 0, "Number of lanes must be at least 1");
        let free_flow_speed = flow_model.free_flow_speed();
        let free_flow_tt = length / free_flow_speed;

        Self {
            id,
            from_node,
            to_node,
            length,
            num_lanes,
            flow_model,
            vehicle_count: 0.0,
            current_speed: free_flow_speed,
            estimated_travel_time: free_flow_tt,
            inflow_rate: 0.0,
            outflow_rate: 0.0,
        }
    }

    /// Convenience constructor with standard urban Newell triangular model.
    pub fn standard_urban(
        id: LinkId,
        from_node: NodeId,
        to_node: NodeId,
        length: f32,
        num_lanes: u32,
    ) -> Self {
        Self::new(
            id,
            from_node,
            to_node,
            length,
            num_lanes,
            FlowModel::NewellTriangular(NewellTriangular::standard_urban()),
        )
    }

    /// Free-flow travel time $\tau = L / v_f$ in seconds.
    #[inline]
    pub fn free_flow_travel_time(&self) -> f32 {
        self.length / self.flow_model.free_flow_speed()
    }

    /// Free-flow speed $v_f$ in m/s.
    #[inline]
    pub fn free_flow_speed(&self) -> f32 {
        self.flow_model.free_flow_speed()
    }

    /// Total storage capacity $K = k_j \cdot L \cdot \text{lanes}$ (maximum vehicles the link can physically hold).
    #[inline]
    pub fn storage_capacity(&self) -> f32 {
        self.flow_model.jam_density() * self.length * (self.num_lanes as f32)
    }

    /// Link-wide flow capacity $Q_{\max} = q_{\max} \cdot \text{lanes}$ in veh/s.
    #[inline]
    pub fn total_capacity(&self) -> f32 {
        self.flow_model.capacity() * (self.num_lanes as f32)
    }

    /// Current traffic density $k$ in veh/m/lane.
    #[inline]
    pub fn density(&self) -> f32 {
        if self.length <= 0.0 || self.num_lanes == 0 {
            0.0
        } else {
            self.vehicle_count / (self.length * (self.num_lanes as f32))
        }
    }

    /// Updates link dynamic states (speed, travel time) from current density.
    pub fn update_dynamics(&mut self) {
        let k = self.density();
        self.current_speed = self.flow_model.speed(k).max(0.1); // Avoid 0 div
        // Travel time based on current speed
        let speed_tt = self.length / self.current_speed;
        self.estimated_travel_time = speed_tt.max(self.free_flow_travel_time());
    }

    /// Available storage space in vehicles before jam density is reached.
    #[inline]
    pub fn available_space(&self) -> f32 {
        (self.storage_capacity() - self.vehicle_count).max(0.0)
    }

    /// Sending capacity (vehicles ready to leave downstream in time dt).
    #[inline]
    pub fn sending_flow(&self) -> f32 {
        let k = self.density();
        self.flow_model.demand(k) * (self.num_lanes as f32)
    }

    /// Receiving capacity (vehicles link can accept upstream in time dt).
    #[inline]
    pub fn receiving_flow(&self) -> f32 {
        let k = self.density();
        self.flow_model.supply(k) * (self.num_lanes as f32)
    }
}

/// Road network containing nodes and directed links.
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct RoadNetwork {
    nodes: HashMap<NodeId, Node>,
    links: HashMap<LinkId, Link>,
}

impl RoadNetwork {
    /// Creates an empty road network.
    pub fn new() -> Self {
        Self {
            nodes: HashMap::new(),
            links: HashMap::new(),
        }
    }

    /// Adds a node to the network.
    pub fn add_node(&mut self, id: NodeId, position: Vec2) -> Result<()> {
        if self.nodes.contains_key(&id) {
            return Err(UxsimError::DuplicateNode(id));
        }
        self.nodes.insert(id, Node::new(id, position));
        Ok(())
    }

    /// Adds a directed link between two existing nodes.
    pub fn add_link(&mut self, link: Link) -> Result<()> {
        if self.links.contains_key(&link.id) {
            return Err(UxsimError::DuplicateLink(link.id));
        }
        if !self.nodes.contains_key(&link.from_node) {
            return Err(UxsimError::NodeNotFound(link.from_node));
        }
        if !self.nodes.contains_key(&link.to_node) {
            return Err(UxsimError::NodeNotFound(link.to_node));
        }

        let link_id = link.id;
        let from = link.from_node;
        let to = link.to_node;

        self.links.insert(link_id, link);

        if let Some(from_node) = self.nodes.get_mut(&from) {
            from_node.outgoing_links.push(link_id);
        }
        if let Some(to_node) = self.nodes.get_mut(&to) {
            to_node.incoming_links.push(link_id);
        }

        Ok(())
    }

    /// Adds a two-way street between two nodes, creating two separate directed links.
    #[allow(clippy::too_many_arguments)]
    pub fn add_bidirectional_link(
        &mut self,
        link_id_fwd: LinkId,
        link_id_rev: LinkId,
        node_a: NodeId,
        node_b: NodeId,
        length: f32,
        num_lanes_per_dir: u32,
        flow_model: FlowModel,
    ) -> Result<()> {
        self.add_link(Link::new(
            link_id_fwd,
            node_a,
            node_b,
            length,
            num_lanes_per_dir,
            flow_model,
        ))?;
        self.add_link(Link::new(
            link_id_rev,
            node_b,
            node_a,
            length,
            num_lanes_per_dir,
            flow_model,
        ))?;
        Ok(())
    }

    /// Returns a reference to a node.
    pub fn get_node(&self, id: NodeId) -> Option<&Node> {
        self.nodes.get(&id)
    }

    /// Returns a mutable reference to a node.
    pub fn get_node_mut(&mut self, id: NodeId) -> Option<&mut Node> {
        self.nodes.get_mut(&id)
    }

    /// Returns a reference to a link.
    pub fn get_link(&self, id: LinkId) -> Option<&Link> {
        self.links.get(&id)
    }

    /// Returns a mutable reference to a link.
    pub fn get_link_mut(&mut self, id: LinkId) -> Option<&mut Link> {
        self.links.get_mut(&id)
    }

    /// Returns all nodes in the network.
    pub fn nodes(&self) -> &HashMap<NodeId, Node> {
        &self.nodes
    }

    /// Returns all links in the network.
    pub fn links(&self) -> &HashMap<LinkId, Link> {
        &self.links
    }

    /// Returns mutable map of all links in the network.
    pub fn links_mut(&mut self) -> &mut HashMap<LinkId, Link> {
        &mut self.links
    }

    /// Computes straight-line Euclidean distance between two nodes if both exist.
    pub fn euclidean_distance(&self, from: NodeId, to: NodeId) -> Option<f32> {
        let n1 = self.nodes.get(&from)?;
        let n2 = self.nodes.get(&to)?;
        Some(n1.position.distance(n2.position))
    }

    /// Updates dynamic conditions on all links based on their current vehicle densities.
    pub fn update_all_link_dynamics(&mut self) {
        for link in self.links.values_mut() {
            link.update_dynamics();
        }
    }
}
