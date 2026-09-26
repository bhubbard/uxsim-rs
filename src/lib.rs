//! # uxsim-rs
//!
//! A pure Rust macroscopic city-wide traffic network flow simulator inspired by UXsim.
//! Designed for open-world games and large-scale urban simulation (e.g. in Bevy).
//!
//! ## Core Features
//!
//! - **Fundamental Diagrams**: Greenshields linear model, Newell triangular diagram with backward shockwave speed, and trapezoidal diagrams.
//! - **Dynamic Network Flow**: Node-link propagation, queue spillback, bottleneck outflow, and signal states.
//! - **Routing**: Dijkstra and A* pathfinding using dynamic travel time, free-flow time, or physical distance.
//! - **Microscopic Handoff**: `RenderBubble` boundary system enabling seamless transition of vehicles between inexpensive background macroscopic simulation and microscopic agents (e.g., `sumo-rs` or local physics) inside the player's render bubble.

pub mod error;
pub mod flow_model;
pub mod handoff;
pub mod network;
pub mod routing;
pub mod simulation;

// Re-exports for convenience
pub use error::{Result, UxsimError};
pub use flow_model::{
    FlowModel, FundamentalDiagram, Greenshields, NewellTriangular, TrapezoidalDiagram,
};
pub use handoff::{MicroVehicleHandoff, RenderBubble};
pub use network::{Link, LinkId, Node, NodeId, RoadNetwork, SignalState};
pub use routing::{Route, Router, RoutingCostMetric};
pub use simulation::{SimulationStats, TrafficSimulator, Vehicle, VehicleState};
