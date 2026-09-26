//! Background city simulation and microscopic handoff system.
//!
//! Provides the boundary abstraction (`RenderBubble`) for seamless handoff between
//! lightweight macroscopic network flow outside the player bubble and microscopic
//! physics / agent steering (e.g. `sumo-rs`, Bevy ECS) within the player's render range.

use glam::Vec2;
use serde::{Deserialize, Serialize};

use crate::network::LinkId;

/// Represents a player or camera render bubble in the open world.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct RenderBubble {
    /// World position of the player/camera center.
    pub center: Vec2,
    /// Radius in meters around the center where microscopic simulation is active.
    pub radius: f32,
    /// Margin / hysteresis buffer to avoid rapid toggling at the boundary.
    pub margin: f32,
}

impl RenderBubble {
    /// Creates a new render bubble.
    pub fn new(center: Vec2, radius: f32, margin: f32) -> Self {
        Self {
            center,
            radius,
            margin,
        }
    }

    /// Checks if a world position is strictly inside the render bubble.
    #[inline]
    pub fn contains(&self, position: Vec2) -> bool {
        self.center.distance_squared(position) <= self.radius * self.radius
    }

    /// Checks if a world position is outside the bubble plus hysteresis margin.
    #[inline]
    pub fn outside_margin(&self, position: Vec2) -> bool {
        let r = self.radius + self.margin;
        self.center.distance_squared(position) > r * r
    }
}

/// Data package transferred when handing off a vehicle from macroscopic to microscopic simulation,
/// or assimilating back from microscopic to macroscopic.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct MicroVehicleHandoff {
    /// Unique vehicle identifier.
    pub vehicle_id: u64,
    /// Origin node ID.
    pub origin_node: u64,
    /// Ultimate destination node ID.
    pub destination_node: u64,
    /// Current link ID the vehicle is on.
    pub link_id: LinkId,
    /// Remaining link IDs on the vehicle's assigned route.
    pub remaining_links: Vec<LinkId>,
    /// Distance traveled along the current link in meters ($0 \le x \le L$).
    pub distance_on_link: f32,
    /// Current 2D world position.
    pub world_position: Vec2,
    /// Current 2D forward heading unit vector.
    pub forward_direction: Vec2,
    /// Current vehicle speed (m/s).
    pub speed: f32,
    /// Assigned lane index (0-indexed).
    pub lane: u32,
    /// Timestamp when handoff occurred.
    pub handoff_timestamp: f32,
}
