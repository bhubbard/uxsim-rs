//! Macroscopic traffic simulation engine and vehicle flow propagation.
//!
//! Models thousands of lightweight vehicles / flow packets traversing
//! the road network links with dynamic congestion, queue spillback,
//! and seamless handoff to microscopic Bevy/sumo systems.

use std::collections::{HashMap, VecDeque};
use glam::Vec2;
use serde::{Deserialize, Serialize};

use crate::error::{Result, UxsimError};
use crate::handoff::{MicroVehicleHandoff, RenderBubble};
use crate::network::{LinkId, NodeId, RoadNetwork, SignalState};
use crate::routing::{Router, RoutingCostMetric};

/// State of an individual vehicle or flow packet in the simulation.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum VehicleState {
    /// Queued awaiting departure time.
    WaitingToDepart,
    /// Actively traveling along a link at macroscopic flow speed.
    EnRoute,
    /// Queued at the downstream intersection of a link awaiting entry to next link.
    QueuedAtIntersection,
    /// Handed off to the microscopic render bubble (controlled by local physics / Bevy ECS).
    HandedOffToMicro,
    /// Arrived at final destination and completed trip.
    Arrived,
}

/// A vehicle or aggregated flow packet traversing the macroscopic network.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Vehicle {
    /// Unique identifier for this vehicle / packet.
    pub id: u64,
    /// Origin node ID.
    pub origin: NodeId,
    /// Destination node ID.
    pub destination: NodeId,
    /// Scheduled departure time in seconds.
    pub departure_time: f32,
    /// Actual arrival time in seconds (if arrived).
    pub arrival_time: Option<f32>,
    /// Assigned list of links forming the trip route.
    pub route_links: Vec<LinkId>,
    /// Current index in `route_links`.
    pub route_index: usize,
    /// Distance traveled along the current link in meters ($0 \le x \le L$).
    pub distance_on_link: f32,
    /// Current vehicle speed in m/s.
    pub speed: f32,
    /// Assigned lane index.
    pub lane: u32,
    /// Current lifecycle state of the vehicle.
    pub state: VehicleState,
    /// Aggregation packet size (1 for individual car, >1 for aggregated flow platoon).
    pub packet_size: u32,
}

impl Vehicle {
    /// Creates a new vehicle with origin, destination, departure time, and route.
    pub fn new(
        id: u64,
        origin: NodeId,
        destination: NodeId,
        departure_time: f32,
        route_links: Vec<LinkId>,
    ) -> Self {
        Self {
            id,
            origin,
            destination,
            departure_time,
            arrival_time: None,
            route_links,
            route_index: 0,
            distance_on_link: 0.0,
            speed: 0.0,
            lane: 0,
            state: VehicleState::WaitingToDepart,
            packet_size: 1,
        }
    }

    /// Sets the packet size for aggregated platoon simulation.
    pub fn with_packet_size(mut self, size: u32) -> Self {
        self.packet_size = size.max(1);
        self
    }

    /// Returns the current link ID if the vehicle is currently on one.
    pub fn current_link(&self) -> Option<LinkId> {
        if self.state == VehicleState::EnRoute || self.state == VehicleState::QueuedAtIntersection {
            self.route_links.get(self.route_index).copied()
        } else {
            None
        }
    }

    /// Returns the next link ID along the route if one exists.
    pub fn next_link(&self) -> Option<LinkId> {
        self.route_links.get(self.route_index + 1).copied()
    }
}

/// Statistics and performance metrics of the macroscopic simulation.
#[derive(Debug, Clone, Copy, Default, PartialEq, Serialize, Deserialize)]
pub struct SimulationStats {
    /// Total number of vehicles introduced into the system.
    pub total_vehicles: usize,
    /// Vehicles currently en route or queued.
    pub active_vehicles: usize,
    /// Vehicles currently in microscopic simulation.
    pub handed_off_vehicles: usize,
    /// Vehicles that have reached their destination.
    pub completed_vehicles: usize,
    /// Total travel time experienced by completed trips (seconds).
    pub total_travel_time: f32,
    /// Average trip travel time for completed trips (seconds).
    pub average_travel_time: f32,
    /// Average network speed (m/s).
    pub average_speed: f32,
}

/// Macroscopic traffic flow simulator.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TrafficSimulator {
    /// Underlying road network graph.
    pub network: RoadNetwork,
    /// Map of all vehicles by ID.
    pub vehicles: HashMap<u64, Vehicle>,
    /// Link downstream intersection queues: LinkId -> FIFO queue of vehicle IDs.
    link_queues: HashMap<LinkId, VecDeque<u64>>,
    /// Current simulation time in seconds.
    pub current_time: f32,
    /// Simulation timestep size in seconds ($\Delta t$).
    pub dt: f32,
    /// Routing engine for path assignment.
    pub router: Router,
}

impl TrafficSimulator {
    /// Creates a new traffic simulator for a given network.
    pub fn new(network: RoadNetwork, dt: f32) -> Self {
        assert!(dt > 0.0, "Simulation timestep dt must be positive");
        Self {
            network,
            vehicles: HashMap::new(),
            link_queues: HashMap::new(),
            current_time: 0.0,
            dt,
            router: Router::new(RoutingCostMetric::DynamicTravelTime),
        }
    }

    /// Adds a pre-routed vehicle to the simulation.
    pub fn add_vehicle(&mut self, vehicle: Vehicle) -> Result<()> {
        if self.vehicles.contains_key(&vehicle.id) {
            return Err(UxsimError::DuplicateVehicle(vehicle.id));
        }
        self.vehicles.insert(vehicle.id, vehicle);
        Ok(())
    }

    /// Creates and adds a vehicle, automatically computing its optimal route using A*.
    pub fn add_trip(
        &mut self,
        id: u64,
        origin: NodeId,
        destination: NodeId,
        departure_time: f32,
    ) -> Result<()> {
        let route = self.router.a_star(&self.network, origin, destination)?;
        if route.links.is_empty() {
            return Err(UxsimError::NoPathFound(origin, destination));
        }
        let vehicle = Vehicle::new(id, origin, destination, departure_time, route.links);
        self.add_vehicle(vehicle)
    }

    /// Calculates world position and forward direction of a vehicle on the road network.
    pub fn vehicle_world_pose(&self, vehicle_id: u64) -> Option<(Vec2, Vec2)> {
        let vehicle = self.vehicles.get(&vehicle_id)?;
        let link_id = vehicle.current_link()?;
        let link = self.network.get_link(link_id)?;
        let from_node = self.network.get_node(link.from_node)?;
        let to_node = self.network.get_node(link.to_node)?;

        let segment = to_node.position - from_node.position;
        let segment_len = segment.length();
        let direction = if segment_len > 1e-4 {
            segment / segment_len
        } else {
            Vec2::X
        };

        let t = if link.length > 0.0 {
            (vehicle.distance_on_link / link.length).clamp(0.0, 1.0)
        } else {
            0.0
        };

        let position = from_node.position + segment * t;
        Some((position, direction))
    }

    /// Hands off a macroscopic vehicle to microscopic simulation (e.g. within player bubble).
    pub fn handoff_to_micro(&mut self, vehicle_id: u64) -> Result<MicroVehicleHandoff> {
        let (world_pos, forward) = self
            .vehicle_world_pose(vehicle_id)
            .ok_or_else(|| UxsimError::InvalidParameter("Failed to compute vehicle pose".to_string()))?;

        let (link_id, speed, distance_on_link, lane, origin, destination, remaining_links, packet_size) = {
            let vehicle = self
                .vehicles
                .get_mut(&vehicle_id)
                .ok_or(UxsimError::VehicleNotFound(vehicle_id))?;

            if vehicle.state == VehicleState::HandedOffToMicro || vehicle.state == VehicleState::Arrived {
                return Err(UxsimError::InvalidParameter(format!(
                    "Vehicle {vehicle_id} is in state {:?}, cannot hand off",
                    vehicle.state
                )));
            }

            let link_id = vehicle
                .current_link()
                .ok_or_else(|| UxsimError::InvalidParameter("Vehicle not on an active link".to_string()))?;

            let speed = vehicle.speed;
            let distance_on_link = vehicle.distance_on_link;
            let lane = vehicle.lane;
            let origin = vehicle.origin;
            let destination = vehicle.destination;
            let remaining_links = vehicle.route_links[vehicle.route_index..].to_vec();
            let packet_size = vehicle.packet_size;

            vehicle.state = VehicleState::HandedOffToMicro;
            (
                link_id,
                speed,
                distance_on_link,
                lane,
                origin,
                destination,
                remaining_links,
                packet_size,
            )
        };

        // Reduce link vehicle count so macroscopic density does not double count
        if let Some(link) = self.network.get_link_mut(link_id) {
            link.vehicle_count = (link.vehicle_count - packet_size as f32).max(0.0);
        }

        // Remove from queue if present
        if let Some(queue) = self.link_queues.get_mut(&link_id) {
            queue.retain(|&id| id != vehicle_id);
        }

        Ok(MicroVehicleHandoff {
            vehicle_id,
            origin_node: origin,
            destination_node: destination,
            link_id,
            remaining_links,
            distance_on_link,
            world_position: world_pos,
            forward_direction: forward,
            speed,
            lane,
            handoff_timestamp: self.current_time,
        })
    }

    /// Assimilates a vehicle returning from microscopic simulation back into macroscopic network.
    pub fn assimilate_from_micro(&mut self, handoff: MicroVehicleHandoff) -> Result<()> {
        let vehicle = self
            .vehicles
            .get_mut(&handoff.vehicle_id)
            .ok_or(UxsimError::VehicleNotFound(handoff.vehicle_id))?;

        let link = self
            .network
            .get_link_mut(handoff.link_id)
            .ok_or(UxsimError::LinkNotFound(handoff.link_id))?;

        // Re-inject vehicle into macro link
        link.vehicle_count += vehicle.packet_size as f32;

        vehicle.distance_on_link = handoff.distance_on_link.clamp(0.0, link.length);
        vehicle.speed = handoff.speed;
        vehicle.lane = handoff.lane;
        vehicle.route_links = handoff.remaining_links;
        vehicle.route_index = 0;

        if vehicle.distance_on_link >= link.length {
            vehicle.state = VehicleState::QueuedAtIntersection;
            self.link_queues
                .entry(handoff.link_id)
                .or_default()
                .push_back(vehicle.id);
        } else {
            vehicle.state = VehicleState::EnRoute;
        }

        Ok(())
    }

    /// Queries all active vehicles currently within a render bubble.
    pub fn query_vehicles_in_bubble(&self, bubble: &RenderBubble) -> Vec<u64> {
        let mut in_bubble = Vec::new();
        for (&id, vehicle) in &self.vehicles {
            if (vehicle.state == VehicleState::EnRoute || vehicle.state == VehicleState::QueuedAtIntersection)
                && let Some((pos, _)) = self.vehicle_world_pose(id)
                    && bubble.contains(pos) {
                        in_bubble.push(id);
                    }
        }
        in_bubble
    }

    /// Advances the macroscopic simulation by one timestep $\Delta t$.
    pub fn step(&mut self) {
        let dt = self.dt;
        let current_time = self.current_time;

        // 1. Depart scheduled vehicles
        let mut to_depart = Vec::new();
        for (&id, veh) in &self.vehicles {
            if veh.state == VehicleState::WaitingToDepart && veh.departure_time <= current_time {
                to_depart.push(id);
            }
        }

        for id in to_depart {
            if let Some(veh) = self.vehicles.get_mut(&id)
                && let Some(&first_link_id) = veh.route_links.first()
                    && let Some(link) = self.network.get_link_mut(first_link_id) {
                        link.vehicle_count += veh.packet_size as f32;
                        veh.state = VehicleState::EnRoute;
                        veh.distance_on_link = 0.0;
                        veh.speed = link.current_speed;
                    }
        }

        // 2. Advance vehicles along links
        let mut reached_end = Vec::new();
        for (&id, veh) in &mut self.vehicles {
            if veh.state == VehicleState::EnRoute
                && let Some(link_id) = veh.current_link()
                    && let Some(link) = self.network.get_link(link_id) {
                        veh.speed = link.current_speed;
                        veh.distance_on_link += veh.speed * dt;

                        if veh.distance_on_link >= link.length {
                            veh.distance_on_link = link.length;
                            veh.state = VehicleState::QueuedAtIntersection;
                            reached_end.push((id, link_id));
                        }
                    }
        }

        // Enqueue vehicles that reached downstream intersection
        for (veh_id, link_id) in reached_end {
            self.link_queues
                .entry(link_id)
                .or_default()
                .push_back(veh_id);
        }

        // 3. Process intersection queues (downstream propagation)
        let link_ids: Vec<LinkId> = self.link_queues.keys().copied().collect();
        for link_id in link_ids {
            let (to_node_id, sending_rate) = match self.network.get_link(link_id) {
                Some(l) => (l.to_node, l.sending_flow()),
                None => continue,
            };

            // Check traffic signal at intersection
            let signal_allowed = match self.network.get_node(to_node_id) {
                Some(n) => n.signal_state != SignalState::Red,
                None => true,
            };

            if !signal_allowed {
                continue;
            }

            // Number of vehicles that can physically leave link in dt
            let mut remaining_capacity = ((sending_rate * dt).ceil() as usize).max(1);

            let queue = match self.link_queues.get_mut(&link_id) {
                Some(q) => q,
                None => continue,
            };

            let mut passed = Vec::new();

            for &veh_id in queue.iter() {
                if remaining_capacity == 0 {
                    break;
                }

                let veh = match self.vehicles.get(&veh_id) {
                    Some(v) => v,
                    None => continue,
                };

                // Check next destination / next link
                if let Some(next_link_id) = veh.next_link() {
                    let next_link = match self.network.get_link(next_link_id) {
                        Some(l) => l,
                        None => continue,
                    };

                    // Check downstream receiving capacity and storage space
                    let has_space = next_link.available_space() >= veh.packet_size as f32;
                    let has_flow_supply =
                        (next_link.receiving_flow() * dt).max(1.0) >= veh.packet_size as f32;

                    if has_space && has_flow_supply {
                        passed.push((veh_id, Some(next_link_id)));
                        remaining_capacity = remaining_capacity.saturating_sub(1);
                    }
                } else {
                    // Vehicle reached its final destination!
                    passed.push((veh_id, None));
                    remaining_capacity = remaining_capacity.saturating_sub(1);
                }
            }

            // Remove passed vehicles from the queue and advance them
            for (veh_id, next_link_opt) in passed {
                if let Some(q) = self.link_queues.get_mut(&link_id) {
                    q.retain(|&id| id != veh_id);
                }

                // Decrement count on current link
                if let Some(curr_link) = self.network.get_link_mut(link_id)
                    && let Some(veh) = self.vehicles.get(&veh_id) {
                        curr_link.vehicle_count =
                            (curr_link.vehicle_count - veh.packet_size as f32).max(0.0);
                        curr_link.outflow_rate += veh.packet_size as f32 / dt;
                    }

                if let Some(veh) = self.vehicles.get_mut(&veh_id) {
                    if let Some(next_link_id) = next_link_opt {
                        // Move to next link
                        veh.route_index += 1;
                        veh.distance_on_link = 0.0;
                        veh.state = VehicleState::EnRoute;

                        if let Some(next_link) = self.network.get_link_mut(next_link_id) {
                            next_link.vehicle_count += veh.packet_size as f32;
                            next_link.inflow_rate += veh.packet_size as f32 / dt;
                            veh.speed = next_link.current_speed;
                        }
                    } else {
                        // Final destination reached
                        veh.state = VehicleState::Arrived;
                        veh.arrival_time = Some(self.current_time);
                    }
                }
            }
        }

        // 4. Update all link dynamics across network
        self.network.update_all_link_dynamics();

        // 5. Advance simulation clock
        self.current_time += dt;
    }

    /// Runs the simulation for `duration` seconds in steps of `self.dt`.
    pub fn run_for(&mut self, duration: f32) {
        let steps = (duration / self.dt).ceil() as usize;
        for _ in 0..steps {
            self.step();
        }
    }

    /// Computes summary statistics of the traffic simulation.
    pub fn stats(&self) -> SimulationStats {
        let total = self.vehicles.len();
        let mut active = 0;
        let mut handed_off = 0;
        let mut completed = 0;
        let mut total_tt = 0.0;
        let mut speed_sum = 0.0;
        let mut speed_count = 0;

        for veh in self.vehicles.values() {
            match veh.state {
                VehicleState::EnRoute | VehicleState::QueuedAtIntersection => {
                    active += 1;
                    speed_sum += veh.speed;
                    speed_count += 1;
                }
                VehicleState::HandedOffToMicro => {
                    handed_off += 1;
                }
                VehicleState::Arrived => {
                    completed += 1;
                    if let Some(arr) = veh.arrival_time {
                        total_tt += (arr - veh.departure_time).max(0.0);
                    }
                }
                VehicleState::WaitingToDepart => {}
            }
        }

        let avg_tt = if completed > 0 {
            total_tt / completed as f32
        } else {
            0.0
        };

        let avg_speed = if speed_count > 0 {
            speed_sum / speed_count as f32
        } else {
            0.0
        };

        SimulationStats {
            total_vehicles: total,
            active_vehicles: active,
            handed_off_vehicles: handed_off,
            completed_vehicles: completed,
            total_travel_time: total_tt,
            average_travel_time: avg_tt,
            average_speed: avg_speed,
        }
    }
}
