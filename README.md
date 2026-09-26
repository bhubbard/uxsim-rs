# uxsim-rs

[![Crates.io](https://img.shields.io/badge/crates.io-v0.1.0-orange.svg)](https://crates.io)
[![Documentation](https://docs.rs/uxsim-rs/badge.svg)](https://docs.rs/uxsim-rs)
[![Website](https://img.shields.io/badge/website-live-brightgreen.svg)](https://code.brandonhubbard.com/uxsim-rs/)
[![License](https://img.shields.io/badge/license-MIT%20OR%20Apache--2.0-blue.svg)](LICENSE)

A pure Rust macroscopic city-wide traffic network flow simulator inspired by [UXsim](https://github.com/toruseo/UXsim). Designed specifically for open-world games and large-scale urban simulation (such as open-world Bevy RPGs).

Simulate tens of thousands of vehicles across an entire metropolis at negligible CPU cost, seamlessly handing off vehicles to microscopic simulation (such as `sumo-rs` or local physics) inside the player's render bubble.

---

## Key Features

- **Macroscopic Traffic Flow Theory**:
  - **Greenshields Linear Model**: $v(k) = v_f \left(1 - \frac{k}{k_j}\right)$ with parabolic flow-density curve $q(k) = v_f \left(k - \frac{k^2}{k_j}\right)$.
  - **Newell Simplified Triangular Diagram**: Free-flow regime $q = v_f \cdot k$ and congested branch $q = w \cdot (k_j - k)$ with constant backward shockwave speed $w$.
  - **Trapezoidal Fundamental Diagram**: Free-flow, capacity plateau $q_{\max}$, and congested recovery.
  - **Shockwave Speed Dynamics**: Exact Rankine-Hugoniot jump condition $w_{\text{shock}} = \frac{q_2 - q_1}{k_2 - k_1}$ for queue and shockwave tracking.
- **Node-Link Network Dynamics**:
  - Directed links with physical length, lane count, free-flow travel time $\tau = L / v_f$, and storage capacity $K = k_j \cdot L \cdot \text{lanes}$.
  - Intersection queues with traffic signal control (Green, Yellow, Red, Uncontrolled).
  - Upstream queue spillback and bottleneck back-propagation.
  - Aggregated flow packet support (simulate platoons of $N$ cars as a single entity).
- **Dynamic Routing Engine**:
  - **Dijkstra** and **A\*** pathfinding.
  - Routing cost metrics: dynamic congestion travel time, free-flow travel time, or physical distance.
  - Dynamic re-routing around traffic bottlenecks.
- **Background City Simulation & Micro Handoff**:
  - **`RenderBubble` Boundary System**: Evaluates vehicles approaching the player or active camera.
  - **Handoff to Microscopic Simulation**: Extracts exact 2D world coordinates, forward heading, lane, speed, and remaining route to hand off to local physics or Bevy ECS agents.
  - **Assimilation from Microscopic Simulation**: Smoothly re-integrates vehicles exiting the render bubble back into the macroscopic road network queue without double-counting link density.

---

## Quick Start

Add to your `Cargo.toml`:

```toml
[dependencies]
uxsim-rs = "0.1"
glam = "0.29"
```

### Building a Network and Running Simulation

```rust
use glam::Vec2;
use uxsim_rs::prelude::*;
use uxsim_rs::{
    Link, Node, RoadNetwork, TrafficSimulator, Vehicle,
    flow_model::{FlowModel, NewellTriangular},
};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut network = RoadNetwork::new();

    // 1. Add intersection nodes
    network.add_node(1, Vec2::new(0.0, 0.0))?;
    network.add_node(2, Vec2::new(500.0, 0.0))?;
    network.add_node(3, Vec2::new(1000.0, 0.0))?;

    // 2. Add road links (length: 500m, 2 lanes, urban Newell model)
    let flow_model = FlowModel::NewellTriangular(NewellTriangular::standard_urban());
    network.add_link(Link::new(12, 1, 2, 500.0, 2, flow_model))?;
    network.add_link(Link::new(23, 2, 3, 500.0, 2, flow_model))?;

    // 3. Initialize simulator with 1.0 second timestep
    let mut sim = TrafficSimulator::new(network, 1.0);

    // 4. Inject trips (either explicit route or A* auto-routed)
    sim.add_trip(1001, 1, 3, 0.0)?;

    // 5. Step simulation
    for _ in 0..60 {
        sim.step();
    }

    let stats = sim.stats();
    println!("Completed trips: {}", stats.completed_vehicles);
    println!("Average travel time: {:.1}s", stats.average_travel_time);

    Ok(())
}
```

### Microscopic Handoff Example (Bevy Render Bubble)

```rust
use glam::Vec2;
use uxsim_rs::handoff::RenderBubble;
use uxsim_rs::simulation::TrafficSimulator;

fn update_traffic_boundary(sim: &mut TrafficSimulator, player_pos: Vec2) {
    let bubble = RenderBubble::new(player_pos, 150.0, 25.0);

    // 1. Query macroscopic vehicles that entered the player render bubble
    let entering_ids = sim.query_vehicles_in_bubble(&bubble);

    for veh_id in entering_ids {
        // Handoff to micro: removes from macro count and returns precise world pose
        if let Ok(handoff) = sim.handoff_to_micro(veh_id) {
            println!(
                "Spawning micro car #{} at {:?} heading {:?}",
                handoff.vehicle_id, handoff.world_position, handoff.forward_direction
            );
            // Spawn Bevy ECS entity or hand off to sumo-rs micro controller...
        }
    }
}
```

---

## Architecture

```
                  ┌──────────────────────────────────────────────┐
                  │          Background City Simulation          │
                  │   Inexpensive Macroscopic Link Propagation   │
                  │       (Greenshields / Newell Triangular)     │
                  └──────────────────────┬───────────────────────┘
                                         │
                         Entering        │        Exiting
                       Render Bubble     │     Render Bubble
                             ▼           │           ▲
                  ┌──────────────────────┴───────────┴───────────┐
                  │             RenderBubble Boundary            │
                  │         `handoff_to_micro` / `assimilate`    │
                  └──────────────────────┬───────────────────────┘
                                         │
                                         ▼
                  ┌──────────────────────────────────────────────┐
                  │          Foreground Microscopic Sim          │
                  │   High-fidelity Bevy ECS / sumo-rs / Detour  │
                  │      (Full physics, steering & rendering)    │
                  └──────────────────────────────────────────────┘
```

---

## License

Dual-licensed under either:

- MIT License ([LICENSE-MIT](LICENSE-MIT) or <http://opensource.org/licenses/MIT>)
- Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or <http://www.apache.org/licenses/LICENSE-2.0>)

at your option.
