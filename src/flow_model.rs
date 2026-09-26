//! Macroscopic traffic flow fundamental diagrams and models.
//!
//! Provides Greenshields, Newell triangular, and trapezoidal fundamental diagrams
//! relating density ($k$), flow ($q$), and speed ($v$), along with shockwave propagation.

use serde::{Deserialize, Serialize};

/// Trait defining the relationship between traffic density $k$ (veh/m/lane),
/// speed $v$ (m/s), and flow $q$ (veh/s/lane).
pub trait FundamentalDiagram: Send + Sync {
    /// Free-flow speed $v_f$ in m/s.
    fn free_flow_speed(&self) -> f32;

    /// Jam density $k_j$ in veh/m/lane.
    fn jam_density(&self) -> f32;

    /// Capacity / maximum flow $q_{\max}$ in veh/s/lane.
    fn capacity(&self) -> f32;

    /// Critical density $k_c$ at capacity in veh/m/lane.
    fn critical_density(&self) -> f32;

    /// Calculates speed $v(k)$ in m/s for a given density $k$ in veh/m/lane.
    fn speed(&self, density: f32) -> f32;

    /// Calculates flow rate $q(k) = k \cdot v(k)$ in veh/s/lane.
    fn flow(&self, density: f32) -> f32 {
        if density <= 0.0 {
            0.0
        } else {
            (density * self.speed(density)).min(self.capacity()).max(0.0)
        }
    }

    /// Demand / sending flow function $D(k)$: max vehicles ready to leave link.
    fn demand(&self, density: f32) -> f32 {
        let k = density.clamp(0.0, self.jam_density());
        if k <= self.critical_density() {
            self.flow(k)
        } else {
            self.capacity()
        }
    }

    /// Supply / receiving flow function $S(k)$: max vehicles that downstream can accept.
    fn supply(&self, density: f32) -> f32 {
        let k = density.clamp(0.0, self.jam_density());
        if k <= self.critical_density() {
            self.capacity()
        } else {
            self.flow(k)
        }
    }

    /// Calculates the shockwave propagation speed between two traffic states $(k_1, k_2)$.
    ///
    /// Rankine-Hugoniot condition: $w = \frac{q_2 - q_1}{k_2 - k_1}$.
    fn shockwave_speed(&self, k1: f32, k2: f32) -> f32 {
        let dk = k2 - k1;
        if dk.abs() < 1e-6 {
            // Numerical derivative
            let eps = 1e-4;
            let q_plus = self.flow(k1 + eps);
            let q_minus = self.flow((k1 - eps).max(0.0));
            (q_plus - q_minus) / (2.0 * eps)
        } else {
            (self.flow(k2) - self.flow(k1)) / dk
        }
    }
}

/// Greenshields linear speed-density model:
///
/// $$v(k) = v_f \left(1 - \frac{k}{k_j}\right)$$
///
/// Yields a parabolic flow-density curve:
///
/// $$q(k) = v_f \left(k - \frac{k^2}{k_j}\right)$$
///
/// with critical density $k_c = k_j / 2$ and capacity $q_{\max} = \frac{v_f \cdot k_j}{4}$.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Greenshields {
    /// Free-flow speed in m/s.
    pub free_flow_speed: f32,
    /// Jam density in veh/m/lane.
    pub jam_density: f32,
}

impl Greenshields {
    /// Creates a new Greenshields model with $v_f$ and $k_j$.
    pub fn new(free_flow_speed: f32, jam_density: f32) -> Self {
        assert!(free_flow_speed > 0.0, "Free flow speed must be positive");
        assert!(jam_density > 0.0, "Jam density must be positive");
        Self {
            free_flow_speed,
            jam_density,
        }
    }

    /// Convenience constructor with standard urban values (e.g. 50 km/h, 0.14 veh/m = 140 veh/km).
    pub fn standard_urban() -> Self {
        Self {
            free_flow_speed: 13.89, // ~50 km/h
            jam_density: 0.14,      // 140 veh/km/lane (~7.14 m per vehicle)
        }
    }

    /// Convenience constructor with highway values (e.g. 100 km/h, 0.13 veh/m).
    pub fn standard_highway() -> Self {
        Self {
            free_flow_speed: 27.78, // ~100 km/h
            jam_density: 0.13,      // 130 veh/km/lane
        }
    }
}

impl FundamentalDiagram for Greenshields {
    #[inline]
    fn free_flow_speed(&self) -> f32 {
        self.free_flow_speed
    }

    #[inline]
    fn jam_density(&self) -> f32 {
        self.jam_density
    }

    #[inline]
    fn capacity(&self) -> f32 {
        (self.free_flow_speed * self.jam_density) / 4.0
    }

    #[inline]
    fn critical_density(&self) -> f32 {
        self.jam_density / 2.0
    }

    #[inline]
    fn speed(&self, density: f32) -> f32 {
        if density <= 0.0 {
            self.free_flow_speed
        } else if density >= self.jam_density {
            0.0
        } else {
            self.free_flow_speed * (1.0 - density / self.jam_density)
        }
    }
}

/// Newell's simplified triangular fundamental diagram:
///
/// - Free-flow branch ($k \le k_c$): $q = v_f \cdot k$, $v = v_f$
/// - Congested branch ($k > k_c$): $q = w \cdot (k_j - k)$, $v = \frac{w (k_j - k)}{k}$
///
/// where $w$ is the constant backward wave speed (shockwave speed in congestion).
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct NewellTriangular {
    /// Free-flow speed in m/s ($v_f$).
    pub free_flow_speed: f32,
    /// Backward wave speed in m/s ($w$).
    pub wave_speed: f32,
    /// Jam density in veh/m/lane ($k_j$).
    pub jam_density: f32,
}

impl NewellTriangular {
    /// Creates a Newell triangular model given $v_f$, backward wave speed $w$, and jam density $k_j$.
    pub fn new(free_flow_speed: f32, wave_speed: f32, jam_density: f32) -> Self {
        assert!(free_flow_speed > 0.0, "Free flow speed must be positive");
        assert!(wave_speed > 0.0, "Wave speed must be positive");
        assert!(jam_density > 0.0, "Jam density must be positive");
        Self {
            free_flow_speed,
            wave_speed,
            jam_density,
        }
    }

    /// Creates a Newell triangular model given $v_f$, capacity $q_{\max}$, and jam density $k_j$.
    pub fn from_capacity(free_flow_speed: f32, capacity: f32, jam_density: f32) -> Self {
        let kc = capacity / free_flow_speed;
        assert!(kc < jam_density, "Critical density must be less than jam density");
        let wave_speed = capacity / (jam_density - kc);
        Self {
            free_flow_speed,
            wave_speed,
            jam_density,
        }
    }

    /// Typical urban road configuration ($v_f \approx 15$ m/s, $w \approx 5$ m/s, $k_j \approx 0.14$ veh/m).
    pub fn standard_urban() -> Self {
        Self {
            free_flow_speed: 15.0,
            wave_speed: 5.0,
            jam_density: 0.14,
        }
    }

    /// Backward wave speed $w$.
    #[inline]
    pub fn backward_wave_speed(&self) -> f32 {
        self.wave_speed
    }
}

impl FundamentalDiagram for NewellTriangular {
    #[inline]
    fn free_flow_speed(&self) -> f32 {
        self.free_flow_speed
    }

    #[inline]
    fn jam_density(&self) -> f32 {
        self.jam_density
    }

    #[inline]
    fn critical_density(&self) -> f32 {
        (self.wave_speed * self.jam_density) / (self.free_flow_speed + self.wave_speed)
    }

    #[inline]
    fn capacity(&self) -> f32 {
        self.free_flow_speed * self.critical_density()
    }

    #[inline]
    fn speed(&self, density: f32) -> f32 {
        if density <= 0.0 {
            self.free_flow_speed
        } else if density >= self.jam_density {
            0.0
        } else {
            let kc = self.critical_density();
            if density <= kc {
                self.free_flow_speed
            } else {
                let cong_flow = self.wave_speed * (self.jam_density - density);
                (cong_flow / density).clamp(0.0, self.free_flow_speed)
            }
        }
    }

    #[inline]
    fn flow(&self, density: f32) -> f32 {
        if density <= 0.0 || density >= self.jam_density {
            0.0
        } else {
            let kc = self.critical_density();
            if density <= kc {
                self.free_flow_speed * density
            } else {
                self.wave_speed * (self.jam_density - density)
            }
        }
    }
}

/// Trapezoidal fundamental diagram with a flat capacity plateau between $k_{c1}$ and $k_{c2}$.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct TrapezoidalDiagram {
    /// Free-flow speed in m/s ($v_f$).
    pub free_flow_speed: f32,
    /// Capacity / max flow in veh/s/lane.
    pub capacity: f32,
    /// Upper critical density where congestion begins.
    pub upper_critical_density: f32,
    /// Jam density in veh/m/lane ($k_j$).
    pub jam_density: f32,
}

impl TrapezoidalDiagram {
    /// Creates a new trapezoidal diagram.
    pub fn new(
        free_flow_speed: f32,
        capacity: f32,
        upper_critical_density: f32,
        jam_density: f32,
    ) -> Self {
        let kc1 = capacity / free_flow_speed;
        assert!(kc1 <= upper_critical_density, "Lower critical density must <= upper");
        assert!(upper_critical_density < jam_density, "Upper critical density must < jam density");
        Self {
            free_flow_speed,
            capacity,
            upper_critical_density,
            jam_density,
        }
    }

    /// Lower critical density $k_{c1} = q_{\max} / v_f$.
    #[inline]
    pub fn lower_critical_density(&self) -> f32 {
        self.capacity / self.free_flow_speed
    }

    /// Congested wave speed $w = q_{\max} / (k_j - k_{c2})$.
    #[inline]
    pub fn wave_speed(&self) -> f32 {
        self.capacity / (self.jam_density - self.upper_critical_density)
    }
}

impl FundamentalDiagram for TrapezoidalDiagram {
    #[inline]
    fn free_flow_speed(&self) -> f32 {
        self.free_flow_speed
    }

    #[inline]
    fn jam_density(&self) -> f32 {
        self.jam_density
    }

    #[inline]
    fn capacity(&self) -> f32 {
        self.capacity
    }

    #[inline]
    fn critical_density(&self) -> f32 {
        self.lower_critical_density()
    }

    #[inline]
    fn speed(&self, density: f32) -> f32 {
        if density <= 0.0 {
            self.free_flow_speed
        } else if density >= self.jam_density {
            0.0
        } else {
            let kc1 = self.lower_critical_density();
            if density <= kc1 {
                self.free_flow_speed
            } else if density <= self.upper_critical_density {
                self.capacity / density
            } else {
                let w = self.wave_speed();
                (w * (self.jam_density - density) / density).clamp(0.0, self.free_flow_speed)
            }
        }
    }

    #[inline]
    fn flow(&self, density: f32) -> f32 {
        if density <= 0.0 || density >= self.jam_density {
            0.0
        } else {
            let kc1 = self.lower_critical_density();
            if density <= kc1 {
                self.free_flow_speed * density
            } else if density <= self.upper_critical_density {
                self.capacity
            } else {
                let w = self.wave_speed();
                w * (self.jam_density - density)
            }
        }
    }
}

/// Dynamic enum wrapper for serializable fundamental diagrams.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub enum FlowModel {
    Greenshields(Greenshields),
    NewellTriangular(NewellTriangular),
    Trapezoidal(TrapezoidalDiagram),
}

impl Default for FlowModel {
    fn default() -> Self {
        Self::NewellTriangular(NewellTriangular::standard_urban())
    }
}

impl FundamentalDiagram for FlowModel {
    #[inline]
    fn free_flow_speed(&self) -> f32 {
        match self {
            Self::Greenshields(m) => m.free_flow_speed(),
            Self::NewellTriangular(m) => m.free_flow_speed(),
            Self::Trapezoidal(m) => m.free_flow_speed(),
        }
    }

    #[inline]
    fn jam_density(&self) -> f32 {
        match self {
            Self::Greenshields(m) => m.jam_density(),
            Self::NewellTriangular(m) => m.jam_density(),
            Self::Trapezoidal(m) => m.jam_density(),
        }
    }

    #[inline]
    fn capacity(&self) -> f32 {
        match self {
            Self::Greenshields(m) => m.capacity(),
            Self::NewellTriangular(m) => m.capacity(),
            Self::Trapezoidal(m) => m.capacity(),
        }
    }

    #[inline]
    fn critical_density(&self) -> f32 {
        match self {
            Self::Greenshields(m) => m.critical_density(),
            Self::NewellTriangular(m) => m.critical_density(),
            Self::Trapezoidal(m) => m.critical_density(),
        }
    }

    #[inline]
    fn speed(&self, density: f32) -> f32 {
        match self {
            Self::Greenshields(m) => m.speed(density),
            Self::NewellTriangular(m) => m.speed(density),
            Self::Trapezoidal(m) => m.speed(density),
        }
    }

    #[inline]
    fn flow(&self, density: f32) -> f32 {
        match self {
            Self::Greenshields(m) => m.flow(density),
            Self::NewellTriangular(m) => m.flow(density),
            Self::Trapezoidal(m) => m.flow(density),
        }
    }

    #[inline]
    fn demand(&self, density: f32) -> f32 {
        match self {
            Self::Greenshields(m) => m.demand(density),
            Self::NewellTriangular(m) => m.demand(density),
            Self::Trapezoidal(m) => m.demand(density),
        }
    }

    #[inline]
    fn supply(&self, density: f32) -> f32 {
        match self {
            Self::Greenshields(m) => m.supply(density),
            Self::NewellTriangular(m) => m.supply(density),
            Self::Trapezoidal(m) => m.supply(density),
        }
    }

    #[inline]
    fn shockwave_speed(&self, k1: f32, k2: f32) -> f32 {
        match self {
            Self::Greenshields(m) => m.shockwave_speed(k1, k2),
            Self::NewellTriangular(m) => m.shockwave_speed(k1, k2),
            Self::Trapezoidal(m) => m.shockwave_speed(k1, k2),
        }
    }
}
