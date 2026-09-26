use uxsim_rs::flow_model::{
    FlowModel, FundamentalDiagram, Greenshields, NewellTriangular, TrapezoidalDiagram,
};

#[test]
fn test_greenshields_model() {
    let vf = 20.0; // 20 m/s
    let kj = 0.20; // 0.2 veh/m = 200 veh/km
    let model = Greenshields::new(vf, kj);

    // Free flow (k = 0)
    assert!((model.speed(0.0) - 20.0).abs() < 1e-4);
    assert!((model.flow(0.0) - 0.0).abs() < 1e-4);

    // Critical density kc = kj / 2 = 0.10
    assert!((model.critical_density() - 0.10).abs() < 1e-4);
    assert!((model.speed(0.10) - 10.0).abs() < 1e-4);

    // Capacity q_max = vf * kj / 4 = 20 * 0.2 / 4 = 1.0 veh/s
    assert!((model.capacity() - 1.0).abs() < 1e-4);
    assert!((model.flow(0.10) - 1.0).abs() < 1e-4);

    // Jam density (k = kj = 0.20)
    assert!((model.speed(0.20) - 0.0).abs() < 1e-4);
    assert!((model.flow(0.20) - 0.0).abs() < 1e-4);

    // Beyond jam density clamps to 0
    assert!((model.speed(0.25) - 0.0).abs() < 1e-4);

    // Shockwave speed between k1 = 0.05 (q = 20 * (0.05 - 0.0025/0.2) = 20 * 0.0375 = 0.75)
    // and k2 = 0.15 (q = 20 * (0.15 - 0.0225/0.2) = 20 * 0.0375 = 0.75)
    // dq/dk = 0 => shockwave speed should be 0
    let w_shock = model.shockwave_speed(0.05, 0.15);
    assert!(w_shock.abs() < 1e-4);
}

#[test]
fn test_newell_triangular_model() {
    let vf = 15.0; // 15 m/s
    let w = 5.0;   // backward wave speed 5 m/s
    let kj = 0.20; // 0.2 veh/m
    let model = NewellTriangular::new(vf, w, kj);

    // kc = w * kj / (vf + w) = 5 * 0.2 / 20 = 0.05 veh/m
    let kc = model.critical_density();
    assert!((kc - 0.05).abs() < 1e-4);

    // Capacity q_max = vf * kc = 15 * 0.05 = 0.75 veh/s
    let q_max = model.capacity();
    assert!((q_max - 0.75).abs() < 1e-4);

    // Free flow regime (k = 0.02 < kc)
    assert!((model.speed(0.02) - 15.0).abs() < 1e-4);
    assert!((model.flow(0.02) - 0.30).abs() < 1e-4);

    // At capacity (k = 0.05)
    assert!((model.flow(0.05) - 0.75).abs() < 1e-4);

    // Congested regime (k = 0.10 > kc): q = w * (kj - k) = 5 * (0.2 - 0.1) = 0.50 veh/s
    assert!((model.flow(0.10) - 0.50).abs() < 1e-4);
    assert!((model.speed(0.10) - 5.0).abs() < 1e-4);

    // Shockwave speed in congested branch: (q(0.15) - q(0.10)) / (0.15 - 0.10) = -w = -5.0
    let shock_cong = model.shockwave_speed(0.10, 0.15);
    assert!((shock_cong - (-5.0)).abs() < 1e-4);

    // Demand and supply
    assert!((model.demand(0.02) - 0.30).abs() < 1e-4);
    assert!((model.demand(0.10) - 0.75).abs() < 1e-4); // Congested demand is capacity
    assert!((model.supply(0.02) - 0.75).abs() < 1e-4); // Free flow supply is capacity
    assert!((model.supply(0.10) - 0.50).abs() < 1e-4); // Congested supply is q(k)
}

#[test]
fn test_trapezoidal_diagram() {
    let vf = 20.0;
    let q_max = 1.0;
    let kc2 = 0.10;
    let kj = 0.20;
    let model = TrapezoidalDiagram::new(vf, q_max, kc2, kj);

    // kc1 = q_max / vf = 1.0 / 20.0 = 0.05
    assert!((model.lower_critical_density() - 0.05).abs() < 1e-4);

    // Plateau region: between 0.05 and 0.10, flow is q_max = 1.0
    assert!((model.flow(0.07) - 1.0).abs() < 1e-4);
    assert!((model.speed(0.07) - (1.0 / 0.07)).abs() < 1e-4);

    // Congested branch (k = 0.15): w = 1.0 / (0.20 - 0.10) = 10.0
    let w = model.wave_speed();
    assert!((w - 10.0).abs() < 1e-4);
    assert!((model.flow(0.15) - (10.0 * (0.20 - 0.15))).abs() < 1e-4);
}

#[test]
fn test_flow_model_enum_dispatch() {
    let tri = FlowModel::NewellTriangular(NewellTriangular::standard_urban());
    assert!(tri.free_flow_speed() > 0.0);
    assert!(tri.capacity() > 0.0);
    assert!(tri.jam_density() > 0.0);
}
