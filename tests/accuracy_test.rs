//! Rigorous Traffic Flow Accuracy & Conservation Benchmark Tests
//! Evaluates Rankine-Hugoniot shockwave jump condition, Greenshields extremum, and Newell capacity.

use uxsim_rs::flow_model::{FundamentalDiagram, Greenshields, NewellTriangular};

#[test]
fn test_greenshields_analytical_capacity_extremum() {
    let vf = 25.0f32; // 25 m/s (~90 km/h)
    let kj = 0.25f32; // 250 veh/km
    let model = Greenshields::new(vf, kj);

    // Analytical critical density: kc = kj / 2
    let expected_kc = kj / 2.0;
    assert!((model.critical_density() - expected_kc).abs() < 1e-5);

    // Analytical maximum capacity: q_max = (vf * kj) / 4
    let expected_qmax = (vf * kj) / 4.0;
    assert!((model.capacity() - expected_qmax).abs() < 1e-5);
    assert!((model.flow(expected_kc) - expected_qmax).abs() < 1e-5);

    // Verify first derivative is zero at kc: dq/dk = vf * (1 - 2*k/kj) = 0
    let eps = 1e-4f32;
    let numerical_derivative = (model.flow(expected_kc + eps) - model.flow(expected_kc - eps)) / (2.0 * eps);
    assert!(numerical_derivative.abs() < 1e-3);

    // Parabolic flow symmetry: q(kc - delta) == q(kc + delta)
    for delta in [0.01f32, 0.03, 0.05, 0.08] {
        let q_left = model.flow(expected_kc - delta);
        let q_right = model.flow(expected_kc + delta);
        assert!((q_left - q_right).abs() < 1e-5);
    }
}

#[test]
fn test_rankine_hugoniot_shockwave_jump_condition() {
    let vf = 30.0f32;
    let kj = 0.20f32;
    let model = Greenshields::new(vf, kj);

    // Shockwave between light traffic k1 = 0.04 and congested traffic k2 = 0.16
    let k1 = 0.04f32;
    let k2 = 0.16f32;

    let q1 = model.flow(k1);
    let q2 = model.flow(k2);

    // Analytical Rankine-Hugoniot jump condition: w = (q2 - q1) / (k2 - k1)
    let analytical_w = (q2 - q1) / (k2 - k1);
    let computed_w = model.shockwave_speed(k1, k2);

    assert!((computed_w - analytical_w).abs() < 1e-5);

    // Greenshields specific simplification: w = vf * (1 - (k1 + k2)/kj)
    let simplified_w = vf * (1.0 - (k1 + k2) / kj);
    assert!((computed_w - simplified_w).abs() < 1e-5);
}

#[test]
fn test_newell_triangular_shockwave_conservation() {
    let vf = 20.0f32;
    let w_wave = 5.0f32;
    let kj = 0.20f32;
    let model = NewellTriangular::new(vf, w_wave, kj);

    let kc = model.critical_density();

    // In congested regime (k1, k2 > kc), shockwave speed must be exactly -w_wave:
    let k1 = kc + 0.02;
    let k2 = kc + 0.08;

    let computed_shock = model.shockwave_speed(k1, k2);
    assert!((computed_shock - (-w_wave)).abs() < 1e-5);

    // In free-flow regime (k1, k2 < kc), wave speed must be exactly +vf:
    let k_free1 = 0.01;
    let k_free2 = 0.03;
    let free_shock = model.shockwave_speed(k_free1, k_free2);
    assert!((free_shock - vf).abs() < 1e-5);
}
