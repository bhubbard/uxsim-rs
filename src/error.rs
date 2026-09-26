//! Error types for uxsim-rs.

use thiserror::Error;

/// Result alias for uxsim operations.
pub type Result<T> = std::result::Result<T, UxsimError>;

/// Errors that can occur during traffic simulation, network creation, and routing.
#[derive(Debug, Error, Clone, PartialEq)]
pub enum UxsimError {
    #[error("Node not found: {0}")]
    NodeNotFound(u64),

    #[error("Link not found: {0}")]
    LinkNotFound(u64),

    #[error("Vehicle not found: {0}")]
    VehicleNotFound(u64),

    #[error("Node already exists: {0}")]
    DuplicateNode(u64),

    #[error("Link already exists: {0}")]
    DuplicateLink(u64),

    #[error("Vehicle already exists: {0}")]
    DuplicateVehicle(u64),

    #[error("No path found between origin {0} and destination {1}")]
    NoPathFound(u64, u64),

    #[error("Invalid parameter: {0}")]
    InvalidParameter(String),

    #[error("Link {0} is at full capacity / jammed")]
    LinkJammed(u64),
}
