//! Provides the error type that will be eventually used
//! throughout this crate.

use thiserror::Error;

/// The error type that will be eventually used
/// throughout this crate
#[derive(Error, Debug)]
pub enum MannequinError<NodeID> {
    // Internal errors
    #[error("Node reference {0} is out of bound")]
    ReferenceOutOfBound(usize),
    #[error("Node not in tree: {0}")]
    UnknownNode(NodeID),
    #[error("No root node set")]
    RootNotSet,
    #[error("ID not unique: {0}")]
    NotUnique(NodeID),
    #[error("Wrong array dimensions: {0}")]
    DimensionMismatch(usize),
    // Errors specific to ndarray
    #[cfg(feature = "ndarray")]
    #[error("Error raised by `ndarray`: ")]
    ShapeError(#[from] ndarray::ShapeError),
    // Add errors specific to nalgebra
    // Add errors specific to faer
}
