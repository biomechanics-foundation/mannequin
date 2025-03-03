use thiserror::Error;

#[derive(Error, Debug)]
pub enum MannequinError<NodeID> {
    // Internal errors
    #[error("Node reference {0} is out of bound")]
    ReferenceOutOfBound(usize),
    #[error("Node not in tree: {0}")]
    UnkonwnNode(NodeID),
    #[error("No root node set")]
    RootNotSet,
    #[error("ID not unique: {0}")]
    NotUnique(NodeID),
    #[error("Wrong array dimensions: {0}")]
    DimensionMismatch(usize),
    // Errors specific to ndarray
    #[cfg(feature = "ndarray")]
    #[error("Error raised by NDArray: ")]
    ShapeError(#[from] ndarray::ShapeError),
    // Add errors specific to nalgebra
    // Add errors specific to faer
}
