//! ## Naming conventions
//!
//! * Traits – adjectives that indicate capability and behavior (TODO this is currently not always true)
//! * Structs – substantives that indicate entities implementing a behavior
//! * Methods – imperative forms with the exception of getters and factories, which
//!             are uses substantives (i.e., omit a `get_` prefix) much like the standard library.
//!             Callback methods have a `on_` prefix

pub mod arena;
pub mod dummy;
pub mod iterable_tree;
pub mod mannequin;
pub mod rigid;

pub use arena::{ArenaNode, ArenaTree};
pub use iterable_tree::{
    Nodelike, Order,
    Order::{BreadthFirst, DepthFirst, Unordered},
    TreeIterable,
};
pub use mannequin::{Forward, Inverse, Mannequin};
pub use rigid::{accumulate, Accumulator, Rigid};

// Backends
#[cfg(feature = "ndarray")]
pub mod ndarray;
