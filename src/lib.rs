//! ## About
//!
//! **Warning: still under heavy development**
//!
//! This crate contains methods for kinematics and rigging, and dynamics in biomechanics applications
//! and animation. It contains data structures and visitor algorithms for handling tree structures, and
//! will support multiple algebra packages (planned: [ndarray](https://github.com/rust-ndarray/ndarray),
//! [ngalgebra](https://github.com/dimforge/nalgebra), [faer](https://github.com/sarah-ek/faer-rs),
//! [glam](https://github.com/bitshifter/glam-rs))
//!
//! See the [Mannequin] struct to get started.
//!
//! ## Reading list
//!
//! * [Lecture on Inverse Kinematics](https://cseweb.ucsd.edu/classes/wi17/cse169-a/sessions.html)
//! * [Dynamics in Cumputer Graphics](https://foswiki.cs.rpi.edu/foswiki/pub/RoboticsWeb/LabPublications/BETCstar_part1.pdf)
//!
//! ## Naming conventions
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
