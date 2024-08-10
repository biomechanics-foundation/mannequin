//! ## Naming conventions
//!
//! * Traits – adjectives that indicate capability and behavior
//! * Structs – substantives that indicate entities implementing a behavior
//! * Methods – imperative forms with the exception of getters and factories, which
//!             are uses substantives (i.e., omit a `get_` prefix) much like the standard library.
//!             Callback methods have a `on_` prefix

pub mod iterator;
pub mod kinematics;
pub mod visitor;

pub use iterator::TreeIteration;
pub use visitor::visitable::{Accumulable, Visitable, Visiting};
pub use visitor::visiting::Visitor;
