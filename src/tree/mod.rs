//! # Nomenclature
//!
//! * Traits are adjectives that describe a property
//! * Structs are nouns. They may directly relate to the trait they implement
//!
//! In this module, trees cannot be iterated but can be *visited*. They therefore
//! implement the visitable trait. When visiting, a visitor object is created
//! that represents the (depth-first) traversal, it implements the visiting trait.

pub mod iterator;
pub mod visitable;
pub mod visiting;
