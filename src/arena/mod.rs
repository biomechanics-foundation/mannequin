//! [Arena allocated](https://en.wikipedia.org/wiki/Region-based_memory_management)
//! tree structure for fast, directional (i.e., breadth-first/depth-first) traversal.

pub mod breadth;
pub mod depth;
pub mod directed;
pub mod iterables;
mod utils;

pub use breadth::BreadthFirstIterator;
pub use depth::{DepthFirstArenaTree, DepthFirstIterator};
pub use directed::{ArenaIndex, ArenaNode, DirectedArenaTree};
use iterables::BaseDirectionIterable;
pub use iterables::{BreadthFirstIterable, DepthFirstIterable, DirectionIterable, NodeLike};
