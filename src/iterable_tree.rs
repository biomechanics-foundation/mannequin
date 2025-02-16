//! Definition of the interfaces for tree iteration
use std::{fmt::Debug, hash::Hash};
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
    // TODO Errors specific to nalgebra
    // TODO Errors specific to faer
}

/// Order of iteration
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Order {
    DepthFirst,
    BreadthFirst,
    // Unordered,
}

/// Container that holds data in a `TreeIterable`
pub trait Nodelike<Load, NodeId> {
    fn is_leaf(&self) -> bool;
    fn get(&self) -> &Load;

    fn id(&self) -> NodeId;
    /// Get the node's distance to its root node. Required for computing accumulations.
    fn depth(&self) -> usize;

    // Note: a get chidren would be useful but it is quite a challenge (open branch for that) to have an
    // associated type `NodeRef`` and enfore equality with the `NodeRef` in `TreeIterable`!.
    // Therefor this has been (will be) added there
}

trait BaseDirectionIterable<Load, NodeId>
where
    Load: PartialEq,
    NodeId: Eq + Clone + Hash + Debug,
{
    type Node: 'static + Nodelike<Load, NodeId> + Debug; // cannot hold references

    fn root(&self) -> Result<&Self::Node, MannequinError<NodeId>>;
    fn children(&self, node: &Self::Node) -> Result<Vec<&Self::Node>, MannequinError<NodeId>>;
    fn node_by_load(&self, load: &Load) -> Option<&Self::Node>;
    fn node_by_id(&self, node_id: &NodeId) -> Option<&Self::Node>;
}

/// A datastructure to hold a tree hierarchy of data contained in `NodeLike`s.
///
/// TODO Put an emphasis on that mutability is tough. after the tree has been constructed
/// it is advised to not access it in a mutable way (because of borrowed node references)
pub trait DirectionIterable<Load, NodeId>: BaseDirectionIterable<Load, NodeId>
where
    Load: PartialEq,
    NodeId: Eq + Clone + Hash,
{
    fn iter_depth(&self) -> impl Iterator<Item = &Self::Node>;
    fn iter_depth_sub(&self, root: &Self::Node) -> impl Iterator<Item = &Self::Node>;

    fn iter_breadth(&self) -> impl Iterator<Item = &Self::Node>;
    fn iter_breadth_sub(&self, root: &Self::Node) -> impl Iterator<Item = &Self::Node>;

    // Leave in for historic reasons .. until documented how to return Boxed iterators
    // fn iter<'a, 'b>(&'a self, traversal: Order, root: Option<&Self::Node>) -> impl Iterator<Item = &'a Self::Node>
    // where
    //     'a: 'b;

    // Modifying methods

    /// Add a new node to the tree. A tree can have multiple root nodes; their parents are `None`
    fn add(&mut self, load: Load, node_ref: NodeId, parent: &NodeId) -> Result<NodeId, MannequinError<NodeId>>;

    /// Deletes all nodes and sets a new root
    fn set_root(&mut self, root_load: Load, root_ref: NodeId) -> NodeId;

    /// Generate optimized
    fn to_depth_first(self) -> impl DepthFirstIterable<Load, NodeId>;
    fn to_breadth_first(self) -> impl BreadthFirstIterable<Load, NodeId>;
}

pub trait OptimizedDirectionIterable<Load, NodeId>: BaseDirectionIterable<Load, NodeId>
where
    Load: PartialEq,
    NodeId: Eq + Clone + Hash,
{
    fn iter(&self) -> impl Iterator<Item = &Self::Node>;
    fn iter_mut(&self) -> impl Iterator<Item = &mut Self::Node>;
    fn iter_sub(&self, root: &Self::Node) -> impl Iterator<Item = &Self::Node>;
    // That one maybe not :-(
    fn iter_sub_mut(&self, root: &Self::Node) -> impl Iterator<Item = &mut Self::Node>;
}

pub trait DepthFirstIterable<Load, NodeId>: OptimizedDirectionIterable<Load, NodeId>
where
    Load: PartialEq,
    NodeId: Eq + Clone + Hash,
{
    fn test() -> Self::Node;
}

pub trait BreadthFirstIterable<Load, NodeId>: OptimizedDirectionIterable<Load, NodeId>
where
    Load: PartialEq,
    NodeId: Eq + Clone + Hash,
{
    // Either copy or make extending traits with no additional methods
}
