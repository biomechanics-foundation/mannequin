//! Definition of the traits of iterable trees.

use crate::MannequinError;
use std::{fmt::Debug, hash::Hash};

/// Container that holds data in a [BaseDirectionIterable]
/// Note: You probably do not need to implement this trait.
/// Implementations of trees have [NodeLike]s that own your data.
pub trait Nodelike<Load, NodeId> {
    fn is_leaf(&self) -> bool;
    fn get(&self) -> &Load;

    fn id(&self) -> NodeId;
    /// Get the node's distance to its root node. Required for computing accumulations.
    fn depth(&self) -> usize;

    // Note: a get chidren would be useful but it is quite a challenge
    // (enforcing equality of associated types). Simpler to implement on [BaseDirectionIterable]
}

/// (Abstract) Basis trait for a tree structure common to [DirectionIterable], [DepthFirstIterable], and
/// [BreadthFirstIterable].
pub trait BaseDirectionIterable<Load, NodeId>
where
    Load: PartialEq,
    NodeId: Eq + Clone + Hash + Debug,
{
    type Node: 'static + Nodelike<Load, NodeId> + Debug; // cannot hold references

    /// Get the (single) root node of the tree.
    fn root(&self) -> Result<&Self::Node, MannequinError<NodeId>>;
    /// Access the children of a node. Not implemented on [Nodelike] for simplicity
    fn children(&self, node: &Self::Node) -> Result<Vec<&Self::Node>, MannequinError<NodeId>>;
    /// Lookup a node by its load.
    fn node_by_load(&self, load: &Load) -> Option<&Self::Node>;
    /// Get [NodeLike] from an identifier.
    fn node_by_id(&self, node_id: &NodeId) -> Option<&Self::Node>;
}

/// Trait for a mutable tree that can be iterated (traverserd) in both directions: depth-first and
/// breadth first. The immutable, optimized versions [DepthFirstIterable] and [BreadthFirstIterable]
/// are generated from structs that implement this trait.
pub trait DirectionIterable<Load, NodeId>: BaseDirectionIterable<Load, NodeId>
where
    Load: PartialEq,
    NodeId: Eq + Clone + Hash + Debug,
{
    //
    fn iter_depth(&self) -> impl Iterator<Item = &Self::Node>;
    fn iter_depth_sub(&self, root: &Self::Node) -> impl Iterator<Item = &Self::Node>;

    fn iter_breadth(&self) -> impl Iterator<Item = &Self::Node>;
    fn iter_breadth_sub(&self, root: &Self::Node) -> impl Iterator<Item = &Self::Node>;

    /// Add a new node to the tree. A tree can have multiple root nodes; their parents are `None`
    fn add(&mut self, load: Load, node_id: NodeId, parent: &NodeId) -> Result<NodeId, MannequinError<NodeId>>;

    /// Deletes all nodes and sets a new root
    fn set_root(&mut self, root_load: Load, root_ref: NodeId) -> NodeId;

    /// Generate optimized
    fn depth_first(self) -> impl DepthFirstIterable<Load, NodeId>;
    // TODO: breadth-first implementation
    // fn breadth_first(self) -> impl BreadthFirstIterable<Load, NodeId>;
}

/// (Abstract) Base trait for [DepthFirstIterable] and [BreadthFirstIterable]
pub trait OptimizedDirectionIterable<Load, NodeId>: BaseDirectionIterable<Load, NodeId>
where
    Load: PartialEq,
    NodeId: Eq + Clone + Hash + Debug,
{
    fn iter(&self) -> impl Iterator<Item = &Self::Node>;
    fn iter_mut(&mut self) -> impl Iterator<Item = &mut Self::Node>;
    // TODO: As these trees are mutuable (i.e., no nodes can be added), we can use the arena
    // index for much faster lookups. Hashmaps are slow!
}

/// An immutable (in a sense of modifying the tree by adding nodes) depth-first itrable/traversable
/// tree optimized for fast traversal (no index lookups)
pub trait DepthFirstIterable<Load, NodeId>: OptimizedDirectionIterable<Load, NodeId>
where
    Load: PartialEq,
    NodeId: Eq + Clone + Hash + Debug,
{
    fn iter_sub(&self, root: &Self::Node) -> impl Iterator<Item = &Self::Node>;
    fn iter_sub_mut(&mut self, root: &Self::Node) -> impl Iterator<Item = &mut Self::Node>;
}

/// An immutable (in a sense of modifying the tree by adding nodes) breadth-first itrable/traversable
/// tree optimized for fast traversal (no index lookups)
pub trait BreadthFirstIterable<Load, NodeId>: OptimizedDirectionIterable<Load, NodeId>
where
    Load: PartialEq,
    NodeId: Eq + Clone + Hash + Debug,
{
    // subtree iteration more difficult in bredth-first ordering. This trait likely remains empty
}
