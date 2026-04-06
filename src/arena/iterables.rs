//! Definitions of all the traits for iterable trees in this crate.

use crate::MannequinError;
use std::{fmt::Debug, hash::Hash};

/// A tree node, that is a, Container that holds arbitrary data. It is implemented
/// by [super::ArenaNode] which is used exclusively throughout this crate.
pub trait NodeLike<LoadType, IdType> {
    fn is_leaf(&self) -> bool;
    fn get(&self) -> &LoadType;
    fn get_mut(&mut self) -> &mut LoadType;

    fn id(&self) -> &IdType;
    /// Get the node's distance to its root node. Required for computing accumulations.
    fn depth(&self) -> usize;

    // TODO Note: a get children would be useful but it is quite a challenge
    // (enforcing equality of associated types). Simpler to implement on [BaseDirectionIterable]
}

/// (Abstract) Basis trait for a tree structure common to [DirectionIterable], [DepthFirstIterable], and
/// [BreadthFirstIterable].
pub trait BaseDirectionIterable<LoadType, IdType>
where
    LoadType: PartialEq,
    IdType: Eq + Clone + Hash + Debug,
{
    type Node: 'static + NodeLike<LoadType, IdType> + Debug; // cannot hold references

    /// Get the (single) root node of the tree.
    fn root(&self) -> Result<&Self::Node, MannequinError<IdType>>;
    /// Access the children of a node. Not implemented on [NodeLike] for simplicity
    fn children(&self, node: &Self::Node) -> Result<Vec<&Self::Node>, MannequinError<IdType>>;
    /// Lookup a node by its load.
    fn node_by_load(&self, load: &LoadType) -> Option<&Self::Node>;
    /// Get [NodeLike] from an identifier.
    fn node_by_id(&self, node_id: &IdType) -> Option<&Self::Node>;

    /// Get the number of nodes.
    fn len(&self) -> usize;
    /// Returns whether the tree contains any nodes.
    fn is_empty(&self) -> bool;
}

/// Trait for a mutable tree that can be iterated (traverserd) in both directions: depth-first and
/// breadth first. It is implemented by [super::DirectedArenaTree] and [super::DepthFirstArenaTree]
/// which are used exclusively throughout this crate.
pub trait DirectionIterable<LoadType, IdType>: BaseDirectionIterable<LoadType, IdType>
where
    LoadType: PartialEq,
    IdType: Eq + Clone + Hash + Debug,
{
    /// Depth-first iteration.
    fn iter_depth(&self) -> impl Iterator<Item = &Self::Node>;
    /// Depth-first iteration of a subtree.
    fn iter_depth_sub(&self, root: &Self::Node) -> impl Iterator<Item = &Self::Node>;

    /// **TBD** Breadth-first iteration.
    fn iter_breadth(&self) -> impl Iterator<Item = &Self::Node>;
    /// **TBD** Breadth-first iteration of a subtree.
    fn iter_breadth_sub(&self, root: &Self::Node) -> impl Iterator<Item = &Self::Node>;

    /// Add a new node to the tree. A tree can have multiple root nodes; their parents are `None`
    fn add(&mut self, load: LoadType, node_id: IdType, parent: &IdType) -> Result<IdType, MannequinError<IdType>>;

    /// Deletes all nodes and sets a new root
    fn set_root(&mut self, root_load: LoadType, root_ref: IdType) -> IdType;

    /// Generate optimized
    fn depth_first(self) -> impl DepthFirstIterable<LoadType, IdType>;
    // TODO: breadth-first implementation
    // fn breadth_first(self) -> impl BreadthFirstIterable<Load, NodeId>;
}

/// (Abstract) Base trait for [DepthFirstIterable] and [BreadthFirstIterable]
pub trait OptimizedDirectionIterable<LoadType, IdType>: BaseDirectionIterable<LoadType, IdType>
where
    LoadType: PartialEq,
    IdType: Eq + Clone + Hash + Debug,
{
    fn iter(&self) -> impl Iterator<Item = &Self::Node>;
    fn iter_mut(&mut self) -> impl Iterator<Item = &mut Self::Node>;
    // FIXME: As these trees are mutuable (i.e., no nodes can be added), we can use the arena
    // index for much faster lookups. Hashmaps are slow!
}

/// An immutable (in a sense of modifying the tree by adding nodes) depth-first itrable/traversable
/// tree optimized for fast traversal (no index lookups)
pub trait DepthFirstIterable<LoadType, IdType>: OptimizedDirectionIterable<LoadType, IdType>
where
    LoadType: PartialEq,
    IdType: Eq + Clone + Hash + Debug,
{
    fn iter_sub(&self, root: &Self::Node) -> impl Iterator<Item = &Self::Node>;
    fn iter_sub_mut(&mut self, root: &Self::Node) -> impl Iterator<Item = &mut Self::Node>;
}

/// An immutable (in a sense of modifying the tree by adding nodes) breadth-first itrable/traversable
/// tree optimized for fast traversal (no index lookups)
pub trait BreadthFirstIterable<LoadType, IdType>: OptimizedDirectionIterable<LoadType, IdType>
where
    LoadType: PartialEq,
    IdType: Eq + Clone + Hash + Debug,
{
    // subtree iteration more difficult in bredth-first ordering. This trait likely remains empty
}
