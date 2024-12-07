/// Definition of the interfaces for tree iteration
use thiserror::Error;

#[derive(Error, Debug)]
pub enum MannequinError {
    #[error("Node reference {0} is out of bound")]
    ReferenceOutOfBound(usize),
}

/// Order of iteration
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Order {
    DepthFirst,
    BreadthFirst,
    // Unordered,
}

/// Container that holds data in a `TreeIterable`
pub trait Nodelike<T> {
    fn is_leaf(&self) -> bool;
    fn get(&self) -> &T;

    /// Get the node's distance to its root node. Required for computing accumulations.
    fn depth(&self) -> usize;

    // Note: a get chidren would be useful but it is quite a challenge (open branch for that) to have an
    // associated type `NodeRef`` and enfore equality with the `NodeRef` in `TreeIterable`!.
    // Therefor this has been (will be) added there
}

/// A datastructure to hold a tree hierarchy of data contained in `NodeLike`s.
pub trait TreeIterable<T> {
    type Node: 'static + Nodelike<T>; // cannot hold references
    type NodeRef;

    fn iter<'a, 'b, 'c>(&'a self, traversal: Order, roots: &'b [Self::NodeRef]) -> impl Iterator<Item = &Self::Node>
    where
        'a: 'c,
        'b: 'c;
    fn add(&mut self, load: T, parent: Option<Self::NodeRef>) -> Result<Self::NodeRef, MannequinError>;

    // TODO implement these members
    // fn iter_mut(&self, traversal: Order, root: Option<Self::NodeRef>) -> Self::Iterator;
    // fn pop(&mut self, node_ref: Self::NodeRef) -> Self::Node;
    // fn node(&self, node_ref: Self::NodeRef) -> &Self::Node;
    // fn children(&self, node_ref: Self::NodeRef) -> &[self:NodeRef];
    // fn reorder(self, order: Order) -> Self;
}
