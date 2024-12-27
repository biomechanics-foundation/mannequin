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
pub trait TreeIterable<T: PartialEq> {
    type Node: 'static + Nodelike<T>; // cannot hold references
    type NodeRef;

    fn iter<'a, 'b, 'c>(&'a self, traversal: Order, roots: &'b [Self::NodeRef]) -> impl Iterator<Item = &'a Self::Node>
    where
        'a: 'c,
        'b: 'c;

    /// Add a new node to the tree. A tree can have multiple root nodes; their parents are `None`
    fn add(&mut self, load: T, parent: Option<Self::NodeRef>) -> Result<Self::NodeRef, MannequinError>;

    /// Optimizes the tree for a traversal order after all nodes have been added. May also
    /// generate caches if applicable.
    /// **Warning** breaks all existing references in your program!
    fn optimize(&mut self, for_traversal: Order);

    fn get_ref(&self, node: &Self::Node) -> Self::NodeRef;
    fn get_ref_by_load(&self, load: &T) -> Option<Self::NodeRef>;
    fn get_node_by_ref(&self, node_ref: &Self::NodeRef) -> Option<&Self::Node>;

    // TODO implement these members
    // fn iter_mut(&self, traversal: Order, root: Option<Self::NodeRef>) -> Self::Iterator;
    // fn pop(&mut self, node_ref: Self::NodeRef) -> Self::Node;
    // fn node(&self, node_ref: Self::NodeRef) -> &Self::Node;
    // fn children(&self, node_ref: Self::NodeRef) -> &[self:NodeRef];
    // fn reorder(self, order: Order) -> Self;
}

/// Accumulation over a *Depth First* iteration (this is not checked).
pub trait DepthFirstAccumulation<'a, Node, T, F, Parameter, Accumulated>
where
    Node: Nodelike<T> + 'a,
    F: Fn(&Node, &Parameter, &Accumulated) -> Accumulated + 'static,
    Accumulated: Clone,
{
    /// Method on an iterator. Accepts a slice of parameters used to
    /// compute a value that can be accumulated considering the depth of the tree.
    /// The latter means that each node receives the accumulated values from its parent.
    /// The calculation of the value and the accumulation is specified by the `accumulator`
    /// parameter (usually, a closure). Accumulation requires a `first` element (neutral element)
    /// and a max depth (for performance)
    fn accumulate(
        self,
        param: &[Parameter],
        accumulator: F,
        first: Accumulated,
        max_depth: usize,
    ) -> impl Iterator<Item = (&'a Node, Accumulated)>;
}

impl<'a, 'b, N, T, F, P, R> DepthFirstAccumulation<'a, N, T, F, P, R> for Box<dyn Iterator<Item = &'a N> + 'b>
where
    N: Nodelike<T> + 'a,
    F: Fn(&N, &P, &R) -> R + 'static,
    R: Clone,
{
    fn accumulate(self, param: &[P], acc: F, first: R, max_depth: usize) -> impl Iterator<Item = (&'a N, R)> {
        self.into_iter()
            .zip(param.iter())
            // TODO look up whether the move here (required for the acc parameter) slows down execution
            // TODO this function should replace the accumulation in `rigid.rs`
            .scan(Vec::<R>::with_capacity(max_depth), move |stack, (node, param)| {
                let depth = stack.len();
                while node.depth() < depth {
                    stack.pop();
                }
                let result = acc(node, param, stack.last().unwrap_or(&first));
                stack.push(result.clone());
                Some((node, result))
            })
    }
}
