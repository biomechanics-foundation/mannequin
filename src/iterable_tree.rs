/// Definition of the interfaces for tree iteration

/// Order of iteration
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Order {
    DepthFirst,
    BreadthFirst,
    Unordered,
}

/// Container that holds data in a `TreeIterable`
pub trait Nodelike<T> {
    fn is_leaf(&self) -> bool;
    fn get(&self) -> &T;

    // Note: a get chidren would be useful but it is quite a challenge to have an
    // associated type `NodeRef`` and enfore equality with the `NodeRef` in `TreeIterable`!.
    // Therefor this has been (will be) added there
}

/// A datastructure to hold a tree hierarchy of data contained in `NodeLike`s.
pub trait TreeIterable<T> {
    type Node: Nodelike<T>;
    type NodeRef;

    fn iter<'a, 'b>(
        &'a self,
        traversal: Order,
        root: &'b [Self::NodeRef],
    ) -> Box<dyn Iterator<Item = &Self::Node> + 'b>
    where
        'a: 'b;
    // fn iter_mut(&self, traversal: Order, root: Option<Self::NodeRef>) -> Self::Iterator;
    // fn add(&mut self, node: Self::Node, parent: Option<Self::Node>) -> Self::NodeRef;
    // fn pop(&mut self, node_ref: Self::NodeRef) -> Self::Node;
    // fn node(&self, node_ref: Self::NodeRef) -> &Self::Node;
    // fn children(&self, node_ref: Self::NodeRef) -> &[self:NodeRef];
    // fn reorder(self, order: Order) -> Self;
}
