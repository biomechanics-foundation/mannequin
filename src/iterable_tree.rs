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
    type NodeRef;

    fn is_leaf(&self) -> bool;
    fn get(&self) -> &T;
    fn get_ref(&self) -> Self::NodeRef;
    fn children(&self) -> &[Self::NodeRef];
}

/// A datastructure to hold a tree hierarchy of data contained in `NodeLike`s.
pub trait TreeIterable<Node, T>
where
    Node: Nodelike<T, NodeRef = Self::NodeRef>,
{
    type NodeRef;

    fn iter<'a, 'b>(&'a self, traversal: Order, root: &'b [Self::NodeRef]) -> Box<dyn Iterator<Item = &Node> + 'b>
    where
        'a: 'b;
    // fn iter_mut(&self, traversal: Order, root: Option<Self::NodeRef>) -> Self::Iterator;
    // fn add(&mut self, node: Self::Node, parent: Option<Self::Node>) -> Self::NodeRef;
    // fn pop(&mut self, node_ref: Self::NodeRef) -> Self::Node;
    // fn node(&self, node_ref: Self::NodeRef) -> &Self::Node;
    // fn children(&self, node_ref: Self::NodeRef) -> &[self:NodeRef];
    // fn reorder(self, order: Order) -> Self;
}
