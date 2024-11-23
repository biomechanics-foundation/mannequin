#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Order {
    DepthFirst,
    BreadthFirst,
    Unordered,
}

pub trait Node<T> {
    fn is_leaf(&self) -> bool;
    fn get(&self) -> &T;
}

pub trait IterableTree<T> {
    type Node: Node<T>;
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
    // fn get(&self, node_ref: Self::NodeRef) -> &Self::Node;
    // fn reorder(self, order: Order) -> Self;
}
