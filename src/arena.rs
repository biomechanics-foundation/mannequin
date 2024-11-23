use crate::{IterableTree, Node, Order};

pub struct DepthFirstIterator<'a, 'b, T>
where
    'a: 'b,
{
    tree: &'a ArenaTree<T>,
    stack: Vec<std::slice::Iter<'b, <ArenaTree<T> as IterableTree<T>>::NodeRef>>,
    // root: &'b Vec<<ArenaTree<T> as IterableTree>::NodeRef>,
    // root2: Vec<<ArenaTree<T> as IterableTree>::NodeRef>,
    // stack2: Vec<std::slice::Iter<'b, <ArenaTree<T> as IterableTree>::NodeRef>>,
}

impl<'a, 'b, T> DepthFirstIterator<'a, 'b, T> {
    // TODO did not manage to over write the root nodes :(
    pub fn new(tree: &'a ArenaTree<T>, root: &'b [<ArenaTree<T> as IterableTree<T>>::NodeRef]) -> Self {
        let mut stack = Vec::with_capacity(tree.max_depth);
        stack.push(root.iter());
        DepthFirstIterator { tree, stack }
    }
}
impl<'a, 'b, T> Iterator for DepthFirstIterator<'a, 'b, T> {
    type Item = &'a ArenaNode<T>;

    fn next(&mut self) -> Option<Self::Item> {
        if let Some(last) = self.stack.last_mut() {
            if let Some(child_ref) = last.next() {
                let node = &self.tree.nodes[*child_ref];
                self.stack.push(node.children.iter());
                Some(&self.tree.nodes[*child_ref])
            } else {
                self.stack.pop();
                self.next()
            }
        } else {
            None
        }
    }
}
struct BreadthFirstIterator<'a, T> {
    tree: &'a ArenaTree<T>,
}

impl<'a, T> BreadthFirstIterator<'a, T> {
    pub fn new(tree: &'a ArenaTree<T>) -> Self {
        BreadthFirstIterator { tree }
    }
}
impl<'a, T> Iterator for BreadthFirstIterator<'a, T> {
    type Item = &'a ArenaNode<T>;

    fn next(&mut self) -> Option<Self::Item> {
        todo!()
    }
}

// implement deref for this or add a getter
pub struct ArenaNode<T> {
    payload: T,
    node_ref: usize,
    next_sibling: usize,
    children: Vec<usize>,
}

impl<T> Node<T> for ArenaNode<T> {
    fn is_leaf(&self) -> bool {
        self.children.is_empty()
    }

    fn get(&self) -> &T {
        &self.payload
    }
}

// we can probably have a single implementation for
pub struct ArenaTree<T> {
    sorting: Order,
    nodes: Vec<ArenaNode<T>>,
    roots: Vec<<ArenaTree<T> as IterableTree<T>>::NodeRef>,
    max_depth: usize,
}

impl<T> ArenaTree<T> {
    pub fn new(sorting: Order) -> Self {
        ArenaTree {
            sorting,
            nodes: vec![],
            roots: vec![],
            max_depth: 42,
        }
    }
}

impl<T> IterableTree<T> for ArenaTree<T> {
    type Node = ArenaNode<T>;
    type NodeRef = usize;

    fn iter<'a, 'b>(&'a self, traversal: Order, root: &'b [Self::NodeRef]) -> Box<dyn Iterator<Item = &Self::Node> + 'b>
    where
        'a: 'b,
    {
        let root = if root.is_empty() { &self.roots } else { root };

        match (self.sorting, traversal) {
            (_, Order::Unordered) => Box::new(self.nodes.iter()), // Probably not a good idea—when would you stop?
            (a, b) if a == b => Box::new(self.nodes.iter()),      // Big Todo: skip_while and take_while
            (_, Order::DepthFirst) => Box::new(DepthFirstIterator::new(self, root)),
            (_, Order::BreadthFirst) => Box::new(BreadthFirstIterator::new(self)),
        }
    }
}
