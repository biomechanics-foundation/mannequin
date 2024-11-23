/*! Implementation of a tree iteration with [arena allocation](https://en.wikipedia.org/wiki/Region-based_memory_management) */

// TODO: split this up into two files and a separate folder

use crate::{Nodelike, Order, TreeIterable};

/// Iterator for a depth-first iteration when the data is not already sorted accordingly
pub struct DepthFirstIterator<'a, 'b, T>
where
    'a: 'b,
{
    tree: &'a ArenaTree<T>,
    stack: Vec<std::slice::Iter<'b, ArenaNodeRef<T>>>,
    // root: &'b Vec<<ArenaTree<T> as IterableTree>::NodeRef>,
    // root2: Vec<<ArenaTree<T> as IterableTree>::NodeRef>,
    // stack2: Vec<std::slice::Iter<'b, <ArenaTree<T> as IterableTree>::NodeRef>>,
}

impl<'a, 'b, T> DepthFirstIterator<'a, 'b, T> {
    // TODO did not manage to over write the root nodes :(
    pub fn new(tree: &'a ArenaTree<T>, root: &'b [ArenaNodeRef<T>]) -> Self {
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

/// Iterator for a breadth-first iteration when the data is not already sorted accordingly

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

/// A node structure to be used in an arena allocated tree. Fields are used to speed up iteration
pub struct ArenaNode<T> {
    payload: T,
    node_ref: usize,
    next_sibling: usize,
    children: Vec<usize>,
}

impl<T> Nodelike<T> for ArenaNode<T> {
    fn is_leaf(&self) -> bool {
        self.children.is_empty()
    }

    fn get(&self) -> &T {
        &self.payload
    }

    type NodeRef = usize;

    fn get_ref(&self) -> Self::NodeRef {
        todo!()
    }

    fn children(&self) -> &[Self::NodeRef] {
        todo!()
    }
}

/// Iterable tree that uses arena allocation.
pub struct ArenaTree<T> {
    sorting: Order,
    nodes: Vec<ArenaNode<T>>,
    roots: Vec<usize>,
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

impl<T> TreeIterable<ArenaNode<T>, T> for ArenaTree<T> {
    type NodeRef = usize;

    fn iter<'a, 'b>(
        &'a self,
        traversal: Order,
        root: &'b [Self::NodeRef],
    ) -> Box<dyn Iterator<Item = &ArenaNode<T>> + 'b>
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

/// Shortcut for Using the Noderef which unfortunately got pretty complex.
type ArenaNodeRef<T> = <ArenaTree<T> as TreeIterable<ArenaNode<T>, T>>::NodeRef;
