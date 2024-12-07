/*! Implementation of a tree iteration with [arena allocation](https://en.wikipedia.org/wiki/Region-based_memory_management) */

use crate::{
    MannequinError,
    MannequinError::ReferenceOutOfBound,
    Nodelike,
    Order::{self, BreadthFirst, DepthFirst},
    TreeIterable,
};
use core::fmt;

/// Iterator for a depth-first iteration when the data is not already sorted accordingly
pub struct DepthFirstIterator<'a, 'b, T: 'static>
where
    'a: 'b,
{
    tree: &'a ArenaTree<T>,
    stack: Vec<std::slice::Iter<'b, <ArenaTree<T> as TreeIterable<T>>::NodeRef>>,
    // root: &'b Vec<<ArenaTree<T> as IterableTree>::NodeRef>,
    // root2: Vec<<ArenaTree<T> as IterableTree>::NodeRef>,
    // stack2: Vec<std::slice::Iter<'b, <ArenaTree<T> as IterableTree>::NodeRef>>,
}

impl<'a, 'b, T: 'static> DepthFirstIterator<'a, 'b, T> {
    // TODO did not manage to over write the root nodes :(
    pub fn new(tree: &'a ArenaTree<T>, root: &'b [<ArenaTree<T> as TreeIterable<T>>::NodeRef]) -> Self {
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
#[derive(Debug)]
pub struct ArenaNode<T> {
    load: T,
    node_ref: usize,
    children: Vec<usize>,
    /// Only used when data is sorted as it is traversed
    width: usize,
    /// Only used in depth-first interation ([DepthFirstIterator])
    depth: usize,
}

impl<T> ArenaNode<T> {
    fn new(payload: T, node_ref: usize, width: usize, children: Vec<usize>, depth: usize) -> Self {
        ArenaNode {
            load: payload,
            node_ref,
            width,
            children,
            depth,
        }
    }
}

impl<T> Nodelike<T> for ArenaNode<T> {
    fn is_leaf(&self) -> bool {
        self.children.is_empty()
    }

    fn get(&self) -> &T {
        &self.load
    }

    fn depth(&self) -> usize {
        self.depth
    }
}

impl<T> fmt::Display for ArenaNode<T>
where
    T: fmt::Display,
{
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "NodeRef {}, children: {:?}, payload: {} ",
            self.node_ref, self.children, self.load
        )
    }
}

/// Iterable tree that uses arena allocation.
pub struct ArenaTree<T> {
    sorting: Option<Order>,
    nodes: Vec<ArenaNode<T>>,
    roots: Vec<usize>,
    max_depth: usize,
}

impl<T> ArenaTree<T> {
    /// Contructor. Sorting indicates whether the elements are stored to
    /// make either deoth or breadth first traversal efficient (slow insertion). `None` indicates
    /// that the data will be unordered (fast insertion, slower traversal).
    pub fn new(sorting: Option<Order>) -> Self {
        ArenaTree {
            sorting,
            nodes: vec![],
            roots: vec![],
            max_depth: 42,
        }
    }

    /// Inserts a node at an index or pushes it to the end
    fn add_leaf(&mut self, load: T, depth: usize, index: Option<usize>) -> usize {
        let node_ref = if let Some(index) = index {
            index
        } else {
            self.nodes.len()
        };
        self.nodes.push(ArenaNode::new(load, node_ref, 0, vec![], depth));
        node_ref
    }
}

impl<T: 'static> TreeIterable<T> for ArenaTree<T> {
    type Node = ArenaNode<T>;
    type NodeRef = usize;

    fn iter<'a, 'b, 'c>(
        &'a self,
        traversal: Order,
        roots: &'b [Self::NodeRef],
    ) -> Box<dyn Iterator<Item = &Self::Node> + 'c>
    where
        'a: 'c, //  tree outlives the iterator
        'b: 'c, //  sodes outlives the root list
    {
        let roots = if roots.is_empty() { &self.roots } else { roots };

        match (self.sorting, traversal) {
            // (Some(a), b) if a == b => Box::new(self.nodes.iter()), // Big Todo: skip_while and take_while
            (Some(a), b) if a == b => Box::new(roots.iter().flat_map(|root| {
                self.nodes
                    .iter()
                    .enumerate()
                    .skip_while(|(i, _)| i < root)
                    .take_while(|(i, _)| i < &self.nodes.get(*root).expect("Out of bound in managed arena").width)
                    .map(|(_, n)| n)
            })), // Big Todo: skip_while and take_while
            (_, DepthFirst) => Box::new(DepthFirstIterator::new(self, roots)),
            (_, BreadthFirst) => Box::new(BreadthFirstIterator::new(self)),
        }
    }

    fn add(&mut self, load: T, parent: Option<Self::NodeRef>) -> Result<Self::NodeRef, MannequinError> {
        let depth = if let Some(parent_ref) = parent {
            self.nodes
                .get(parent_ref)
                .ok_or(MannequinError::ReferenceOutOfBound(parent_ref))?
                .depth
                + 1
        } else {
            0
        };
        let node_ref = match self.sorting {
            Some(DepthFirst) => {
                if let Some(parent_ref) = parent {
                    let parent = self
                        .nodes
                        .get_mut(parent_ref)
                        .ok_or(MannequinError::ReferenceOutOfBound(parent_ref))?;
                    parent.width += 1;
                    let node_ref = parent_ref + parent.width;
                    // add node to children
                    self.add_leaf(load, depth, Some(node_ref));

                    node_ref
                } else {
                    self.add_leaf(load, depth, None)
                }
            }
            Some(BreadthFirst) => unimplemented!(),
            None => self.add_leaf(load, depth, None),
        };
        if parent.is_none() {
            self.roots.push(node_ref);
        }
        Ok(node_ref)
    }
}

#[cfg(test)]
mod tests {
    use std::result;

    use itertools::Itertools;

    use crate::*;

    #[test]
    fn test_adding_iteration() {
        let mut tree = ArenaTree::<usize>::new(Some(DepthFirst));
        let first = tree.add(1, None).unwrap();
        // Add the second root first to see whether resorting of the vector works
        let second = tree.add(5, None).unwrap();
        let third = tree.add(2, Some(first)).unwrap();
        tree.add(4, Some(first)).unwrap();
        tree.add(3, Some(third)).unwrap();
        tree.add(6, Some(second)).unwrap();
        let result = tree.iter(DepthFirst, &[first, second]).map(|i| *i.get()).collect_vec();
        assert_eq!(result, &[1, 2, 3, 4, 5, 6]);

        // TODO add assert for a subtree traversal
    }

    // TODO add unit test for a `Box`ed node load
}
