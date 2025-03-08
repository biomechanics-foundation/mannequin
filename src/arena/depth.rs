//! Implementations for depth-first traversal, optimazatized trees and tree conversion.

use super::{
    iterables::OptimizedDirectionIterable, utils::sort_by_indices, ArenaIndex, ArenaNode, BaseDirectionIterable,
    DepthFirstIterable, DirectedArenaTree, DirectionIterable,
};
use crate::MannequinError;
use itertools::Itertools;
use std::{fmt::Debug, hash::Hash};

/// Data structure representing an arena tree in which the arena is sorted in depth-first
/// order for faster access
///
/// "Extends" [DirectedArenaTree] by composition.
pub struct DepthFirstArenaTree<Load, NodeId>(DirectedArenaTree<Load, NodeId>);

impl<Load, NodeId> From<DirectedArenaTree<Load, NodeId>> for DepthFirstArenaTree<Load, NodeId>
where
    Load: 'static + Debug + PartialEq,
    NodeId: Eq + 'static + Clone + Hash + Debug,
{
    fn from(mut value: DirectedArenaTree<Load, NodeId>) -> Self {
        // sorts the order of nodes such that depth-first decent is optimal

        let optimal_order = value.iter_depth().map(|node| node.index).collect_vec();

        DirectedArenaTree::update_child_indices(&mut value.nodes, &optimal_order);
        sort_by_indices(&mut value.nodes, optimal_order);

        value.nodes.iter().for_each(|node| {
            value.lookup.insert(node.id.clone(), node.index);
        });
        Self(value)
    }
}

impl<Load, NodeId> BaseDirectionIterable<Load, NodeId> for DepthFirstArenaTree<Load, NodeId>
where
    Load: 'static + Debug + PartialEq,
    NodeId: Eq + 'static + Clone + Hash + Debug,
{
    type Node = ArenaNode<Load, NodeId>;

    fn root(&self) -> Result<&Self::Node, MannequinError<NodeId>> {
        self.0.root()
    }

    fn children(&self, node: &Self::Node) -> Result<Vec<&Self::Node>, MannequinError<NodeId>> {
        self.0.children(node)
    }

    fn node_by_load(&self, load: &Load) -> Option<&Self::Node> {
        self.0.node_by_load(load)
    }

    fn node_by_id(&self, node_id: &NodeId) -> Option<&Self::Node> {
        self.0.node_by_id(node_id)
    }

    fn len(&self) -> usize {
        self.0.len()
    }

    fn is_empty(&self) -> bool {
        self.0.is_empty()
    }
}

impl<Load, NodeId> OptimizedDirectionIterable<Load, NodeId> for DepthFirstArenaTree<Load, NodeId>
where
    Load: 'static + Debug + PartialEq,
    NodeId: Eq + 'static + Clone + Hash + Debug,
{
    fn iter(&self) -> impl Iterator<Item = &Self::Node> {
        self.0.nodes.iter()
    }

    fn iter_mut(&mut self) -> impl Iterator<Item = &mut Self::Node> {
        self.0.nodes.iter_mut()
    }
}

impl<Load, NodeId> DepthFirstIterable<Load, NodeId> for DepthFirstArenaTree<Load, NodeId>
where
    Load: 'static + Debug + PartialEq,
    NodeId: Eq + 'static + Clone + Hash + Debug,
{
    fn iter_sub(&self, root: &Self::Node) -> impl Iterator<Item = &Self::Node> {
        let (start, width) = (root.index, root.width);
        self.0.nodes[start.0..start.0 + width].iter()
    }

    fn iter_sub_mut(&mut self, root: &Self::Node) -> impl Iterator<Item = &mut Self::Node> {
        let (start, width) = (root.index, root.width);
        self.0.nodes[start.0..start.0 + width].iter_mut()
    }
}

/// Iterator for a depth-first iteration over a tree that implements [super::DirectionIterable].
pub struct DepthFirstIterator<'a, 'b, T, N>
where
    'a: 'b,
    T: 'static + Debug + PartialEq,
{
    tree: &'a DirectedArenaTree<T, N>,
    stack: Vec<std::slice::Iter<'b, ArenaIndex>>,
    root: Option<ArenaIndex>,
}

impl<'a, T, N> DepthFirstIterator<'a, '_, T, N>
where
    T: 'static + Debug + PartialEq,
{
    pub fn new(tree: &'a DirectedArenaTree<T, N>, root: ArenaIndex) -> Self {
        let stack = Vec::with_capacity(tree.max_depth);
        println!("Creating new depth-first iterator (slow)");
        DepthFirstIterator {
            tree,
            stack,
            root: Some(root),
        }
    }
}
impl<'a, T, N> Iterator for DepthFirstIterator<'a, '_, T, N>
where
    T: Debug + PartialEq,
{
    type Item = &'a ArenaNode<T, N>;

    fn next(&mut self) -> Option<Self::Item> {
        if let Some(root) = &self.root {
            let root = &self.tree.nodes[root.0];
            self.stack.push(root.children.iter());
            self.root = None;
            Some(root)
        } else if let Some(last) = self.stack.last_mut() {
            if let Some(child_ref) = last.next() {
                let node = &self.tree.nodes[child_ref.0];
                self.stack.push(node.children.iter());
                Some(&self.tree.nodes[child_ref.0])
            } else {
                self.stack.pop();
                self.next()
            }
        } else {
            None
        }
    }
}

#[cfg(test)]
mod tests {

    use super::*;
    use crate::*;
    use arena::directed::ArenaIndex;
    use itertools::Itertools;
    use test_log;

    #[test_log::test]
    fn test_adding_iteration() {
        // Loadas are integers chosen such that after optimization, they are sorted in increasing order
        // IDs are string representations of integers that reflect the order of  insertion
        // Before optimization, the nodes are stored according to the IDs, iteration is ordered by load
        // because of the chosen tree layout. After optimization, the nodes are sorted by load.

        // Layout of the tree (such that optimization will enforce reordering)
        //     0
        //    / \
        //  1    5
        // | \   |
        // 2  4  6
        // |
        // 3

        let mut tree = DirectedArenaTree::<usize, String>::new();

        let root = tree.set_root(0, "root".to_string());

        let first = tree.add(1, "first".to_string(), &root).unwrap();
        let second = tree.add(5, "second".to_string(), &root).unwrap();
        let third = tree.add(2, "third".to_string(), &first).unwrap();

        tree.add(4, "fourth".to_string(), &first).unwrap();
        tree.add(3, "fifth".to_string(), &third).unwrap();
        tree.add(6, "sixth".to_string(), &second).unwrap();

        // Check storage unoptimized for depth-first decent
        assert_eq!(tree.nodes.iter().map(|n| n.load).collect_vec(), &[0, 1, 5, 2, 4, 3, 6]);
        assert_eq!(
            tree.nodes.iter().map(|n| &n.id).collect_vec(),
            &["root", "first", "second", "third", "fourth", "fifth", "sixth"]
        );

        // This uses the depth slow first iterator!
        let result = tree.iter_depth().map(|i| *i.get()).collect_vec();
        assert_eq!(result, &[0, 1, 2, 3, 4, 5, 6]);
        let result = tree.iter_depth().map(|i| i.id()).collect_vec();

        // Check whether iteration is not in the same order as insertion and matches the expectation
        assert_eq!(
            result,
            &["root", "first", "third", "fifth", "fourth", "second", "sixth"]
        );

        // Check correctness of child references for two nodes
        assert_eq!(tree.nodes[0].children, &[ArenaIndex(1), ArenaIndex(2)]);
        assert_eq!(tree.nodes[1].children, &[ArenaIndex(3), ArenaIndex(4)]);
        assert_eq!(tree.nodes[2].children, &[ArenaIndex(6)]);
        assert_eq!(tree.nodes[3].children, &[ArenaIndex(5)]);

        // // Example of how to print the hierarchy
        // tree.nodes
        // .iter()
        // .enumerate()
        // .for_each(|(i, n)| println!("{i}.: id: {}, load: {}, children {:?}", n.id, n.load, n.children));

        // Optimize the tree such that the nodes are sorted in depth-first manner
        let tree: DepthFirstArenaTree<usize, String> = tree.into();

        // Check correctness of child references for two nodes
        assert_eq!(tree.0.nodes[0].children, &[ArenaIndex(1), ArenaIndex(5)]);
        assert_eq!(tree.0.nodes[1].children, &[ArenaIndex(2), ArenaIndex(4)]);
        assert_eq!(tree.0.nodes[2].children, &[ArenaIndex(3)]);
        assert_eq!(tree.0.nodes[5].children, &[ArenaIndex(6)]);

        // check correctness of storage
        assert_eq!(
            tree.0.nodes.iter().map(|n| n.load).collect_vec(),
            &[0, 1, 2, 3, 4, 5, 6]
        );

        // Check whether indices are stored correctly
        assert_eq!(
            tree.0.nodes.iter().map(|n| n.index).collect_vec(),
            &[
                ArenaIndex(0),
                ArenaIndex(1),
                ArenaIndex(2),
                ArenaIndex(3),
                ArenaIndex(4),
                ArenaIndex(5),
                ArenaIndex(6)
            ]
        );

        assert_eq!(
            tree.0.nodes.iter().map(|n| &n.id).collect_vec(),
            &["root", "first", "third", "fifth", "fourth", "second", "sixth"]
        );

        // Check whether the widths are computed correctly
        let result = tree.0.nodes.iter().map(|n| n.width).collect_vec();
        assert_eq!(result, &[7, 4, 2, 1, 1, 2, 1]);

        // Now we can safely immutably borrow the node
        let first_node = tree.node_by_id(&first).unwrap();

        // Now check whether iterating over subtrees work
        let result = tree.iter_sub(first_node).map(|i| *i.get()).collect_vec();
        assert_eq!(result, &[1, 2, 3, 4]);
        let second_node = tree.node_by_id(&second).unwrap();

        let result = tree.iter_sub(second_node).map(|i| *i.get()).collect_vec();
        assert_eq!(result, &[5, 6]);
    }

    #[test]
    fn test_iter_mut() {
        // TODO implement test for mutable iteration
    }
}
