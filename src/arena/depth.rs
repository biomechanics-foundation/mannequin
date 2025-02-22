//! Data structure representing an arena tree in which the arena is sorted in depth-frist
//! order for faster access

use super::{
    iterables::OptimizedDirectionIterable, utils::sort_by_indices, ArenaNode, BaseDirectionIterable,
    DepthFirstIterable, DirectedArenaTree, DirectionIterable, Nodelike,
};
use crate::MannequinError;
use itertools::Itertools;
use std::{fmt::Debug, hash::Hash};

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

    fn nodes(&self) -> &[Self::Node] {
        self.0.nodes()
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
        self.0.nodes[start..start + width].iter()
    }

    fn iter_sub_mut(&mut self, root: &Self::Node) -> impl Iterator<Item = &mut Self::Node> {
        let (start, width) = (root.index, root.width);
        self.0.nodes[start..start + width].iter_mut()
    }
}

// TODO the following iterator should be
// * checked whether it is needed
// * implemented for not a box
// * we could directly add accumulate to the depthfirst trait??

/// Accumulation over a *Depth First* iteration (this is not checked).
pub trait DepthFirstAccumulation<'a, Node, Load, F, Parameter, Accumulated, NodeRef>
where
    Node: Nodelike<Load, NodeRef> + 'a,
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

impl<'a, 'b, Node, Load, F, Parameter, Accumulated, NodeRef>
    DepthFirstAccumulation<'a, Node, Load, F, Parameter, Accumulated, NodeRef>
    for Box<dyn Iterator<Item = &'a Node> + 'b>
where
    Node: Nodelike<Load, NodeRef> + 'a,
    F: Fn(&Node, &Parameter, &Accumulated) -> Accumulated + 'static,
    Accumulated: Clone,
{
    fn accumulate(
        self,
        param: &[Parameter],
        acc: F,
        first: Accumulated,
        max_depth: usize,
    ) -> impl Iterator<Item = (&'a Node, Accumulated)> {
        self.into_iter()
            .zip(param.iter())
            // TODO look up whether the move here (required for the acc parameter) slows down execution
            // TODO this function should replace the accumulation in `rigid.rs`
            .scan(
                Vec::<Accumulated>::with_capacity(max_depth),
                move |stack, (node, param)| {
                    let depth = stack.len();
                    while node.depth() < depth {
                        stack.pop();
                    }
                    let result = acc(node, param, stack.last().unwrap_or(&first));
                    stack.push(result.clone());
                    Some((node, result))
                },
            )
    }
}

#[cfg(test)]
mod tests {

    use super::*;
    use crate::*;
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
        assert_eq!(tree.nodes()[0].children, &[1, 2]);
        assert_eq!(tree.nodes()[1].children, &[3, 4]);
        assert_eq!(tree.nodes()[2].children, &[6]);
        assert_eq!(tree.nodes()[3].children, &[5]);

        // // Example of how to print the hierachy
        // tree.nodes
        // .iter()
        // .enumerate()
        // .for_each(|(i, n)| println!("{i}.: id: {}, load: {}, children {:?}", n.id, n.load, n.children));

        // Optimize the tree such that the nodes are sorted in depth-frist manner
        let tree: DepthFirstArenaTree<usize, String> = tree.into();

        // Check correctness of child references for two nodes
        assert_eq!(tree.0.nodes[0].children, &[1, 5]);
        assert_eq!(tree.0.nodes[1].children, &[2, 4]);
        assert_eq!(tree.0.nodes[2].children, &[3]);
        assert_eq!(tree.0.nodes[5].children, &[6]);

        // check correctness of storage
        assert_eq!(
            tree.0.nodes.iter().map(|n| n.load).collect_vec(),
            &[0, 1, 2, 3, 4, 5, 6]
        );

        // Check whether indices are stored correctly
        assert_eq!(
            tree.0.nodes.iter().map(|n| n.index).collect_vec(),
            &[0, 1, 2, 3, 4, 5, 6]
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

        // TODO Check whether bread-first works (this will already use the cache as optimize has been called)
    }

    #[test]
    fn test_iter_mut() {
        // TODO implement test for mutable iteration
    }
}
