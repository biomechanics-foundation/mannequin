use core::fmt;
use itertools::Itertools;
use std::{
    collections::{hash_map, HashMap},
    fmt::Debug,
    hash::Hash,
    usize,
};

use crate::{
    utils::sort_by_indices,
    MannequinError, Nodelike,
    Order::{self, BreadthFirst, DepthFirst},
    TreeIterable,
};

use super::{BreadthFirstIterator, DepthFirstIterator};
/// A node structure to be used in an arena allocated tree. Fields are used to speed up iteration
#[derive(Debug)]
pub struct ArenaNode<Load, NodeId> {
    load: Load,
    index: usize,

    id: NodeId,
    pub(crate) children: Vec<usize>,
    /// Only used when data is sorted as it is traversed
    width: usize,
    /// Only used in depth-first interation ([DepthFirstIterator])
    depth: usize,
    parent_ref: Option<usize>,
}

impl<Load, NodeRef> ArenaNode<Load, NodeRef> {
    fn new(
        payload: Load,
        node_ref: NodeRef,
        index: usize,
        width: usize,
        children: Vec<usize>,
        depth: usize,
        parent: Option<usize>,
    ) -> Self {
        ArenaNode {
            load: payload,
            id: node_ref,
            index,
            width,
            children,
            depth,
            parent_ref: parent,
        }
    }
}

impl<Load, NodeRef> Nodelike<Load, NodeRef> for ArenaNode<Load, NodeRef>
where
    NodeRef: Clone,
{
    fn is_leaf(&self) -> bool {
        self.children.is_empty()
    }

    fn get(&self) -> &Load {
        &self.load
    }

    fn depth(&self) -> usize {
        self.depth
    }

    fn id(&self) -> NodeRef {
        self.id.clone()
    }
}

impl<Load, NodeRef> fmt::Display for ArenaNode<Load, NodeRef>
where
    Load: fmt::Display,
{
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "NodeRef {}, children: {:?}, payload: {} ",
            self.index, self.children, self.load
        )
    }
}

/// Iterable tree that uses arena allocation.
///
/// Optimized for fast traversal. Building and optimizing the
/// tree may be slower as a consequence. Adding nodes is fast
/// but invalidates optimization. Optimization uses the slow
/// general purpose iterators and caches the descending order.
/// That way, we do not need to manyally update references
/// to the children in each node.
/// TODO: optimization should also change the order the nodes
/// are stored (reduces the lookup) and change references but this
/// is more complex. Would still be useful as it reduces traversal
/// costs by two.
///
/// The struct also prefers speed for memory, and also keeping track
/// of parents, width, and more information simplify the implementation.
pub struct ArenaTree<Load, NodeID> {
    /// Only used when the nodes are stored in the right order
    /// Consider removal?
    sorting: Option<Order>,

    /// Memory allocated area for nodes
    pub(crate) nodes: Vec<ArenaNode<Load, NodeID>>,

    /// Caches the sequence of iteration (depth-first)
    depth_first_cache: Option<Vec<usize>>,
    /// Caches the sequence of iteration (breadth-first)
    breadh_first_cache: Option<Vec<usize>>,

    /// Maximal recursion depth of the dree
    pub(crate) max_depth: usize,

    lookup: HashMap<NodeID, usize>,
}

impl<Load, NodeId> ArenaTree<Load, NodeId> {
    /// Contructor. Sorting indicates whether the elements are stored to
    /// make either deoth or breadth first traversal efficient (slow insertion). `None` indicates
    /// that the data will be unordered (fast insertion, slower traversal).
    pub fn with_capacity(capacity: usize) -> Self {
        let mut nodes = Vec::with_capacity(capacity);

        ArenaTree {
            sorting: None,
            nodes,
            depth_first_cache: None,
            breadh_first_cache: None,
            max_depth: 42,
            lookup: HashMap::with_capacity(capacity),
        }
    }

    /// Contructor. Sorting indicates whether the elements are stored to
    /// make either deoth or breadth first traversal efficient (slow insertion). `None` indicates
    /// that the data will be unordered (fast insertion, slower traversal).
    pub fn new() -> Self {
        ArenaTree {
            sorting: None,
            nodes: vec![],
            depth_first_cache: None,
            breadh_first_cache: None,
            max_depth: 42,
            lookup: HashMap::new(),
        }
    }

    fn update_child_indices(nodes: &mut [ArenaNode<Load, NodeId>], indices: &[usize]) {
        nodes.iter_mut().for_each(|node| {
            node.children.iter_mut().for_each(|child_ref| {
                *child_ref = indices
                    .iter()
                    .position(|i| i == child_ref)
                    .expect("Internal error. Could not find index!")
            });
            node.index = indices
                .iter()
                .position(|i| *i == node.index)
                .expect("Internal error. Could not find index!");
        });
    }
}

impl<Load, NodeId> TreeIterable<Load, NodeId> for ArenaTree<Load, NodeId>
where
    Load: 'static + fmt::Debug + PartialEq,
    NodeId: Eq + 'static + Clone + Hash + Debug,
{
    type Node = ArenaNode<Load, NodeId>;

    #[allow(refining_impl_trait)]
    fn iter<'a, 'b>(
        &'a self,
        traversal: Order,
        root: Option<&Self::Node>,
    ) -> Box<dyn Iterator<Item = &'a Self::Node> + 'b>
    where
        'a: 'b, //  tree outlives the iterator
    {
        let (start, width) = if let Some(root) = root {
            (root.index, root.width)
        } else {
            (0, self.nodes.len())
        };

        match (self.sorting, traversal) {
            // (Some(a), b) if a == b => Box::new(self.nodes.iter()), // Big Todo: skip_while and take_while
            (Some(a), b) if a == b => Box::new(self.nodes.iter().skip(start).take(width)), // Big Todo: skip_while and take_while
            (_, DepthFirst) => {
                // TODO use the depth_first_cache
                Box::new(DepthFirstIterator::new(self, 0))
            }
            (_, BreadthFirst) => {
                // TODO use the breadth_first_cache
                Box::new(BreadthFirstIterator::new(self))
            }
        }
    }

    fn add(&mut self, load: Load, node_ref: NodeId, parent: &NodeId) -> Result<NodeId, MannequinError<NodeId>> {
        self.breadh_first_cache = None;
        self.depth_first_cache = None;
        self.sorting = None;

        let parent = self
            .node_by_id(parent)
            .ok_or(MannequinError::UnkonwnNode(parent.clone()))?;
        // println!("Adding {:?} to parent {:?}", load, parent);

        let index = self.nodes.len();

        // First check whether we can add the node (id not used yet)
        // TODO: This check was after updatinng the parent which should be wrong. Confirm that it's working here.
        if self.nodes.iter().any(|n| n.id() == node_ref) {
            return Err(MannequinError::NotUnique(node_ref));
        }
        let parent_index = parent.index;
        let mut parent = self
            .nodes
            .get_mut(parent_index)
            .ok_or(MannequinError::ReferenceOutOfBound(parent_index))?;

        // * Get the new node's depth
        // * update the parent's width and add the node as a child
        // * Add the node to the root list if it does not have a parent

        parent.children.push(index);

        let depth = parent.depth + 1;
        parent.width += 1;

        // update parent's parents
        while let Some(parent_ref) = parent.parent_ref {
            parent = self
                .nodes
                .get_mut(parent_ref)
                .ok_or(MannequinError::ReferenceOutOfBound(parent_ref))?;
            parent.width += 1;
        }

        self.lookup.insert(node_ref.clone(), index);
        // Finally, add the node
        self.nodes.push(ArenaNode::new(
            load,
            node_ref,
            index,
            1,
            vec![],
            depth,
            Some(parent_index),
        ));

        // println!(
        //     "Nodes after insert {:?}",
        //     self.nodes.iter().map(|i| (&i.load, i.depth)).collect_vec()
        // );
        Ok(self.nodes.last().unwrap().id.clone())
    }

    fn optimize(&mut self, for_traversal: Order) {
        // populate caches usually with the iterators.
        self.depth_first_cache = Some(self.iter(DepthFirst, None).map(|node| node.index).collect_vec());

        // TODO not implemented yet
        // self.breadh_first_cache = Some(
        //     self.iter(BreadthFirst, &self.roots)
        //         .map(|node| node.node_ref)
        //         .collect_vec(),
        // );

        // update roots before

        // reorder the node list and update references
        self.sorting = Some(for_traversal);

        match for_traversal {
            DepthFirst => {
                Self::update_child_indices(&mut self.nodes, self.depth_first_cache.as_ref().unwrap());
                sort_by_indices(&mut self.nodes, self.depth_first_cache.take().unwrap());
            }
            BreadthFirst => {
                unimplemented!();
                Self::update_child_indices(&mut self.nodes, self.breadh_first_cache.as_ref().unwrap());
                sort_by_indices(&mut self.nodes, self.breadh_first_cache.take().unwrap());
            }
        }
        self.lookup.clear();
        self.nodes.iter().for_each(|node| {
            self.lookup.insert(node.id.clone(), node.index);
        });
    }

    fn node_by_load(&self, load: &Load) -> Option<&Self::Node> {
        self.nodes.iter().find(|node| node.load == *load)
    }

    fn node_by_id(&self, node_ref: &NodeId) -> Option<&Self::Node> {
        // A hashmap is be faster: O(1)!
        // TODO Confirm code is still working

        let index = self.lookup.get(node_ref)?;
        self.nodes.get(*index)

        // self.nodes.iter().find(|node| node.id == *node_ref)
    }

    fn set_root(&mut self, root_load: Load, root_ref: NodeId) -> NodeId {
        self.nodes.clear();
        let root = ArenaNode::<Load, NodeId>::new(root_load, root_ref.clone(), 0, 1, vec![], 0, None);
        self.nodes.push(root);
        self.lookup.insert(root_ref, 0);
        self.nodes[0].id.clone()
    }

    fn root(&self) -> Result<&Self::Node, MannequinError<NodeId>> {
        self.nodes.first().ok_or_else(|| MannequinError::RootNotSet)
    }

    fn children(&self, node: &Self::Node) -> Result<Vec<&Self::Node>, MannequinError<NodeId>> {
        let id = node.id();
        // can we rely on this check?
        self.node_by_id(&id).ok_or(MannequinError::UnkonwnNode(id))?;
        Ok(self
            .nodes
            .iter()
            .filter(|n| node.children.iter().contains(&n.index))
            .collect_vec())
    }
}

#[cfg(test)]
mod tests {

    use itertools::Itertools;
    use test_log;

    use crate::*;

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

        let mut tree = ArenaTree::<usize, String>::new();

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
        let result = tree.iter(DepthFirst, None).map(|i| *i.get()).collect_vec();
        assert_eq!(result, &[0, 1, 2, 3, 4, 5, 6]);
        let result = tree.iter(DepthFirst, None).map(|i| &i.id).collect_vec();

        // Check whether iteration is not in the same order as insertion and matches the expectation
        assert_eq!(
            result,
            &["root", "first", "third", "fifth", "fourth", "second", "sixth"]
        );

        // Check correctness of child references for two nodes
        assert_eq!(tree.nodes[0].children, &[1, 2]);
        assert_eq!(tree.nodes[1].children, &[3, 4]);
        assert_eq!(tree.nodes[2].children, &[6]);
        assert_eq!(tree.nodes[3].children, &[5]);

        // // Example of how to print the hierachy
        // tree.nodes
        // .iter()
        // .enumerate()
        // .for_each(|(i, n)| println!("{i}.: id: {}, load: {}, children {:?}", n.id, n.load, n.children));

        // Optimize the tree such that the nodes are sorted in depth-frist manner
        tree.optimize(DepthFirst);

        // Check correctness of child references for two nodes
        assert_eq!(tree.nodes[0].children, &[1, 5]);
        assert_eq!(tree.nodes[1].children, &[2, 4]);
        assert_eq!(tree.nodes[2].children, &[3]);
        assert_eq!(tree.nodes[5].children, &[6]);

        // check correctness of storage
        assert_eq!(tree.nodes.iter().map(|n| n.load).collect_vec(), &[0, 1, 2, 3, 4, 5, 6]);

        // Check whether indices are stored correctly
        assert_eq!(tree.nodes.iter().map(|n| n.index).collect_vec(), &[0, 1, 2, 3, 4, 5, 6]);

        assert_eq!(
            tree.nodes.iter().map(|n| &n.id).collect_vec(),
            &["root", "first", "third", "fifth", "fourth", "second", "sixth"]
        );

        // Check whether the widths are computed correctly
        let result = tree.nodes.iter().map(|n| n.width).collect_vec();
        assert_eq!(result, &[7, 4, 2, 1, 1, 2, 1]);

        // Now we can safely immutably borrow the node
        let first_node = tree.node_by_id(&first).unwrap();

        // Now check whether iterating over subtrees work
        let result = tree.iter(DepthFirst, Some(first_node)).map(|i| *i.get()).collect_vec();
        assert_eq!(result, &[1, 2, 3, 4]);
        let second_node = tree.node_by_id(&second).unwrap();

        let result = tree.iter(DepthFirst, Some(second_node)).map(|i| *i.get()).collect_vec();
        assert_eq!(result, &[5, 6]);

        // TODO Check whether bread-first works (this will already use the cache as optimize has been called)
    }

    // TODO add unit test for a `Box`ed node load
    #[test]
    fn test_boxed() {
        // let mut tree = ArenaTree::<Box<usize>>::new();
        // let first = tree.add(Box::new(1), None).unwrap();
        // // Add the second root first to see whether resorting of the vector works
        // let second = tree.add(Box::new(5), None).unwrap();
        // let third = tree.add(Box::new(2), Some(first)).unwrap();
        // tree.add(Box::new(4), Some(first)).unwrap();
        // tree.add(Box::new(3), Some(third)).unwrap();
        // tree.add(Box::new(6), Some(second)).unwrap();

        // // This uses the depth first iterator!
        // let result = tree
        //     .iter(DepthFirst, &[first, second])
        //     // TODO check whether the clone is a problem or not.
        //     .map(|i| i.get().as_ref())
        //     .copied()
        //     .collect_vec();
        // assert_eq!(result, &[1, 2, 3, 4, 5, 6]);
    }
}
