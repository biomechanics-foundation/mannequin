//! Implementation of a directionally iterable
//! [arena allocated](https://en.wikipedia.org/wiki/Region-based_memory_management)
//! tree implementation, which supports depth- and breadth-first element iteration.
//! Iteration uses references and if therefore slower than the implementation in the [depth]
//! and [breadth] suubmodules.

use super::iterables::{BaseDirectionIterable, DirectionIterable, Nodelike};
use super::{DepthFirstArenaTree, DepthFirstIterator};
use crate::MannequinError;
use core::fmt;
use itertools::Itertools;
use std::{collections::HashMap, fmt::Debug, hash::Hash};

#[derive(Debug, PartialEq, Copy, Clone)]
pub struct ArenaIndex(pub usize);

/// A node structure to be used in an arena allocated tree. Fields are used to speed up iteration
#[derive(Debug)]
pub struct ArenaNode<Load, NodeId> {
    /// The user-defined load that the node owns
    pub(super) load: Load,
    /// Index in the arena allocation
    pub(super) index: ArenaIndex,
    /// identifier for lookups
    pub(super) id: NodeId,
    /// references for children
    pub(super) children: Vec<ArenaIndex>,
    /// Used to optize sub-tree, depth-first traversal in [DepthFirstArenaTree]
    pub(super) width: usize,
    /// Depth in the tree
    depth: usize,
    /// Only used in [DirectedArenaTree]
    parent_ref: Option<ArenaIndex>,
}

impl<Load, NodeRef> ArenaNode<Load, NodeRef> {
    fn new(
        payload: Load,
        node_ref: NodeRef,
        index: ArenaIndex,
        width: usize,
        children: Vec<ArenaIndex>,
        depth: usize,
        parent_ref: Option<ArenaIndex>,
    ) -> Self {
        ArenaNode {
            load: payload,
            id: node_ref,
            index,
            width,
            children,
            depth,
            parent_ref,
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
            "Arena index {:?}, children: {:?}, payload: {} ",
            self.index, self.children, self.load
        )
    }
}

/// Iterable tree that uses arena allocation and allows for
/// unoptimized (possibly slow) iteration/traversal in both
/// directions: breadth-first and depth-first.
/// Can be converted to a [DepthFirstIterable] implementation,
/// namely [DepthFirstArenaTree], via a trait method or with `into()`
///
/// The tree is mutable, that is, adding nodes possible, unlike in
/// the trees optimized for a single direction.
pub struct DirectedArenaTree<Load, NodeID> {
    /// Memory allocated area for nodes
    pub(crate) nodes: Vec<ArenaNode<Load, NodeID>>,

    pub(super) max_depth: usize,

    /// Lookup arena indices
    pub(super) lookup: HashMap<NodeID, ArenaIndex>,
}

impl<Load, NodeId> DirectedArenaTree<Load, NodeId> {
    /// Contructor. Sorting indicates whether the elements are stored to
    /// make either deoth or breadth first traversal efficient (slow insertion). `None` indicates
    /// that the data will be unordered (fast insertion, slower traversal).
    pub fn with_capacity(capacity: usize) -> Self {
        let nodes = Vec::with_capacity(capacity);

        DirectedArenaTree {
            nodes,
            // depth_first_cache: None,
            // breadh_first_cache: None,
            max_depth: 42,
            lookup: HashMap::with_capacity(capacity),
        }
    }

    /// Contructor. Sorting indicates whether the elements are stored to
    /// make either deoth or breadth first traversal efficient (slow insertion). `None` indicates
    /// that the data will be unordered (fast insertion, slower traversal).
    pub fn new() -> Self {
        DirectedArenaTree {
            nodes: vec![],
            // depth_first_cache: None,
            // breadh_first_cache: None,
            max_depth: 42,
            lookup: HashMap::new(),
        }
    }

    /// Given an squenze of nodes (i.e., an areana), update the references to child nodes when
    /// the arena is reorderd. It takes a sequence of the same size with the new indices as a parameter
    pub(super) fn update_child_indices(nodes: &mut [ArenaNode<Load, NodeId>], indices: &[ArenaIndex]) {
        nodes.iter_mut().for_each(|node| {
            node.children.iter_mut().for_each(|child_ref| {
                *child_ref = ArenaIndex(
                    indices
                        .iter()
                        .position(|i| *i == *child_ref)
                        .expect("Internal error. Could not find index!"),
                )
            });
            node.index = ArenaIndex(
                indices
                    .iter()
                    .position(|i| *i == node.index)
                    .expect("Internal error. Could not find index!"),
            );
        });
    }
}

impl<Load, NodeId> Default for DirectedArenaTree<Load, NodeId> {
    fn default() -> Self {
        Self::new()
    }
}

impl<Load, NodeId> BaseDirectionIterable<Load, NodeId> for DirectedArenaTree<Load, NodeId>
where
    Load: 'static + fmt::Debug + PartialEq,
    NodeId: Eq + 'static + Clone + Hash + Debug,
{
    type Node = ArenaNode<Load, NodeId>;

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

    fn node_by_load(&self, load: &Load) -> Option<&Self::Node> {
        self.nodes.iter().find(|node| node.load == *load)
    }

    fn node_by_id(&self, node_ref: &NodeId) -> Option<&Self::Node> {
        // A hashmap is be faster: O(1)
        // TODO Confirm code is still working

        let index = self.lookup.get(node_ref)?;
        self.nodes.get(index.0)

        // self.nodes.iter().find(|node| node.id == *node_ref)
    }

    fn nodes(&self) -> &[Self::Node] {
        &self.nodes
    }
}

impl<Load, NodeId> DirectionIterable<Load, NodeId> for DirectedArenaTree<Load, NodeId>
where
    Load: 'static + fmt::Debug + PartialEq,
    NodeId: Eq + 'static + Clone + Hash + Debug,
{
    fn iter_depth(&self) -> impl Iterator<Item = &Self::Node> {
        Box::new(DepthFirstIterator::new(self, ArenaIndex(0)))
    }

    fn iter_depth_sub(&self, root: &Self::Node) -> impl Iterator<Item = &Self::Node> {
        Box::new(DepthFirstIterator::new(self, root.index))
    }

    fn iter_breadth(&self) -> impl Iterator<Item = &Self::Node> {
        Box::new(BreadthFirstIterator::new(self, ArenaIndex(0)))
    }

    fn iter_breadth_sub(&self, root: &Self::Node) -> impl Iterator<Item = &Self::Node> {
        Box::new(BreadthFirstIterator::new(self, root.index))
    }

    fn depth_first(self) -> impl crate::DepthFirstIterable<Load, NodeId> {
        let result: DepthFirstArenaTree<Load, NodeId> = self.into();
        result
    }

    // fn breadth_first(self) -> impl crate::BreadthFirstIterable<Load, NodeId> {
    //     unimplemented!();
    // }

    fn add(&mut self, load: Load, node_ref: NodeId, parent: &NodeId) -> Result<NodeId, MannequinError<NodeId>> {
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
            .get_mut(parent_index.0)
            .ok_or(MannequinError::ReferenceOutOfBound(parent_index.0))?;

        // * Get the new node's depth
        // * update the parent's width and add the node as a child
        // * Add the node to the root list if it does not have a parent

        parent.children.push(ArenaIndex(index));

        let depth = parent.depth + 1;
        parent.width += 1;

        // update parent's parents
        while let Some(parent_ref) = parent.parent_ref {
            parent = self
                .nodes
                .get_mut(parent_ref.0)
                .ok_or(MannequinError::ReferenceOutOfBound(parent_ref.0))?;
            parent.width += 1;
        }

        self.lookup.insert(node_ref.clone(), ArenaIndex(index));
        // Finally, add the node
        self.nodes.push(ArenaNode::new(
            load,
            node_ref,
            ArenaIndex(index),
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

    fn set_root(&mut self, root_load: Load, root_ref: NodeId) -> NodeId {
        self.nodes.clear();
        let root = ArenaNode::<Load, NodeId>::new(root_load, root_ref.clone(), ArenaIndex(0), 1, vec![], 0, None);
        self.nodes.push(root);
        self.lookup.insert(root_ref, ArenaIndex(0));
        self.nodes[0].id.clone()
    }
}

pub struct BreadthFirstIterator<'a, T, NodeRef> {
    #[allow(dead_code)]
    tree: &'a DirectedArenaTree<T, NodeRef>,
}

impl<'a, T, NodeRef> BreadthFirstIterator<'a, T, NodeRef> {
    pub fn new(tree: &'a DirectedArenaTree<T, NodeRef>, _root: ArenaIndex) -> Self {
        BreadthFirstIterator { tree }
    }
}
impl<'a, T, NodeRef> Iterator for BreadthFirstIterator<'a, T, NodeRef> {
    type Item = &'a ArenaNode<T, NodeRef>;

    fn next(&mut self) -> Option<Self::Item> {
        todo!()
    }
}
