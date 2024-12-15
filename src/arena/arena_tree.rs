use core::fmt;
use itertools::Itertools;

use crate::{
    utils::sort_by_indices,
    MannequinError, Nodelike,
    Order::{self, BreadthFirst, DepthFirst},
    TreeIterable,
};

use super::{BreadthFirstIterator, DepthFirstIterator};
/// A node structure to be used in an arena allocated tree. Fields are used to speed up iteration
#[derive(Debug)]
pub struct ArenaNode<T> {
    load: T,
    /// TODO check if we need this?
    node_ref: usize,
    pub(crate) children: Vec<usize>,
    /// Only used when data is sorted as it is traversed
    width: usize,
    /// Only used in depth-first interation ([DepthFirstIterator])
    depth: usize,
    parent_ref: Option<usize>,
}

impl<T> ArenaNode<T> {
    fn new(
        payload: T,
        node_ref: usize,
        width: usize,
        children: Vec<usize>,
        depth: usize,
        parent: Option<usize>,
    ) -> Self {
        ArenaNode {
            load: payload,
            node_ref,
            width,
            children,
            depth,
            parent_ref: parent,
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
pub struct ArenaTree<T> {
    /// Only used when the nodes are stored in the right order
    /// Consider removal?
    sorting: Option<Order>,

    /// Memory allocated area for nodes
    pub(crate) nodes: Vec<ArenaNode<T>>,

    /// Caches the sequence of iteration (depth-first)
    depth_first_cache: Option<Vec<usize>>,
    /// Caches the sequence of iteration (breadth-first)
    breadh_first_cache: Option<Vec<usize>>,

    /// We support having multiple roots
    roots: Vec<usize>,

    /// Maximal recursion depth of the dree
    pub(crate) max_depth: usize,
}

impl<T> ArenaTree<T> {
    /// Contructor. Sorting indicates whether the elements are stored to
    /// make either deoth or breadth first traversal efficient (slow insertion). `None` indicates
    /// that the data will be unordered (fast insertion, slower traversal).
    pub fn with_capacity(sorting: Option<Order>, capacity: usize) -> Self {
        ArenaTree {
            sorting,
            nodes: Vec::with_capacity(capacity),
            depth_first_cache: None,
            breadh_first_cache: None,
            roots: vec![],
            max_depth: 42,
        }
    }
    /// Contructor. Sorting indicates whether the elements are stored to
    /// make either deoth or breadth first traversal efficient (slow insertion). `None` indicates
    /// that the data will be unordered (fast insertion, slower traversal).
    pub fn new(sorting: Option<Order>) -> Self {
        ArenaTree {
            sorting,
            nodes: vec![],
            depth_first_cache: None,
            breadh_first_cache: None,
            roots: vec![],
            max_depth: 42,
        }
    }

    fn update_child_indices(nodes: &mut [ArenaNode<T>], indices: &[usize]) {
        nodes.iter_mut().for_each(|node| {
            node.children.iter_mut().for_each(|child_ref| {
                *child_ref = indices
                    .iter()
                    .position(|i| i == child_ref)
                    .expect("Internal error. Could not find index!")
            });
            node.node_ref = indices
                .iter()
                .position(|i| *i == node.node_ref)
                .expect("Internal error. Could not find index!");
        });
    }
    fn update_root_indices(roots: &mut [usize], indices: &[usize]) {
        roots.iter_mut().for_each(|root_ref| {
            *root_ref = indices
                .iter()
                .position(|i| *i == *root_ref)
                .expect("Internal error. Could not find index!");
        });
    }
}

impl<T> TreeIterable<T> for ArenaTree<T>
where
    T: 'static + fmt::Debug + PartialEq,
{
    type Node = ArenaNode<T>;
    type NodeRef = usize;

    #[allow(refining_impl_trait)]
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
                    .take_while(|(i, _)| {
                        // Can the boundary checks be disabled for speed? Needs unsafe?
                        let node = self.nodes.get(*root).expect("Out of bound in managed arena");
                        *i <= node.node_ref + node.width
                    })
                    .map(|(_, n)| n)
            })), // Big Todo: skip_while and take_while
            (_, DepthFirst) => {
                // TODO use the depth_first_cache
                Box::new(DepthFirstIterator::new(self, roots))
            }
            (_, BreadthFirst) => {
                // TODO use the breadth_first_cache
                Box::new(BreadthFirstIterator::new(self))
            }
        }
    }

    fn add(&mut self, load: T, parent: Option<Self::NodeRef>) -> Result<Self::NodeRef, MannequinError> {
        self.breadh_first_cache = None;
        self.depth_first_cache = None;
        self.sorting = None;

        // Get depth o fparent
        if let Some(parent_ref) = parent {
            self.nodes
                .get(parent_ref)
                .ok_or(MannequinError::ReferenceOutOfBound(parent_ref))?
                .depth
                + 1
        } else {
            0
        };

        println!("Adding {:?} to parent {:?}", load, parent);

        let node_ref = self.nodes.len();
        let depth: usize;

        // * Get the new node's depth
        // * update the parent's width and add the node as a child
        // * Add the node to the root list if it does not have a parent
        if let Some(parent_ref) = parent {
            let mut parent = self
                .nodes
                .get_mut(parent_ref)
                .ok_or(MannequinError::ReferenceOutOfBound(parent_ref))?;
            parent.children.push(node_ref);
            depth = parent.depth + 1;
            parent.width += 1;

            // update parent's parents
            while let Some(parent_ref) = parent.parent_ref {
                parent = self
                    .nodes
                    .get_mut(parent_ref)
                    .ok_or(MannequinError::ReferenceOutOfBound(parent_ref))?;
                parent.width += 1;
            }
        } else {
            self.roots.push(node_ref);
            depth = 0;
        };

        // Finally, add the node
        self.nodes
            .push(ArenaNode::new(load, node_ref, 0, vec![], depth, parent));

        println!(
            "Nodes after insert {:?}",
            self.nodes.iter().map(|i| (&i.load, i.depth)).collect_vec()
        );
        Ok(node_ref)
    }

    fn optimize(&mut self, for_traversal: Order) {
        // populate caches usually with the iterators.
        self.depth_first_cache = Some(
            self.iter(DepthFirst, &self.roots)
                .map(|node| node.node_ref)
                .collect_vec(),
        );

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
                Self::update_root_indices(&mut self.roots, self.depth_first_cache.as_ref().unwrap());
                sort_by_indices(&mut self.nodes, self.depth_first_cache.take().unwrap());
            }
            BreadthFirst => {
                Self::update_child_indices(&mut self.nodes, self.breadh_first_cache.as_ref().unwrap());
                Self::update_root_indices(&mut self.roots, self.breadh_first_cache.as_ref().unwrap());

                sort_by_indices(&mut self.nodes, self.breadh_first_cache.take().unwrap());
            }
        }
    }

    fn get_by_load(&self, load: &T) -> Option<Self::NodeRef> {
        self.nodes.iter().position(|node| node.load == *load)
    }
}

#[cfg(test)]
mod tests {

    use itertools::Itertools;
    use test_log;

    use crate::*;

    #[test_log::test]
    fn test_adding_iteration() {
        //  1    5
        // | \   |
        // 2  4  6
        // |
        // 3

        let mut tree = ArenaTree::<usize>::new(Some(DepthFirst));
        let mut first = tree.add(1, None).unwrap();
        // Add the second root first to see whether resorting of the vector works
        let mut second = tree.add(5, None).unwrap();
        let third = tree.add(2, Some(first)).unwrap();
        tree.add(4, Some(first)).unwrap();
        tree.add(3, Some(third)).unwrap();
        tree.add(6, Some(second)).unwrap();

        // This uses the depth first iterator!
        let result = tree.iter(DepthFirst, &[first, second]).map(|i| *i.get()).collect_vec();
        assert_eq!(result, &[1, 2, 3, 4, 5, 6]);

        // Optimize the tree such that the nodes are sorted in depth-frist manner
        assert_eq!(tree.nodes[0].children, &[2, 3]);
        assert_eq!(tree.roots, &[0, 1]);
        tree.optimize(DepthFirst);
        assert_eq!(tree.nodes[0].children, &[1, 3]);
        assert_eq!(tree.roots, &[0, 4]);

        // check correctness of storage
        assert_eq!(tree.nodes.iter().map(|n| n.load).collect_vec(), &[1, 2, 3, 4, 5, 6]);
        assert_eq!(tree.nodes.iter().map(|n| n.node_ref).collect_vec(), &[0, 1, 2, 3, 4, 5]);

        println!("{:?}", tree.nodes.iter().map(|n| n.width).collect_vec());

        // references invalidated by optimize, the roots by load
        first = tree.get_by_load(&1).unwrap();
        second = tree.get_by_load(&5).unwrap();
        assert_eq!(second, 4);
        println!("{first} {second}");
        // Now check whether iterating over the sorted vec works
        let result = tree.iter(DepthFirst, &[first]).map(|i| *i.get()).collect_vec();
        assert_eq!(result, &[1, 2, 3, 4]);
        let result = tree.iter(DepthFirst, &[second]).map(|i| *i.get()).collect_vec();
        assert_eq!(result, &[5, 6]);
        let result = tree.iter(DepthFirst, &[first, second]).map(|i| *i.get()).collect_vec();
        assert_eq!(result, &[1, 2, 3, 4, 5, 6]);
        let result = tree.iter(DepthFirst, &[]).map(|i| *i.get()).collect_vec();
        assert_eq!(result, &[1, 2, 3, 4, 5, 6]);

        // Check whether bread-first works (this will already use the cache as optimize has been called)
    }

    // TODO add unit test for a `Box`ed node load
    #[test]
    fn test_boxed() {
        let mut tree = ArenaTree::<Box<usize>>::new(Some(DepthFirst));
        let first = tree.add(Box::new(1), None).unwrap();
        // Add the second root first to see whether resorting of the vector works
        let second = tree.add(Box::new(5), None).unwrap();
        let third = tree.add(Box::new(2), Some(first)).unwrap();
        tree.add(Box::new(4), Some(first)).unwrap();
        tree.add(Box::new(3), Some(third)).unwrap();
        tree.add(Box::new(6), Some(second)).unwrap();

        // This uses the depth first iterator!
        let result = tree
            .iter(DepthFirst, &[first, second])
            // TODO check whether the clone is a problem or not.
            .map(|i| i.get().as_ref())
            .copied()
            .collect_vec();
        assert_eq!(result, &[1, 2, 3, 4, 5, 6]);
    }
}
