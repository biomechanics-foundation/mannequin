//! **TBD.** Breadth-first traversal implementations

// TODO Add Breadth-first Implementation

use super::{ArenaIndex, ArenaNode, DirectedArenaTree};
use std::hash::Hash;

pub struct BreadthFirstIterator<'a, T, NodeRef>
where
    NodeRef: Eq + Hash,
{
    #[allow(dead_code)]
    tree: &'a DirectedArenaTree<T, NodeRef>,
}

impl<'a, T, NodeRef> BreadthFirstIterator<'a, T, NodeRef>
where
    NodeRef: Eq + Hash,
{
    pub fn new(tree: &'a DirectedArenaTree<T, NodeRef>, _root: ArenaIndex) -> Self {
        BreadthFirstIterator { tree }
    }
}
impl<'a, T, NodeRef> Iterator for BreadthFirstIterator<'a, T, NodeRef>
where
    NodeRef: Eq + Hash,
{
    type Item = &'a ArenaNode<T, NodeRef>;

    fn next(&mut self) -> Option<Self::Item> {
        todo!()
    }
}

#[cfg(test)]
mod tests {}
