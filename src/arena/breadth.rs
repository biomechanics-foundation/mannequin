//! TBD. Implement breadth first optimized arena allocated tree
// TODO Implement breadth first optimized arena allocated tree

use super::{ArenaIndex, ArenaNode, DirectedArenaTree};

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
