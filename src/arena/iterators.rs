/*! Implementation of a tree iteration with [arena allocation](https://en.wikipedia.org/wiki/Region-based_memory_management) */
use super::arena_tree::{ArenaNode, ArenaTree};
use crate::Nodelike;
use core::fmt;

/// Iterator for a depth-first iteration when the data is not already sorted accordingly
pub struct DepthFirstIterator<'a, 'b, T, N>
where
    'a: 'b,
    T: 'static + fmt::Debug + PartialEq,
{
    tree: &'a ArenaTree<T, N>,
    stack: Vec<std::slice::Iter<'b, usize>>,
    root: Option<usize>,
}

impl<'a, 'b, T, N> DepthFirstIterator<'a, 'b, T, N>
where
    T: 'static + fmt::Debug + PartialEq,
{
    // TODO did not manage to over write the root nodes :(
    pub fn new(tree: &'a ArenaTree<T, N>, root: usize) -> Self {
        let stack = Vec::with_capacity(tree.max_depth);
        println!("Creating new depth-first iterator (slow)");
        DepthFirstIterator {
            tree,
            stack,
            root: Some(root),
        }
    }
}
impl<'a, 'b, T, N> Iterator for DepthFirstIterator<'a, 'b, T, N>
where
    T: fmt::Debug + PartialEq,
{
    type Item = &'a ArenaNode<T, N>;

    fn next(&mut self) -> Option<Self::Item> {
        if let Some(root) = self.root {
            let root = &self.tree.nodes[root];
            self.stack.push(root.children.iter());
            self.root = None;
            Some(root)
        } else if let Some(last) = self.stack.last_mut() {
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

pub struct BreadthFirstIterator<'a, T, NodeRef> {
    #[allow(dead_code)]
    tree: &'a ArenaTree<T, NodeRef>,
}

impl<'a, T, NodeRef> BreadthFirstIterator<'a, T, NodeRef> {
    pub fn new(tree: &'a ArenaTree<T, NodeRef>) -> Self {
        BreadthFirstIterator { tree }
    }
}
impl<'a, T, NodeRef> Iterator for BreadthFirstIterator<'a, T, NodeRef> {
    type Item = &'a ArenaNode<T, NodeRef>;

    fn next(&mut self) -> Option<Self::Item> {
        todo!()
    }
}

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
