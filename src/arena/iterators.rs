/*! Implementation of a tree iteration with [arena allocation](https://en.wikipedia.org/wiki/Region-based_memory_management) */
use super::arena_tree::{ArenaNode, ArenaTree};

use crate::{Nodelike, TreeIterable};
use core::fmt;

/// Iterator for a depth-first iteration when the data is not already sorted accordingly
pub struct DepthFirstIterator<'a, 'b, T>
where
    'a: 'b,
    T: 'static + fmt::Debug + PartialEq,
{
    tree: &'a ArenaTree<T>,
    stack: Vec<std::slice::Iter<'b, <ArenaTree<T> as TreeIterable<T>>::NodeRef>>,
    // root: &'b Vec<<ArenaTree<T> as IterableTree>::NodeRef>,
    // root2: Vec<<ArenaTree<T> as IterableTree>::NodeRef>,
    // stack2: Vec<std::slice::Iter<'b, <ArenaTree<T> as IterableTree>::NodeRef>>,
}

impl<'a, 'b, T> DepthFirstIterator<'a, 'b, T>
where
    T: 'static + fmt::Debug + PartialEq,
{
    // TODO did not manage to over write the root nodes :(
    pub fn new(tree: &'a ArenaTree<T>, root: &'b [<ArenaTree<T> as TreeIterable<T>>::NodeRef]) -> Self {
        let mut stack = Vec::with_capacity(tree.max_depth);
        stack.push(root.iter());
        println!("Creating new depth-first iterator (slow)");
        DepthFirstIterator { tree, stack }
    }
}
impl<'a, 'b, T> Iterator for DepthFirstIterator<'a, 'b, T>
where
    T: fmt::Debug + PartialEq,
{
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

pub struct BreadthFirstIterator<'a, T> {
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

/// Accumulation over a *Depth First* iteration (this is not checked).
pub trait DepthFirstAccumulation<'a, Node, T, F, Parameter, Accumulated>
where
    Node: Nodelike<T> + 'a,
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

impl<'a, 'b, N, T, F, P, R> DepthFirstAccumulation<'a, N, T, F, P, R> for Box<dyn Iterator<Item = &'a N> + 'b>
where
    N: Nodelike<T> + 'a,
    F: Fn(&N, &P, &R) -> R + 'static,
    R: Clone,
{
    fn accumulate(self, param: &[P], acc: F, first: R, max_depth: usize) -> impl Iterator<Item = (&'a N, R)> {
        self.into_iter()
            .zip(param.iter())
            // TODO look up whether the move here (required for the acc parameter) slows down execution
            // TODO this function should replace the accumulation in `rigid.rs`
            .scan(Vec::<R>::with_capacity(max_depth), move |stack, (node, param)| {
                let depth = stack.len();
                while node.depth() < depth {
                    stack.pop();
                }
                let result = acc(node, param, stack.last().unwrap_or(&first));
                stack.push(result.clone());
                Some((node, result))
            })
    }
}
