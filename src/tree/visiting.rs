//! Internal implementation of depth-first tree traversal
//! with a visitor pattern.

use super::visitable::Accumulable;
use std::marker::PhantomData;

/// Trait for structs (visitors) implementing the
/// [visitor pattern](https://en.wikipedia.org/wiki/Visitor_pattern)
pub trait Visiting<'a, T, Parameter, Accumulator>
where
    T: 'a,
    Accumulator: Accumulable,
{
    /// Visits the next node a tree. A parameter can be provided
    /// to the computation of the accumulation for each node *along a path*.
    ///
    /// In animation, for instance, we want to compute the pose of a character
    /// by applying angles to each joint. Hence, to compute the local coordinate systems
    /// an additional parameter is required.
    fn next(&mut self, parameter: Option<&Parameter>) -> Option<&Vec<(&T, Accumulator)>>;
}

/// Visitor for depth-first tree traversal. Used in in [`Visitable::visitor`],
/// one can create this object manually by defining its behavior with closures.
pub struct Visitor<'a, T, It, Accumulator, GetChildren, Parameter, Accumulate>
where
    It: Iterator<Item = &'a T>,
    GetChildren: Fn(&'a T) -> It,
    Accumulator: Accumulable,
    Accumulate: FnMut(&T, &Accumulator, Option<&Parameter>) -> Accumulator,
{
    /// First element of the tree
    root: &'a T,
    /// Nodes and accumulated data of the current Path.
    /// TODO remove the iterator
    pub stack: Vec<(&'a T, Accumulator)>,
    children: Vec<It>,
    /// Get children of currently visited node
    pub get_children: GetChildren,
    /// Accumulate data while traversing
    pub accumulate: Accumulate,
    /// Placeholder
    zipped: PhantomData<Parameter>, // payload: PhantomData<Payload>,
}

impl<'a, T, It, Accumulator, GetChildren, Parameter, Accumulate>
    Visitor<'a, T, It, Accumulator, GetChildren, Parameter, Accumulate>
where
    It: Iterator<Item = &'a T>,
    Accumulator: Accumulable,
    GetChildren: Fn(&'a T) -> It,
    Accumulate: FnMut(&T, &Accumulator, Option<&Parameter>) -> Accumulator,
{
    /// Create new visitor for a root node
    ///
    /// ## Arguments
    ///
    /// `root` – The root of the tree
    /// `max_depth` – The max depth of the tree for allocating memory
    /// `get_children` – Closure to get children of a node
    /// `accumulate` – Closure to accumulate values on traversal
    /// `on_visit` – Closure to be executed when a node is visited
    ///
    /// ## Example
    ///
    /// See unit tests for now

    pub fn new(
        root: &'a T,
        max_depth: usize,
        get_children: GetChildren,
        accumulate: Accumulate,
        // on_visit: OnVisit,
    ) -> Self {
        Self {
            root,
            stack: Vec::with_capacity(max_depth),
            children: Vec::with_capacity(max_depth),
            get_children,
            accumulate,
            // on_visit,
            zipped: PhantomData {},
        }
    }
}

impl<'a, T, It, Accumulator, GetChildren, Parameter, Accumulate> Visiting<'a, T, Parameter, Accumulator>
    for Visitor<'a, T, It, Accumulator, GetChildren, Parameter, Accumulate>
where
    It: Iterator<Item = &'a T>,
    Accumulator: Accumulable,
    GetChildren: Fn(&'a T) -> It,
    Accumulate: FnMut(&T, &Accumulator, Option<&Parameter>) -> Accumulator,
{
    fn next(&mut self, zipped: Option<&Parameter>) -> Option<&Vec<(&T, Accumulator)>> {
        if self.stack.is_empty() {
            let acc = (self.accumulate)(self.root, &Accumulator::neutral(), zipped);
            self.stack.push((&self.root, acc));
            self.children.push((self.get_children)(self.root));
            return Some(&self.stack);
        }

        loop {
            match self.children.last_mut() {
                Some(current) => match current.next() {
                    Some(next) => {
                        let (_, acc) = self.stack.last().unwrap(); // same length
                        let acc = (self.accumulate)(next, acc, zipped);

                        self.stack.push((&next, acc));

                        let children = (self.get_children)(next);
                        self.children.push(children);

                        return Some(&self.stack);
                    }
                    None => {
                        self.children.pop();
                        self.stack.pop();
                    }
                },
                None => return None,
            };
        }
    }
}

#[cfg(test)]
mod tests {

    use super::*;

    #[test]
    fn test_trivial_visitor() {
        // trivial examples of a walking tree

        // A tree with a children and an integer as placeholder for transformations
        #[derive(Debug)]
        struct Node {
            pub val: i32,
            pub children: Vec<Node>,
        }
        impl Node {
            fn new(val: i32, children: Vec<Node>) -> Self {
                Self { val, children }
            }
        }

        // Simple tree: root node with to children
        let tree = Node::new(1, vec![Node::new(2, vec![]), Node::new(3, vec![])]);

        // Set up the visitor
        let mut visitor = Visitor::new(
            &tree,
            2,
            |x| x.children.iter(),
            |x, acc, zipped| acc + x.val * zipped.unwrap(),
        );

        // Parameter vector for the nodes
        let mut parameter = [2, 3, 4].iter();

        // To verify the correctness
        let mut expectation = [2, 8, 14].iter();

        // visit the nodes
        while let Some(stack) = visitor.next(parameter.next()) {
            let (node, acc) = stack.last().unwrap();
            let depth = stack.len();
            println!("Node: {}, {acc}, (depth {depth})", node.val);
            assert!(expectation.next().unwrap() == acc);
        }
    }
}
