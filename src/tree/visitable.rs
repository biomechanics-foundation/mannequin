//! Interfaces for a depth-first tree traversal
//! with a visitor pattern.

use super::visiting::{Visiting, Visitor};

/// Values that can be accumulated *along a path* during tree traversal.
/// For instance, the affine transformations in a character animation.
/// This is a thin-wrapper for linear algebra data types in the case of kinematics.
pub trait Accumulable: Clone {
    /// Create neutral element
    fn neutral() -> Self;
    /// Accumulate.
    fn accumulate(&self, other: &Self) -> Self;
}

/// Trait for structures that represent nodes of a tree that allows visiting its children depth first.
pub trait Visitable
where
    Self: Sized,
{
    /// Accumulator type. Along each *path* (i.e., sequence of nodes from the root to a leaf),
    /// values can be accumulated. One can count the number of nodes as a trivial example, or
    /// compute local coordinate systems of an animated character's bones.
    type Accumulator: Accumulable;
    /// Parameter type used to modify the Accumulator along a path. In above's example of character
    /// animation, this type represents a joint angle
    type Parameter;
    /// While visiting each node, mutable data is passed around to grant access to the outside scope
    type Payload;

    /// Gets the node's children.
    fn children(&self) -> impl Iterator<Item = &Self>;

    /// When visiting this node, this method is called compute the accumulation along a path.
    fn accumulate(&self, acc: &Self::Accumulator, zipped: Option<&Self::Parameter>) -> Self::Accumulator;

    /// Arbitrary action when visiting a node along path. It receives a reference to the
    /// "history", the previous nodes and accumulator values that is, in addition to mutable
    /// data that allows interaction with the outside context.
    #[allow(unused)]
    fn on_visit(&self, stack: &[(&Self, Self::Accumulator)], payload: &mut Self::Payload);

    /// Generates a visitor for the tree with the current element as its root.
    fn visitor(&self, max_depth: usize) -> impl Visiting<Self, Self::Parameter, Self::Accumulator> {
        Visitor::new(self, max_depth, |s| s.children(), |s, a, z| s.accumulate(a, z))
    }

    /// Visits all children and children's children and calls `accumulate` (implicitly) and `on_visit`
    /// on each node.
    fn visit<'a>(
        &'a self,
        max_depth: usize,
        mut parameters: impl Iterator<Item = &'a Self::Parameter>,
        payload: &mut Self::Payload,
    ) {
        let mut visitor = self.visitor(max_depth);
        while let Some(stack) = visitor.next(parameters.next()) {
            self.on_visit(stack, payload);
        }
    }

    // fn iter<'a>(
    //     &self,
    //     mut parameters: impl Iterator<Item = &'a Self::Parameter>,
    // ) -> TreeIterator<
    //     'a,
    //     Self,
    //     impl Visiting<Self, Self::Parameter, Self::Accumulator>,
    //     Self::Accumulator,
    //     Self::Parameter,
    //     impl Iterator<Item = &'a Self::Parameter>,
    // > {
    //     todo!()
    // }
}

#[cfg(test)]
mod tests {

    /// Use integers as accumulated values in the trivial examples
    impl Accumulable for i32 {
        fn neutral() -> Self {
            0i32
        }

        fn accumulate(&self, other: &Self) -> Self {
            self + other
        }
    }
    use super::*;

    #[test]
    fn test_trivial_visitable() {
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

        impl Visitable for Node {
            type Accumulator = i32;

            type Parameter = i32;

            type Payload = std::slice::Iter<'static, i32>;

            fn children(&self) -> impl Iterator<Item = &Self> {
                self.children.iter()
            }

            fn accumulate(&self, acc: &Self::Accumulator, zipped: Option<&Self::Parameter>) -> Self::Accumulator {
                self.val * zipped.unwrap_or(&1) + acc
            }

            fn on_visit(&self, stack: &[(&Self, Self::Accumulator)], payload: &mut Self::Payload) {
                let depth = stack.len();
                let (_, acc) = stack.last().unwrap(); // guaranteed to bot be empty
                let expectation = payload.next().unwrap();
                println!(
                    "Node: {}, {}, (depth {depth}), expectation {expectation}",
                    self.val, acc
                );
                assert!(expectation == acc);
            }
        }

        // Simple tree: root node with to children
        let tree = Node::new(1, vec![Node::new(2, vec![]), Node::new(3, vec![])]);

        // To verify the correctness
        let mut expectation = [2, 8, 14].iter();

        // Parameter vector for the nodes
        let parameters = [2, 3, 4].iter();

        tree.visit(2, parameters, &mut expectation);
    }
}
