/*! Defines the payload carried by [Nodelike] in the context of kinematics/character animation */

use crate::Nodelike;

/// A Rigid Body represents a single, rigid link connected to other links via a joint.
///
/// Wraps linear algebra transformations such that backends
/// only need to implement this trait.
pub trait Rigid: PartialEq {
    /// E.g., 4x4 matrix, (3x1, 3x3), quaternions ...
    type Transformation: Clone;
    /// Vec, \[f64;4\], ...
    type Point;
    /// typically joint positions (angles/extension), f64, \[f64,3\]
    type Parameter;

    type NodeId: PartialEq + Clone;

    /// Get the Transformation from the parent taking the connecting joint into account
    fn transformation(&self, param: &Self::Parameter) -> Self::Transformation;

    /// Transform a point into the world coordinate system
    fn globalize(&self, other: &Self::Point) -> Self::Point;

    /// Transform a point into the local coordinate system
    fn localize(&self, other: &Self::Point) -> Self::Point;

    /// ...
    fn derivative(&self, param: Self::Parameter) -> Self::Transformation;

    /// ...
    fn n_dof(&self) -> i32;

    /// Returns the eutral element wrt. the transoformation convention used
    fn neutral_element() -> Self::Transformation;

    // Invert a transformation
    fn invert(trafo: &Self::Transformation) -> Self::Transformation;

    /// Concat two transformations
    fn concat(first: &Self::Transformation, second: &Self::Transformation) -> Self::Transformation;
}

/// Helper function to be used in [std::iter::Scan] for accumulating transformations from direct path from a root to a node
pub fn accumulate<'a, Node, Load, NodeRef>(
    stack: &mut Vec<Load::Transformation>,
    arg: (&'a Node, &Load::Parameter),
) -> Option<(&'a Node, Load::Transformation)>
where
    Node: Nodelike<Load, NodeRef>,
    Load: Rigid,
{
    let (node, param) = arg;
    while node.depth() < stack.len() {
        stack.pop();
    }
    let current = Load::concat(
        stack.last().unwrap_or(&Load::neutral_element()),
        &node.get().transformation(param),
    );
    stack.push(current.clone());
    Some((node, current))
}

///  Trait that adds an `accumulate` functions for accumulating transformations from direct path from a root to a node.
/// Wraps a zip and scan operation
///
pub trait TransformationAccumulation<'a, Node, Load, NodeRef>
where
    Load: Rigid,
    Node: Nodelike<Load, NodeRef> + 'a,
{
    fn accumulate_transformations(
        self,
        param: &[Load::Parameter],
        max_depth: usize,
    ) -> impl Iterator<Item = (&'a Node, Load::Transformation)>;
}

impl<'a, 'b, Node, Load, NodeRef> TransformationAccumulation<'a, Node, Load, NodeRef>
    for Box<dyn Iterator<Item = &'a Node> + 'b>
where
    Node: Nodelike<Load, NodeRef>,
    Load: Rigid,
{
    fn accumulate_transformations(
        self,
        param: &[Load::Parameter],
        max_depth: usize,
    ) -> impl Iterator<Item = (&'a Node, <Load as Rigid>::Transformation)> {
        self.into_iter()
            .zip(param.iter())
            .scan(Vec::<Load::Transformation>::with_capacity(max_depth), accumulate)
    }
}

#[cfg(test)]
mod tests {
    use itertools::Itertools;

    use crate::dummy::DummyBody;
    use crate::*;

    #[test]
    fn smoke_test() {}
}
