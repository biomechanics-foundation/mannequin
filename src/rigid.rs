/*! Defines the payload carried by [Nodelike] in the context of kinematics/character animation */

use std::{fmt::Debug, hash::Hash};

use crate::Nodelike;

/// A Rigid Body represents a single, rigid link connected to other links via a joint.
/// Synonyms: Bone
///
/// Wraps linear algebra transformations such that backends
/// only need to implement this trait.
pub trait Rigid: PartialEq {
    /// E.g., 4x4 matrix, (3x1, 3x3), quaternions ...
    type Transformation: Clone + Debug;
    /// Vec, \[f64;4\], ...
    type Point;
    /// typically joint positions (angles/extension), f64, \[f64,3\]
    type Parameter;

    // TODO Explain why this is defined on Rigid (the node) and not Mannequin (the tree) .. in short, otherwise this would be another generic and mannequin.rs would be unreadable because of trait bounds. This way it is quite elegant
    type NodeId: Eq + Hash + Clone + Debug;

    /// Get the Transformation from the parent taking the connecting joint into account
    fn transform(&self, params: &Self::Parameter) -> Self::Transformation;

    /// Transform a point into the world coordinate system
    fn globalize(&self, other: &Self::Point) -> Self::Point;

    /// Transform a point into the local coordinate system
    fn localize(&self, other: &Self::Point) -> Self::Point;

    /// Dimensionality of the partial derivatives (e.g., 3 for position, 6 for position and orientation)
    fn dim(&self) -> usize;

    /// Compute partial derivative of all effectors
    /// pose: This node's FoR in global coordinates
    /// joint: Reference to the joint node
    /// joint_pose: the joint's FoR in global coordinates
    /// target_buffer: Memory location to where the results are written too.
    fn partial_derivative(
        &self,
        pose: &Self::Transformation,
        joint: &Self,
        joint_pose: &Self::Transformation,
        target_buffer: &mut [f64],
    );

    /// number of effectors
    fn effector_count(&self) -> usize;

    /// The number of rows / elements the effector take in the jacobian matrix (usually dim * cound).
    /// However, by manually granting contol, one can have effocters with different dimensionality
    fn effector_size(&self) -> usize {
        self.dim() * self.effector_count()
    }

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
        &node.get().transform(param),
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

impl<'a, Node, Load, NodeRef, T> TransformationAccumulation<'a, Node, Load, NodeRef> for T
//Box<dyn Iterator<Item = &'a Node> + 'b>
where
    Node: Nodelike<Load, NodeRef> + 'a,
    Load: Rigid,
    T: Iterator<Item = &'a Node>,
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
mod tests {}
