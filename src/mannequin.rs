/*! This module defines the flat, composable architecture for a mannequin (an animated character).
 * Different algorithms for forward and inverse kinematics can be freely combined (as long they are
 * using the same backend) such as rigid bodies, deformable skinning,
 * realistic joints, muscle simulation, or classical inverse kinematics and obstacle avoidance.
 */

use crate::{DepthFirstIterable, Forward, Inverse};
use std::{fmt::Debug, hash::Hash, marker::PhantomData};

/// A Rigid Body represents a single, rigid link connected to other links via a joint.
/// Synonyms: Bone
///
/// Wraps all linear algebra transformations such that backends
/// only need to implement this trait.
pub trait Rigid: PartialEq {
    /// E.g., 4x4 matrix, (3x1, 3x3), quaternions ...
    type Transformation: Clone + Debug;
    /// Vec, \[f64;4\], ...
    type Point;
    /// typically joint positions (angles/extension), f64, \[f64,3\]
    type Parameter;

    // TODO Explain why this is defined on Rigid (the node) and not Mannequin (the tree) .. in short: otherwise this would be another generic and mannequin.rs would be unreadable because of trait bounds. This way it is quite elegant
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

/// Struct for holding the composition of character animation algorithms in a flat architecture for
/// character animation.
pub struct Mannequin<IT, RB, FK, IK>
where
    RB: Rigid,
    IT: DepthFirstIterable<RB, RB::NodeId>,

    FK: Forward<IT, RB>,
    IK: Inverse<IT, RB, FK, Array = FK::Transformation>,
{
    pub tree: IT,
    pub fk: FK,
    pub ik: IK,
    rigid_body: PhantomData<RB>,
}

impl<IT, RB, FK, IK> Mannequin<IT, RB, FK, IK>
where
    RB: Rigid,
    IT: DepthFirstIterable<RB, RB::NodeId>,
    FK: Forward<IT, RB>,
    IK: Inverse<IT, RB, FK, Array = FK::Transformation>,
{
    pub fn new(tree: IT, foward_kinematics: FK, inverse_kinematics: IK) -> Self {
        Mannequin {
            tree,
            fk: foward_kinematics,
            ik: inverse_kinematics,
            rigid_body: PhantomData,
        }
    }

    /// Forward kinematics for the targets in `target_refs` and the joint positions in `param`.
    pub fn forward(&mut self, param: FK::Parameter, target_refs: &[RB::NodeId]) -> Vec<FK::Transformation> {
        self.fk.solve(&self.tree, param, target_refs)
    }

    /// Inverse kinematics for the targets in `target_refs` and the desired working space coordinates in `target_val`.
    pub fn inverse(
        &mut self,
        param: IK::Parameter,
        target_refs: &[RB::NodeId],
        target_val: &[IK::Array],
    ) -> IK::Parameter {
        self.ik.solve(&self.tree, &self.fk, param, target_refs, target_val)
    }
}
