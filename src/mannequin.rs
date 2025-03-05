/*! This module defines the flat, composable architecture for a mannequin (an animated character).
 * Different algorithms for forward and inverse kinematics can be freely combined (as long they are
 * using the same backend) such as rigid bodies, deformable skinning,
 * realistic joints, muscle simulation, or classical inverse kinematics and obstacle avoidance.
 */

use num_traits::Float;

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
    // /// typically joint positions (angles/extension), f64, \[f64,3\]
    // type Parameter;
    type FloatType: Float;

    // TODO Explain why this is defined on Rigid (the node) and not Mannequin (the tree) .. in short: otherwise this would be another generic and mannequin.rs would be unreadable because of trait bounds. This way it is quite elegant
    type NodeId: Eq + Hash + Clone + Debug;

    /// Get the Transformation from the parent taking the connecting joint into account
    /// Note: This method receives all parameters and it's in the implementing structs
    /// responsibility to address the correct value (via `index`). This decision has been
    /// made to allow for the implementation of custom multivariate joints (e.g., a realistic
    /// shoulder joint).
    fn transform(&self, params: &[Self::FloatType], index: usize) -> Self::Transformation;

    /// Transform a point into the world coordinate system
    fn globalize(&self, other: &Self::Point) -> Self::Point;

    /// Transform a point into the local coordinate system
    fn localize(&self, other: &Self::Point) -> Self::Point;

    /// Dimensionality of the partial derivatives (e.g., 3 for position, 6 for position and orientation)
    fn dim(&self) -> usize;

    /// Compute partial derivative of all effectors
    /// pose: This node's frame of in global coordinates
    /// joint: Reference to the joint node
    /// joint_pose: the joint's frame of in global coordinates
    /// target_buffer: Memory location to where the results are written to. The implementation is responsible of taking care only that
    /// correct location in the buffer is written to (e.g.,
    /// `[offset..offset + self.effector_size()]`)
    /// offset: Start positin in the buffer to write to
    fn partial_derivative(
        &self,
        pose: &Self::Transformation,
        joint: &Self,
        joint_pose: &Self::Transformation,
        target_buffer: &mut [Self::FloatType],
        offset: usize,
    );

    fn effector(&self, pose: &Self::Transformation, target_buffer: &mut [Self::FloatType], offset: usize);

    /// number of effectors
    fn effector_count(&self) -> usize;

    /// The number of rows / elements the effector take in the jacobian matrix (usually dim * count).
    /// However, by manually granting control, one can have effocters with different dimensionality
    fn effector_size(&self) -> usize {
        self.dim() * self.effector_count()
    }

    /// Returns the eutral element wrt. the transoformation convention used
    fn neutral_element() -> Self::Transformation;

    // Invert a transformation
    fn invert(trafo: &Self::Transformation) -> Self::Transformation;

    /// Concat two transformations
    fn concat(first: &Self::Transformation, second: &Self::Transformation) -> Self::Transformation;

    fn solve_linear(
        matrix: &[Self::FloatType],
        rows: usize,
        cols: usize,
        vector: &[Self::FloatType],
        target_buffer: &mut [Self::FloatType],
    );
}

/// Struct for holding the composition of character animation algorithms in a flat architecture for
/// character animation.
pub struct Mannequin<IT, RB, FK, IK>
where
    RB: Rigid,
    IT: DepthFirstIterable<RB, RB::NodeId>,
    FK: Forward<IT, RB>,
    IK: Inverse<IT, RB>,
{
    pub tree: IT,
    // TODO: make this a vec/hashmap of IK FK, to provide a higher level interface
    pub fk: FK,
    pub ik: IK,
    rigid_body: PhantomData<RB>,
}

impl<IT, RB, FK, IK> Mannequin<IT, RB, FK, IK>
where
    RB: Rigid,
    IT: DepthFirstIterable<RB, RB::NodeId>,
    FK: Forward<IT, RB>,
    IK: Inverse<IT, RB>,
{
    pub fn new(tree: IT, forward_kinematics: FK, inverse_kinematics: IK) -> Self {
        Mannequin {
            tree,
            fk: forward_kinematics,
            ik: inverse_kinematics,
            rigid_body: PhantomData,
        }
    }

    /// Forward kinematics for the targets in `target_refs` and the joint positions in `param`.
    #[allow(unused_variables)]
    pub fn forward(
        &mut self,
        _param: &[RB::FloatType], /*, target_refs: &[RB::NodeId]*/
    ) -> Vec<RB::Transformation> {
        // self.fk.solve(&self.tree, param)
        todo!()
    }

    /// Inverse kinematics for the targets in `target_refs` and the desired working space coordinates in `target_val`.
    #[allow(unused_variables)]
    pub fn inverse(&mut self, _param: &mut [RB::FloatType], _target_val: &[RB::Point]) -> IK::Info {
        // self.ik.solve(&self.tree, param, target_val)
        todo!()
    }
}
