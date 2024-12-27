/*! This module defines the flat, composable architecture for a mannequin (an animated character).
 * Different algorithms for forward and inverse kinematics can be freely combined (as long they are
 * using the same backend) such as rigid bodies, deformable skinning,
 * realistic joints, muscle simulation, or classical inverse kinematics and obstacle avoidance.
 */

use crate::{Order::DepthFirst, Rigid, TransformationAccumulation, TreeIterable};
use std::marker::PhantomData;

/// Trait representing a stateful forward kinematics algoritm. For instance, it can represent a rigid body mannequin
/// (e.g., a robot) or softbody/skinning for character animation.
pub trait Forward<IT, RB>
where
    IT: TreeIterable<RB>,
    RB: Rigid,
{
    // TODO maybe change to slice
    type Parameter: IntoIterator<Item = RB::Parameter>;
    type Transformation;

    // TODO this can have a default implementation
    fn solve(&mut self, tree: &IT, params: Self::Parameter, target_refs: &[IT::NodeRef]) -> Vec<Self::Transformation>;
}

/// Trait representing a stateful inverse kinematics algoritm.
pub trait Inverse<IT, RB, FK>
where
    // Avoid mixing of backends
    IT: TreeIterable<RB>,
    RB: Rigid,
    FK: Forward<IT, RB, Transformation = Self::Array>,
{
    type Parameter: IntoIterator<Item = RB::Parameter>;
    type Array;

    fn solve(
        &mut self,
        tree: &IT,
        fk: &FK,
        param: Self::Parameter,
        target_refs: &[IT::NodeRef],
        target_val: &[Self::Array],
    ) -> Self::Parameter;
}

// pub trait Differential<T>: Inverse<T> {
//     fn get_jacobian(&self, tree: IT, param: Self::Parameter, roots: &[IT::NodeRef]) -> Self::Array;
// }

/// Struct for holding the composition of character animation algorithms in a flat architecture for
/// character animation.
pub struct Mannequin<IT, RB, FK, IK>
where
    RB: Rigid,
    IT: TreeIterable<RB>,
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
    IT: TreeIterable<RB>,
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
    pub fn forward(&mut self, param: FK::Parameter, target_refs: &[IT::NodeRef]) -> Vec<FK::Transformation> {
        self.fk.solve(&self.tree, param, target_refs)
    }

    /// Inverse kinematics for the targets in `target_refs` and the desired working space coordinates in `target_val`.
    pub fn inverse(
        &mut self,
        param: IK::Parameter,
        target_refs: &[IT::NodeRef],
        target_val: &[IK::Array],
    ) -> IK::Parameter {
        self.ik.solve(&self.tree, &self.fk, param, target_refs, target_val)
    }
}
