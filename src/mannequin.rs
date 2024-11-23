/*! This module defines the flat, composable architecture for a mannequin (an animated character).
 * Different algorithms for forward and inverse kinematics can be freely combined (as long they are
 * using the same backend) such as rigid bodies, deformable skinning,
 * realistic joints, muscle simulation, or classical inverse kinematics and obstacle avoidance.
 */

use crate::{Nodelike, Rigid, TreeIterable};
use std::marker::PhantomData;

/// Trait representing a stateful forward kinematics algoritm. For instance, it can represent a rigid body mannequin
/// (e.g., a robot) or softbody/skinning for character animation.
pub trait Forward<NL, TI, RB>
where
    NL: Nodelike<RB, NodeRef = TI::NodeRef>,
    TI: TreeIterable<NL, RB>,
    RB: Rigid<Parameter = Self::Parameter>,
{
    type Parameter;
    type Array;

    fn solve(&mut self, tree: &TI, param: Self::Parameter, target_refs: &[TI::NodeRef]) -> Self::Array;
}

/// Trait representing a stateful inverse kinematics algoritm.
pub trait Inverse<NL, TI, RB, FK>
where
    NL: Nodelike<RB, NodeRef = TI::NodeRef>,

    TI: TreeIterable<NL, RB>,
    RB: Rigid<Parameter = Self::Parameter>,
    FK: Forward<NL, TI, RB, Parameter = RB::Parameter, Array = Self::Array>,
{
    type Parameter;
    type Array;

    fn solve(
        &mut self,
        tree: &TI,
        fk: &FK,
        param: Self::Parameter,
        target_refs: &[TI::NodeRef],
        target_val: &[Self::Array],
    ) -> Self::Parameter;
}

// pub trait Differential<T>: Inverse<T> {
//     fn get_jacobian(&self, tree: TI, param: Self::Parameter, roots: &[TI::NodeRef]) -> Self::Array;
// }

/// Struct for holding the composition of character animation algorithms in a flat architecture for
/// character animation.
pub struct Mannequin<NL, TI, RB, FK, IK>
where
    RB: Rigid,
    NL: Nodelike<RB, NodeRef = TI::NodeRef>,

    TI: TreeIterable<NL, RB>,
    FK: Forward<NL, TI, RB, Parameter = RB::Parameter>,
    IK: Inverse<NL, TI, RB, FK, Parameter = RB::Parameter, Array = FK::Array>,
{
    pub tree: TI,
    pub fk: FK,
    pub ik: IK,
    rigid_body: PhantomData<RB>,
    node_like: PhantomData<NL>,
}

impl<NL, TI, RB, FK, IK> Mannequin<NL, TI, RB, FK, IK>
where
    RB: Rigid,
    NL: Nodelike<RB, NodeRef = TI::NodeRef>,

    TI: TreeIterable<NL, RB>,
    FK: Forward<NL, TI, RB, Parameter = RB::Parameter>,
    IK: Inverse<NL, TI, RB, FK, Parameter = RB::Parameter, Array = FK::Array>,
{
    pub fn new(tree: TI, foward_kinematics: FK, inverse_kinematics: IK) -> Self {
        Mannequin {
            tree,
            fk: foward_kinematics,
            ik: inverse_kinematics,
            rigid_body: PhantomData,
            node_like: PhantomData,
        }
    }

    /// Forward kinematics for the targets in `target_refs` and the joint positions in `param`.
    pub fn forward(&mut self, param: FK::Parameter, target_refs: &[TI::NodeRef]) -> FK::Array {
        self.fk.solve(&self.tree, param, target_refs)
    }

    /// Inverse kinematics for the targets in `target_refs` and the desired working space coordinates in `target_val`.
    pub fn inverse(
        &mut self,
        param: IK::Parameter,
        target_refs: &[TI::NodeRef],
        target_val: &[IK::Array],
    ) -> IK::Parameter {
        self.ik.solve(&self.tree, &self.fk, param, target_refs, target_val)
    }
}
