/* TODO: We should make different implementation configuratble (cf. flat architecture) */

use std::marker::PhantomData;

use crate::{IterableTree, RigidBody};

pub trait Forward<IT, RB>
where
    IT: IterableTree<RB>,
    RB: RigidBody<Parameter = Self::Parameter>,
{
    type Parameter;
    type Array;

    fn solve(&self, tree: &IT, param: Self::Parameter, target_refs: &[IT::NodeRef]) -> Self::Array;
}

pub trait Inverse<IT, RB>
where
    // Avoid mixing of backends
    IT: IterableTree<RB>,
    RB: RigidBody<Parameter = Self::Parameter>,
{
    type Parameter;
    type Array;

    fn solve(
        &self,
        tree: &IT,
        param: Self::Parameter,
        target_refs: &[IT::NodeRef],
        target_val: &[Self::Array],
    ) -> Self::Parameter;
}

// pub trait Differential<T>: Inverse<T> {
//     fn get_jacobian(&self, tree: IT, param: Self::Parameter, roots: &[IT::NodeRef]) -> Self::Array;
// }

pub struct Mannequin<IT, RB, FK, IK>
where
    RB: RigidBody,
    IT: IterableTree<RB>,
    FK: Forward<IT, RB, Parameter = RB::Parameter>,
    IK: Inverse<IT, RB, Parameter = RB::Parameter, Array = FK::Array>,
{
    pub tree: IT,
    pub forward: FK,
    pub inverse: IK,
    rigib_body: PhantomData<RB>,
}
