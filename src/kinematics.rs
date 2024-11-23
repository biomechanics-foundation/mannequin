use crate::{IterableTree, RigidBody};

pub trait Kinematics<T>: IterableTree<T>
where
    T: RigidBody<Parameter = Self::Parameter>,
{
    type Parameter;
}

pub trait Jacobian<T>: IterableTree<T>
where
    // Avoid mixing of backends
    T: RigidBody<Parameter = Self::Parameter>,
{
    type Array;
    type Parameter;

    fn get_jacobian(&self, param: T) -> Self::Array;
}
