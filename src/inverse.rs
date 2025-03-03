use crate::{DepthFirstIterable, Forward, Rigid};

/// Trait representing a stateful inverse kinematics algoritm.
pub trait Inverse<IT, RB, FK>
where
    // Avoid mixing of backends
    IT: DepthFirstIterable<RB, RB::NodeId>,

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
        target_refs: &[RB::NodeId],
        target_val: &[Self::Array],
    ) -> Self::Parameter;
}
