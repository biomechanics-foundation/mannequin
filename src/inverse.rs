use std::collections::HashSet;

use itertools::Itertools;

use crate::{DepthFirstIterable, Differentiable, Nodelike, Rigid};

/// Trait representing a stateful inverse kinematics algoritm.
pub trait Inverse<IT, RB>
where
    IT: DepthFirstIterable<RB, RB::NodeId>,
    RB: Rigid,
{
    type Info;
    fn initialize(&mut self, tree: &IT, selected_joints: &[<RB as Rigid>::NodeId], selected_effectors: &[RB::NodeId]);

    fn solve(&mut self, tree: &IT, param: &mut [RB::Parameter], targets: &[RB::Point]) -> Self::Info;
}

struct InverseKinematicsInfo {
    iteration_count: usize,
    squared_error: f64,
}

/// Reference implementation that is agnostic of the backend
struct DifferentialInverseKinematics<J>
where
    J: Differentiable, // TODO sync somehow the assiciate types
{
    max_depth: usize,
    max_iterations_count: usize,
    selected_effectors: Vec<bool>,
    // MAybe a parameter to accept distances
    jacobian: J,
}

impl<J> DifferentialInverseKinematics<J>
where
    J: Differentiable,
{
    pub fn new(max_depth: usize, max_iterations_count: usize, jacobian: J) -> Self {
        Self {
            max_depth,
            max_iterations_count,
            selected_effectors: vec![],
            jacobian,
        }
    }
}

impl<RB, IT, J> Inverse<IT, RB> for DifferentialInverseKinematics<J>
where
    IT: DepthFirstIterable<RB, RB::NodeId>,
    RB: Rigid,
    J: Differentiable,
{
    type Info = InverseKinematicsInfo;

    fn initialize(
        &mut self,
        tree: &IT,
        selected_joints: &[<RB as Rigid>::NodeId],
        selected_effectors: &[<RB as Rigid>::NodeId],
    ) {
        let selected_effectors = HashSet::from_iter(selected_effectors);
        let selected_joints = HashSet::from_iter(selected_joints);
        self.jacobian.setup(tree, &selected_joints, &selected_effectors);

        // TODO: this should work! maybe after linking the associate in differentiable
        // self.jacobian
        //     .setup(tree, selected_joints.into(), (&selected_effectors).into());
    }

    fn solve(&mut self, tree: &IT, param: &mut [<RB as Rigid>::Parameter], targets: &[RB::Point]) -> Self::Info {
        // Loop until convergence or max_distance
        //  compute Jacobian
        //  vec = compute FK - targets
        //  sum( (vec)^2 )
        //  param = jacobian^-1 * vec
        // return info
        todo!()
    }
}
