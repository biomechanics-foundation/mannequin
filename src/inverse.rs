use std::collections::HashSet;

use itertools::{izip, Itertools};

use crate::{
    differentiable::ComputeSelection, DepthFirstIterable, Differentiable, DifferentiableModel, Nodelike, Rigid,
};

/// Trait representing a stateful inverse kinematics algoritm.
pub trait Inverse<IT, RB>
where
    IT: DepthFirstIterable<RB, RB::NodeId>,
    RB: Rigid,
{
    type Info;
    type Data<'a>;

    fn initialize(&mut self, tree: &IT, selected_joints: &[<RB as Rigid>::NodeId], selected_effectors: &[RB::NodeId]);

    fn solve(&mut self, tree: &IT, param: &mut [RB::Parameter], targets: Self::Data<'_>) -> Self::Info;
}

struct InverseKinematicsInfo {
    iteration_count: usize,
    squared_error: f64,
}

/// Reference implementation that is agnostic of the backend
struct DifferentialInverseKinematics
// TODO sync somehow the assiciate types
{
    max_depth: usize,
    max_iterations_count: usize,
    min_error: f64,
    selected_effectors: Vec<bool>,
    // TODO: I want this to be generic but this fails when binding the GATs
    model: DifferentiableModel,
    // model: DM,
}

impl DifferentialInverseKinematics {
    pub fn new(max_depth: usize, max_iterations_count: usize, min_error: f64, jacobian: DifferentiableModel) -> Self {
        Self {
            max_depth,
            max_iterations_count,
            min_error,
            selected_effectors: vec![],
            model: jacobian,
        }
    }
}

impl<RB, IT> Inverse<IT, RB> for DifferentialInverseKinematics
where
    IT: DepthFirstIterable<RB, RB::NodeId>,
    RB: Rigid,
    // DM: 'b + Differentiable<Data<'b> = &'b [f64]>, // Cannot not use Self::Data here (!?)
{
    type Info = InverseKinematicsInfo;
    type Data<'a> = &'a [f64];

    fn initialize(
        &mut self,
        tree: &IT,
        selected_joints: &[<RB as Rigid>::NodeId],
        selected_effectors: &[<RB as Rigid>::NodeId],
    ) {
        let selected_effectors = HashSet::from_iter(selected_effectors);
        let selected_joints = HashSet::from_iter(selected_joints);
        self.model.setup(tree, &selected_joints, &selected_effectors);
    }

    fn solve(&mut self, tree: &IT, params: &mut [<RB as Rigid>::Parameter], targets: &[f64]) -> Self::Info {
        self.model.compute(tree, params, ComputeSelection::All);

        let mut counter = 0;
        let mut error;
        loop {
            let diff = izip!(self.model.configuration(), targets)
                .map(|(x, y)| x - y)
                .collect_vec();

            RB::solve_linear(
                self.model.jacobian(),
                self.model.rows(),
                self.model.cols(),
                &diff,
                params,
            );

            error = diff.iter().map(|x| x * x).sum();

            if error < self.min_error {
                break;
            }
            counter += 1;
            if counter > self.max_iterations_count {
                break;
            }
        }

        Self::Info {
            iteration_count: counter,
            squared_error: error,
        }
    }
}
