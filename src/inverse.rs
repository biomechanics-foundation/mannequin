use std::{collections::HashSet, iter::Sum};

use itertools::{izip, Itertools};
use num_traits::Float;

use crate::{differentiable::ComputeSelection, DepthFirstIterable, Differentiable, Rigid};

/// Trait representing a stateful inverse kinematics algorithm.
pub trait Inverse<IT, RB>
where
    IT: DepthFirstIterable<RB, RB::NodeId>,
    RB: Rigid,
{
    type Info;

    fn initialize(&mut self, tree: &IT, selected_joints: &[&<RB as Rigid>::NodeId], selected_effectors: &[&RB::NodeId]);

    fn solve(
        &mut self,
        tree: &IT,
        param: &mut [<RB as Rigid>::FloatType],
        targets: &[<RB as Rigid>::FloatType],
    ) -> Self::Info;
}

pub struct InverseKinematicsInfo<F: Float> {
    pub iteration_count: usize,
    pub squared_error: F,
}

/// Reference implementation that is agnostic of the backend
pub struct DifferentialInverseModel<F, D>
where
    F: Float,
    D: Differentiable<F>,
{
    _max_depth: usize,
    max_iterations_count: usize,
    min_error: F,
    differential_model: D,
}

impl<F, D> DifferentialInverseModel<F, D>
where
    F: Float,
    D: Differentiable<F>,
{
    pub fn new(_max_depth: usize, max_iterations_count: usize, min_error: F, differential_model: D) -> Self {
        Self {
            _max_depth,
            max_iterations_count,
            min_error,
            differential_model,
        }
    }
}

impl<RB, IT, F, D> Inverse<IT, RB> for DifferentialInverseModel<F, D>
where
    IT: DepthFirstIterable<RB, RB::NodeId>,
    RB: Rigid<FloatType = F>,
    F: Float + Sum,
    D: Differentiable<F>,
    // DM: 'b + Differentiable<Data<'b> = &'b [f64]>, // Cannot not use Self::Data here (!?)
{
    type Info = InverseKinematicsInfo<F>;

    fn initialize(
        &mut self,
        tree: &IT,
        selected_joints: &[&<RB as Rigid>::NodeId],
        selected_effectors: &[&<RB as Rigid>::NodeId],
    ) {
        let selected_effectors = HashSet::from_iter(selected_effectors.iter().cloned());
        let selected_joints = HashSet::from_iter(selected_joints.iter().cloned());
        self.differential_model
            .setup(tree, &selected_joints, &selected_effectors);
    }

    fn solve(&mut self, tree: &IT, params: &mut [F], targets: &[F]) -> Self::Info {
        self.differential_model.compute(tree, params, ComputeSelection::All);

        let mut counter = 0;
        let mut error: F;
        loop {
            let diff = izip!(self.differential_model.flat_effectors(), targets)
                .map(|(x, y)| *x - *y)
                .collect_vec();

            RB::solve_linear(
                self.differential_model.jacobian(),
                self.differential_model.rows(),
                self.differential_model.cols(),
                &diff,
                params,
            );

            error = diff.iter().map(|x| *x * *x).sum();

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
