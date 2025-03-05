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

    fn setup(&mut self, tree: &IT, selected_joints: &[&<RB as Rigid>::NodeId], selected_effectors: &[&RB::NodeId]);

    fn solve(
        &mut self,
        tree: &IT,
        param: &mut [<RB as Rigid>::FloatType],
        targets: &[<RB as Rigid>::FloatType],
    ) -> Self::Info;
}

#[derive(Debug, Clone)]
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

    fn setup(
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

#[cfg(test)]
mod test {
    // The `ndarray` as a reference implementation is used for testing

    use super::*;
    use crate::ndarray::robot::{Axis, LinkNodeId, Segment};
    use crate::{DepthFirstArenaTree, DifferentiableModel, DirectedArenaTree, DirectionIterable};
    use approx::assert_abs_diff_eq;
    use ndarray::{prelude::*, Order};

    #[test]
    fn test_ik() {
        let mut tree = DirectedArenaTree::<Segment, LinkNodeId>::new();

        let mut trafo = Segment::neutral_element();
        trafo.slice_mut(s![..3, 3]).assign(&array![10.0, 0.0, 0.0]);

        let link1 = Segment::new(&trafo, Axis::RotationZ, None);
        let link2 = Segment::new(&trafo, Axis::RotationZ, Some(trafo.clone()));
        let link3 = Segment::new(&trafo, Axis::RotationZ, None);
        let link4 = Segment::new(&trafo, Axis::RotationZ, Some(trafo.clone()));
        // Doesn't do anything, is at the end
        let link5 = Segment::new(&trafo, Axis::RotationZ, Some(trafo.clone()));

        // TODO .. can we make the refs fix in a way they don't get optimized away?
        // Then these could be strings even!

        let ref1 = tree.set_root(link1, "link1".to_string());
        let _ref2 = tree.add(link2, "link2".to_string(), &ref1).unwrap();
        let ref3 = tree.add(link3, "link3".to_string(), &ref1).unwrap();
        let ref4 = tree.add(link4, "link4".to_string(), &ref3).unwrap();
        tree.add(link5, "link5".to_string(), &ref4).unwrap();
        let tree: DepthFirstArenaTree<_, _> = tree.into();

        let mut ik = DifferentialInverseModel::new(42, 10, 0.01, DifferentiableModel::new());

        ik.setup(
            &tree,
            &[
                &"link1".to_string(),
                &"link2".to_string(),
                &"link3".to_string(),
                &"link4".to_string(),
            ],
            &[&"link2".to_string(), &"link4".to_string()],
        );

        let effectors = vec![vec![20.0, 0.0, 0.0], vec![20.0, 10.0, 0.0]];
        let effectors = effectors.into_iter().flatten().collect_vec();
        let mut param = vec![0.0; 5];

        let result = ik.solve(&tree, &mut param, &effectors);

        dbg!(param);
        dbg!(result);
        // assert_abs_diff_eq!(result, target, epsilon = 1e-6);
    }
}
