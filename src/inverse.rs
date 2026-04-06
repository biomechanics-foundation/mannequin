//! Interface and basic implementor for the inverse kinematic model.

use std::{fmt::Debug, hash::Hash, iter::Sum};

use itertools::{izip, Itertools};
use num_traits::Float;

use crate::{
    differentiable::Filterable, forward::Forward, Articulated, DepthFirstIterable, Differentiable, NodeLike, Rigid,
};

pub trait Invertable<NodeType, LoadType, TreeType, IdType>: Articulated<NodeType, LoadType, TreeType, IdType>
where
    LoadType: Rigid,
    NodeType: NodeLike<LoadType, IdType>,
    IdType: Eq + Clone + Hash + Debug,
{
    type Config;
    type Model<'x, 'y, 'z>: Inverse<LoadType::FloatType>
    where
        Self: 'x;

    fn inverse<'a, 'b, 'c>(
        &'a self,
        forward_config: &'b <Self as Articulated<NodeType, LoadType, TreeType, IdType>>::Config,
        inverse_config: &'c <Self as Invertable<NodeType, LoadType, TreeType, IdType>>::Config,
    ) -> <Self as Invertable<NodeType, LoadType, TreeType, IdType>>::Model<'a, 'b, 'c>;
}

impl<NodeType, LoadType, TreeType, IdType, FloatType> Invertable<NodeType, LoadType, TreeType, IdType> for TreeType
where
    LoadType: Rigid<FloatType = FloatType>,
    FloatType: Float + Sum + Debug + 'static,
    NodeType: NodeLike<LoadType, IdType>,
    IdType: Eq + Clone + Hash + Debug,
    TreeType: DepthFirstIterable<LoadType, IdType, Node = NodeType>,
{
    type Config = InverseConfig<LoadType::FloatType>;

    type Model<'x, 'y, 'z>
        = DifferentialIK<'x, 'y, 'z, NodeType, LoadType, TreeType, IdType>
    where
        Self: 'x;

    fn inverse<'a, 'b, 'c>(
        &'a self,
        forward_config: &'b <Self as Articulated<NodeType, LoadType, TreeType, IdType>>::Config,
        inverse_config: &'c <Self as Invertable<NodeType, LoadType, TreeType, IdType>>::Config,
    ) -> <Self as Invertable<NodeType, LoadType, TreeType, IdType>>::Model<'a, 'b, 'c> {
        DifferentialIK {
            tree: self,
            forward_config,
            inverse_config,
        }
    }
}

pub struct InverseConfig<F: Float> {
    inital: Vec<F>,

    max_iterations_count: usize,
    scale_difference: F,
    min_error: F,
}

pub struct DifferentialIK<'a, 'b, 'c, NodeType, LoadType, TreeType, IdType>
where
    LoadType: Rigid,
    NodeType: NodeLike<LoadType, IdType>,
    TreeType: DepthFirstIterable<LoadType, IdType, Node = NodeType>,
    IdType: Eq + Clone + Hash + Debug,
{
    pub tree: &'a TreeType,
    pub forward_config: &'b <TreeType as Articulated<NodeType, LoadType, TreeType, IdType>>::Config,
    pub inverse_config: &'c InverseConfig<LoadType::FloatType>,
}

pub trait Inverse<F> {
    type Info;

    fn solve(&self, targets: &[F]) -> (Vec<F>, Self::Info);
}

pub struct DifferentialIKResult<F>
where
    F: Float + Debug + Sum,
{
    pub iteration_count: usize,
    pub squared_error: F,
}

impl<'a, 'b, 'c, NodeType, LoadType, TreeType, IdType, FloatType> Inverse<FloatType>
    for DifferentialIK<'a, 'b, 'c, NodeType, LoadType, TreeType, IdType>
where
    LoadType: Rigid<FloatType = FloatType>,
    FloatType: Float + Debug + Sum,
    NodeType: NodeLike<LoadType, IdType>,
    TreeType: 'a + DepthFirstIterable<LoadType, IdType, Node = NodeType>,
    IdType: Eq + Clone + Hash + Debug,
{
    type Info = DifferentialIKResult<FloatType>;

    fn solve(&self, targets: &[FloatType]) -> (Vec<FloatType>, Self::Info) {
        let mut counter = 0;
        let mut error: FloatType;
        let mut result = vec![FloatType::zero(); self.forward_config.rows]; // .differential_model.active().iter().filter(|i| **i).count()];

        let mut params = self.inverse_config.inital.clone();

        loop {
            dbg!(counter);

            let forward = self.tree.pose(&params, self.forward_config);
            // dbg!(&params);
            //
            let effectors = forward.flat_effectors();
            dbg!(forward.flat_effectors());
            // dbg!(self.differential_model.effectors());
            let mut diff = izip!(targets, effectors).map(|(x, y)| (*x - y)).collect_vec();

            // dbg!(&self.differential_model.jacobian());
            error = diff.iter().map(|x| *x * *x).sum();
            dbg!(&error);
            // dbg!(&diff);

            diff.iter_mut()
                .for_each(|x| *x = *x * self.inverse_config.scale_difference);

            LoadType::solve_linear(
                &forward.jacobian(),
                self.forward_config.rows,
                self.forward_config.cols,
                &diff,
                &mut result,
            );

            // dbg!(&result);
            // dbg!(&params);

            params
                .iter_mut()
                .filter_active(&self.forward_config.selected_joints)
                .zip(&result)
                .for_each(|(p, r)| *p = *p + *r);

            if error < self.inverse_config.min_error {
                break;
            }
            counter += 1;
            if counter >= self.inverse_config.max_iterations_count {
                break;
            }
        }

        (
            params,
            Self::Info {
                iteration_count: counter,
                squared_error: error,
            },
        )
    }
}

/* /// Trait representing a stateful inverse kinematics algorithm.
///
/// It allows selecting the effectors to be computed and thus a specific (or multiple) kinematic chain(s), and
/// which joints are active (can be moved).
pub trait Inverse<TreeType, LoadType>
where
    TreeType: DepthFirstIterable<LoadType, LoadType::NodeId>,
    LoadType: Rigid,
{
    /// The return type com [Inverse::solve] that carries information about the outcome
    type Info;

    /// Preparation the computation (memory allocation, and joint/effector selection).
    fn setup(
        &mut self,
        tree: &TreeType,
        selected_joints: &[&<LoadType as Rigid>::NodeId],
        selected_effectors: &[&LoadType::NodeId],
    );

    /// Compute the inverse kinematics
    fn solve(
        &mut self,
        tree: &TreeType,
        param: &mut [<LoadType as Rigid>::FloatType],
        targets: &[<LoadType as Rigid>::FloatType],
    ) -> Self::Info;
}

/// Information about the solution for the default, differential IK solver
#[derive(Debug, Clone)]
pub struct DiffIKInfo<F: Float> {
    /// Number of required iterations (indicates convergence)
    pub iteration_count: usize,
    /// The final squared error
    pub squared_error: F,
}

/// Reference implementation of a differential IK solver that is agnostic of the backend.
///
/// It delegates linear algebra operations (solving the system of linear equations) to
/// backend-specific implementers of [crate::Rigid], and uses a backend-agnostic
/// [Differentiable] for computing the Jacobian matrix.
pub struct DifferentialInverseModel<FloatType, TreeType>
where
    FloatType: Float,
    TreeType: Differentiable<FloatType>,
{
    _max_depth: usize,
    max_iterations_count: usize,
    min_error: FloatType,
    differential_model: TreeType,
    scale_difference: FloatType,
}

impl<FloatType, DifferentiableType> DifferentialInverseModel<FloatType, DifferentiableType>
where
    FloatType: Float,
    DifferentiableType: Differentiable<FloatType>,
{
    pub fn new(
        _max_depth: usize,
        max_iterations_count: usize,
        min_error: FloatType,
        differential_model: DifferentiableType,
        scale_difference: FloatType,
    ) -> Self {
        Self {
            _max_depth,
            max_iterations_count,
            min_error,
            differential_model,
            scale_difference,
        }
    }
}

impl<RB, IT, F, D> Inverse<IT, RB> for DifferentialInverseModel<F, D>
where
    IT: DepthFirstIterable<RB, RB::NodeId>,
    RB: Rigid<FloatType = F>,
    F: Float + Sum + Debug,
    D: Differentiable<F>,
{
    type Info = DiffIKInfo<F>;

    fn setup(
        &mut self,
        tree: &IT,
        selected_joints: &[&<RB as Rigid>::NodeId],
        selected_effectors: &[&<RB as Rigid>::NodeId],
    ) {
        self.differential_model.setup(tree, selected_joints, selected_effectors);
    }

    // TODO Think about turning this into an iterator ...
    fn solve(&mut self, tree: &IT, params: &mut [F], targets: &[F]) -> Self::Info {
        let mut counter = 0;
        let mut error: F;
        let mut result = vec![F::zero(); self.differential_model.active().iter().filter(|i| **i).count()];
        loop {
            dbg!(counter);
            self.differential_model.compute(tree, params, ComputeSelection::All);
            // dbg!(&params);
            dbg!(self.differential_model.flat_effectors());
            // dbg!(self.differential_model.effectors());
            let mut diff = izip!(targets, self.differential_model.flat_effectors())
                .map(|(x, y)| (*x - *y))
                .collect_vec();

            // dbg!(&self.differential_model.jacobian());
            error = diff.iter().map(|x| *x * *x).sum();
            dbg!(&error);
            // dbg!(&diff);

            diff.iter_mut().for_each(|x| *x = *x * self.scale_difference);

            RB::solve_linear(
                self.differential_model.jacobian(),
                self.differential_model.rows(),
                self.differential_model.cols(),
                &diff,
                &mut result,
            );

            // dbg!(&result);
            // dbg!(&params);

            params
                .iter_mut()
                .filter_active(self.differential_model.active())
                .zip(&result)
                .for_each(|(p, r)| *p = *p + *r);

            if error < self.min_error {
                break;
            }
            counter += 1;
            if counter >= self.max_iterations_count {
                break;
            }
        }

        Self::Info {
            iteration_count: counter,
            squared_error: error,
        }
    }
} */

#[cfg(feature = "ndarray")]
#[cfg(test)]
mod test {
    // The `ndarray` as a reference implementation is used for testing

    use super::*;
    use crate::arena::iterables::OptimizedDirectionIterable;
    use crate::ndarray::robot::{Axis, LinkNodeId, Segment};
    use crate::{DepthFirstArenaTree, DifferentiableModel, DirectedArenaTree, DirectionIterable};
    use ndarray::prelude::*;

    #[cfg(feature = "ndarray")]
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

        // let mut ik = DifferentialInverseModel::new(42, 10, 0.01, DifferentiableModel::new());
        let n_iterations = 13;
        let mut ik = DifferentialInverseModel::new(42, n_iterations, 0.01, DifferentiableModel::new(), 1.0);

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
        // let mut param = vec![0.0; 5];
        let mut param = vec![0.0, 0.0, std::f64::consts::FRAC_PI_2, std::f64::consts::FRAC_PI_2, 0.0];

        let result = ik.solve(&tree, &mut param, &effectors);

        assert!(result.iteration_count <= n_iterations);
        assert!(result.squared_error < 1e-2);
        dbg!(param);
        dbg!(result);
        // assert_abs_diff_eq!(result.er, target, epsilon = 1e-6);
        // assert!(x.abs_diff_eq(&array![1., -2., -2.], 1e-9));
        // assert_abs_diff_eq!(result, target, epsilon = 1e-6);
    }

    #[test]
    fn test_tentacle() {
        let mut tree = DirectedArenaTree::<Segment, LinkNodeId>::new();

        let mut trafo = Segment::neutral_element();
        trafo.slice_mut(s![..3, 3]).assign(&array![10.0, 0.0, 0.0]);

        let mut last_node_id = tree.set_root(Segment::new(&trafo, Axis::RotationZ, None), "link_0".into());

        for i in 1..9 {
            last_node_id = tree
                .add(
                    Segment::new(&trafo, Axis::RotationZ, None),
                    format!("link_{i}"),
                    &last_node_id,
                )
                .unwrap();
        }

        tree.add(
            Segment::new(&trafo, Axis::RotationZ, Some(trafo.clone())),
            "link_9".into(),
            &last_node_id,
        )
        .unwrap();

        // finalize tree
        let tree: DepthFirstArenaTree<_, _> = tree.into();
        tree.iter().for_each(|n| {
            dbg!(&n);
        });

        let n_iterations = 13;
        let mut ik = DifferentialInverseModel::new(42, n_iterations, 0.01, DifferentiableModel::new(), 0.001);

        ik.setup(&tree, &[], &[&"link_9".to_string()]);

        let effectors = vec![vec![00.0, 20.0, 0.0]];

        let effectors = effectors.into_iter().flatten().collect_vec();
        // let mut param = vec![0.0; 5];
        let mut param = vec![0.0; 10];

        let result = ik.solve(&tree, &mut param, &effectors);

        // assert_eq!(result.iteration_count, n_iterations);
        dbg!(param);
        dbg!(result);
        // assert!(x.abs_diff_eq(&array![1., -2., -2.], 1e-9));
        // assert_abs_diff_eq!(result, target, epsilon = 1e-6);
    }
}
