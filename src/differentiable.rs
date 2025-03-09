//! Algorithms useful for implementing differential kinematics (such as forward kinematics
//! and partial derivatives (i.e., the Jacobian matrix).
//!
//! The algorithms are independent of
//! the numerical backend and support [f32] and [f64] floating point representations.

use crate::{forward::TransformationAccumulation, DepthFirstIterable, NodeLike, Rigid};
use itertools::{izip, Itertools};
use num_traits::Float;
#[cfg(feature = "rayon")]
use rayon::prelude::*;
use std::{collections::HashSet, fmt::Debug, hash::Hash};

/// Computation shares common intermediate results. This enum
/// allows selecting which results should be computed.
pub enum ComputeSelection {
    /// Compute the forward kinematics
    EffectorsOnly,
    /// Compute only the partial derivatives
    JacobianOnly,
    /// Compute both
    All,
}

/// Mathematical, differentiable representation of a kinematic model. Implementers do the heavy
/// lifting in [crate::ForwardModel] and [crate::DifferentialInverseModel] by computing the
/// Jacobian matrix (partial derivatives) are useful in sovlers. They can be implemented in
/// different ways [[1](https://ieeexplore.ieee.org/document/6177279)] which is the reason for this
/// additional layer of abstraction.
pub trait Differentiable<F: Float> {
    // Document this (blog). It's required for returning a reference to internal data
    // type Data<'a>
    // where
    //     Self: 'a; // https://github.com/rust-lang/rust/issues/87479

    /// returns a reference to the internal data type . Call [Differentiable::compute] first.
    fn jacobian(&self) -> &[F];
    /// Result of the forward kinematics stored in a flat `Vec` to be used in a gradient decent.
    /// Call [Differentiable::compute] first.
    fn flat_effectors(&self) -> &[F];
    /// Result of the forward kinematics as a nested `Vec`. Call [Differentiable::setup] first.
    fn effectors(&self) -> Vec<&[F]>;

    /// Prepare algorithms for computation. This avoids memory allocation when calling [Differentiable::compute].
    fn setup<T, R, I>(&mut self, tree: &T, selected_joints: &[&I], selected_effectors: &[&I])
    where
        T: DepthFirstIterable<R, I>,
        R: Rigid<FloatType = F>,
        I: Eq + Clone + Hash + Debug;

    /// Compute is necessary as the structure holds the memory for the jacobian and the forward vector.
    /// Call [Differentiable::setup] first.
    fn compute<T, R, I>(&mut self, tree: &T, params: &[R::FloatType], selection: ComputeSelection)
    where
        T: DepthFirstIterable<R, I>,
        R: Rigid<FloatType = F>,
        I: Eq + Clone + Hash + Debug;

    /// Get the number of rows of the Jacobian matrix. Call [Differentiable::setup] first.
    fn rows(&self) -> usize;
    /// Get the number of columns of the Jacobian matrix. Call [Differentiable::setup] first.
    fn cols(&self) -> usize;
    /// Get the shape of the Jacobian matrix (rows, columns). Call [Differentiable::setup] first.
    fn shape(&self) -> (usize, usize);

    /// get active joints (those that correspond to columns in the jacobian).
    /// Call [Differentiable::setup] first.
    fn active(&self) -> &[bool];
}
// Note: Won't make the trait itself generic. That would be cleaner but mean more overhead
// (i.e., requiring full qualifiers in compositions)

/// Backend-agnostic implementation of algorithms for computing the forward kinematics
/// and partial derivatives (i.e, Jacobian matrix) or the application in inverse kinematics
/// solvers. Generic in the floating point representation.
#[derive(Debug, Default)]
pub struct DifferentiableModel<F: Float> {
    matrix: Vec<F>,
    configuration: Vec<F>,
    rows: usize,
    cols: usize,
    /// row index at which each effector node starts. Same length as nodes!
    offsets: Vec<usize>,
    /// Widths
    sizes: Vec<usize>,
    /// For each node a bool which decides whether its joint will be used. Same length as nodes!
    selected_joints: Vec<bool>,
    /// For each node a bool which decides whether its effector will be used. Same length as nodes!
    selected_effectors: Vec<bool>,
}

impl<F: Float + Default> DifferentiableModel<F> {
    pub fn new() -> Self {
        Self { ..Default::default() }
    }
}

impl<F: Float> Differentiable<F> for DifferentiableModel<F> {
    fn jacobian(&self) -> &[F] {
        &self.matrix
    }

    fn effectors(&self) -> Vec<&[F]> {
        // &mut col[*offset..*offset + effector_node.get().effector_size()],

        izip!(&self.selected_effectors, &self.offsets, &self.sizes)
            .filter_map(|(&s, &i, &n)| if s { Some(&self.configuration[i..i + n]) } else { None })
            .collect_vec()
    }
    fn flat_effectors(&self) -> &[F] {
        &self.configuration
    }

    fn active(&self) -> &[bool] {
        &self.selected_joints
    }

    fn setup<T, R, I>(&mut self, tree: &T, selected_joints: &[&I], selected_effectors: &[&I])
    where
        T: DepthFirstIterable<R, I>,
        R: Rigid<FloatType = F>,
        I: Eq + Clone + Hash + Debug,
    {
        // todo: fail if node not in tree.

        let selected_effectors: HashSet<&I> = HashSet::from_iter(selected_effectors.iter().copied());
        self.selected_effectors = tree.iter().map(|n| selected_effectors.contains(&n.id())).collect();

        if selected_joints.is_empty() {
            self.selected_joints = vec![true; tree.len()];
        } else {
            let selected_joints: HashSet<&I> = HashSet::from_iter(selected_joints.iter().copied());
            self.selected_joints = tree.iter().map(|n| selected_joints.contains(&n.id())).collect();
        }

        self.offsets = tree
            .iter()
            .scan(0, |offset, node| {
                let result = Some(*offset);
                if selected_effectors.contains(&node.id()) {
                    *offset += node.get().effector_size();
                }
                result
            })
            .collect();

        self.sizes = tree.iter().map(|n| n.get().effector_size()).collect();

        self.rows = tree
            .iter()
            .map(|node| {
                if selected_effectors.contains(&node.id()) {
                    node.get().effector_size()
                } else {
                    0
                }
            })
            .sum();

        self.cols = self.selected_joints.iter().filter(|&selected| *selected).count();
        dbg!((&self.selected_joints, self.rows, self.cols));

        self.matrix.clear();
        self.matrix.resize(self.rows * self.cols, F::zero());

        self.configuration.clear();
        self.configuration.resize(self.rows, F::zero());
    }

    fn rows(&self) -> usize {
        self.rows
    }

    fn cols(&self) -> usize {
        self.cols
    }

    fn shape(&self) -> (usize, usize) {
        (self.rows, self.cols)
    }

    fn compute<T, R, I>(&mut self, tree: &T, params: &[<R as Rigid>::FloatType], selection: ComputeSelection)
    where
        T: DepthFirstIterable<R, I>,
        R: Rigid<FloatType = F>,
        I: Eq + Clone + Hash + Debug,
    {
        debug_assert_eq!(params.len(), tree.len());

        // compute transformations only once
        let nodes_trafos = tree
            .iter()
            .accumulate(params, 42)
            .enumerate()
            .map(|(idx, (node, trafo))| (idx, node, trafo)) // flatten
            .collect_vec();

        if matches!(selection, ComputeSelection::EffectorsOnly | ComputeSelection::All) {
            izip!(&nodes_trafos, &self.selected_effectors, &self.offsets)
                .filter_map(|(x, selected, offset)| if *selected { Some((x, offset)) } else { None })
                .for_each(|((_, node, pose), offset)| {
                    node.get().effector(pose, &mut self.configuration, *offset);
                });
        }

        if matches!(selection, ComputeSelection::JacobianOnly | ComputeSelection::All) {
            self.matrix
                // .iter_mut()
                .chunks_mut(self.rows)
                // TODO use rayon
                .zip(
                    nodes_trafos
                        .iter()
                        .zip(self.selected_joints.iter()) // Add the selected joint lists
                        .filter_map(|(x, selected)| if *selected { Some(x) } else { None }), // filter inactive joints and remove flag
                                                                                             //par_iter()
                )
                .for_each(|(col, (idx, joint_node, joint_pose))| {
                    izip!(
                        tree.iter_sub(joint_node), // iterating over the child tree
                        // zipping the corresponding trafos (by skipping until the current node) and the offsets in the column
                        // Using the index here is ok, keeping an iterator is to hard (gets mutated in a different closure)
                        nodes_trafos.iter().skip(*idx).map(|(_, _, trafo)| trafo),
                        self.offsets.iter().skip(*idx),
                        self.selected_effectors.iter().skip(*idx)
                    )
                    .filter(|(_, _, _, selected)| **selected)
                    .for_each(|(effector_node, effector_pose, offset, _)| {
                        // The slice of the column is itself a column-first matrix
                        effector_node.get().partial_derivative(
                            effector_pose,
                            joint_node.get(),
                            joint_pose,
                            col,
                            // &mut col[*offset..*offset + effector_node.get().effector_size()],
                            *offset,
                        );
                    });
                });
        }
    }
}

#[cfg(test)]
mod tests {

    // The `ndarray` as a reference implementation is used for testing

    use super::*;
    use crate::ndarray::robot::{Axis, LinkNodeId, Segment};
    use crate::{DepthFirstArenaTree, DirectedArenaTree, DirectionIterable};
    use approx::assert_abs_diff_eq;
    use ndarray::{prelude::*, Order};

    #[test]
    fn test_jacobian() {
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

        let mut jacobian = DifferentiableModel::<f64>::new();

        jacobian.setup(
            &tree,
            &[
                &"link1".to_string(),
                &"link2".to_string(),
                &"link3".to_string(),
                &"link4".to_string(),
            ],
            &[&"link2".to_string(), &"link4".to_string()],
        );

        jacobian.compute(
            &tree,
            &[0.0, 0.0, std::f64::consts::FRAC_PI_2, std::f64::consts::FRAC_PI_2, 0.0],
            ComputeSelection::JacobianOnly,
        );

        let result = ArrayView1::<f64>::from(jacobian.jacobian())
            .into_shape_with_order(((jacobian.rows(), jacobian.cols()), Order::ColumnMajor))
            .unwrap();

        let target = array![
            [0.0, 0.0, 0.0, 0.0],
            [20.0, 10.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0,],
            [-10.0, 0.0, -10.0, 0.0],
            [0.0, 0.0, -10.0, -10.0],
            [0.0, 0.0, 0.0, 0.0,]
        ];

        assert_eq!(jacobian.shape(), (6, 4));
        assert_abs_diff_eq!(result, target, epsilon = 1e-6);
    }
}
