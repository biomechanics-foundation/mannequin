//! Algorithms useful for implementing differential kinematics (such as forward kinematics
//! and partial derivatives (i.e., the Jacobian matrix).
//!
//! The algorithms are independent of
//! the numerical backend and support [f32] and [f64] floating point representations.

use crate::{DepthFirstIterable, ForwardModel, NodeLike, Rigid};
use itertools::{izip, Itertools};
use num_traits::Float;
#[cfg(feature = "rayon")]
use rayon::prelude::*;
use std::{fmt::Debug, hash::Hash};

pub trait Differentiable<T> {
    fn jacobian(&self) -> Vec<T>;
}

impl<'a, 'b, NodeType, LoadType, TreeType, IdType, FloatType> Differentiable<FloatType>
    for ForwardModel<'a, 'b, NodeType, LoadType, TreeType, IdType>
where
    LoadType: Rigid<FloatType = FloatType>,
    FloatType: Float,
    NodeType: NodeLike<LoadType, IdType>,
    TreeType: 'a + DepthFirstIterable<LoadType, IdType, Node = NodeType>,
    IdType: Eq + Clone + Hash + Debug,
{
    fn jacobian(&self) -> Vec<FloatType> {
        let mut jacobian = vec![FloatType::zero(); self.config.rows * self.config.cols];

        jacobian
            // .iter_mut()
            // FIXME: Row below can panic .. handle errors
            // Column-major!
            .chunks_mut(self.config.rows)
            // TODO use rayon
            .zip(
                izip!(self.tree.iter(), &self.transformations, &self.config.selected_joints)
                    // self.transformations
                    // .iter()
                    // .zip(self.selected_joints.iter()) // Add the selected joint lists
                    .filter_map(|(node, trafo, selected)| if *selected { Some((node, trafo)) } else { None }), // filter inactive joints and remove flag
                                                                                                               //par_iter()
            )
            .enumerate()
            .for_each(|(idx, (col, (joint_node, joint_pose)))| {
                izip!(
                    self.tree.iter_sub(joint_node), // iterating over the child tree
                    // zipping the corresponding trafos (by skipping until the current node) and the offsets in the column
                    // Using the index here is ok, keeping an iterator is to hard (gets mutated in a different closure)
                    self.transformations.iter().skip(idx),
                    self.config.offsets.iter().skip(idx),
                    self.config.selected_effectors.iter().skip(idx)
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
        jacobian
    }
}

/// Helper trait that is implemented for all iterators. Is used
/// to filter a sequence by the output of [Differentiable::active].
///
/// Example:
///
/// ```rs
/// current_angles
///    .iter_mut() // iterate over all to update
///    .filter_active(self.differentiable.active()) // modify only active joints
///    .zip(&result) // get updates and update
///    .for_each(|(angle, update)| { *angle += update });
/// ```
pub trait Filterable<T> {
    fn filter_active(self, active: &[bool]) -> impl Iterator<Item = T>;
}

impl<T, I> Filterable<T> for I
where
    I: Iterator<Item = T>,
{
    fn filter_active(self, active: &[bool]) -> impl Iterator<Item = T> {
        self.zip(active.iter())
            .filter_map(|(a, b)| if *b { Some(a) } else { None })
    }
}
// /// Computation shares common intermediate results. This enum
// /// allows selecting which results should be computed.
// pub enum ComputeSelection {
//     /// Compute the forward kinematics
//     EffectorsOnly,
//     /// Compute only the partial derivatives
//     JacobianOnly,
//     /// Compute both
//     All,
// }

// struct Differentiated<F: Float> {
//     /// Jacobian matrix as a flattened, column-major array.
//     jacobian: Vec<F>,
//     /// Flattened result of the forwards kinematics.
//     effectors: Vec<F>,
// }

/*
/// Mathematical, differentiable representation of a kinematic model. Implementers do the heavy
/// lifting in [crate::ForwardModel] and [crate::DifferentialInverseModel] by computing the
/// Jacobian matrix (partial derivatives) are useful in solvers. They can be implemented in
/// different ways [[1](https://ieeexplore.ieee.org/document/6177279)] which is the reason for this
/// additional layer of abstraction.
*/

// pub trait DifferentiableOld<F, T, I, R>
// where
//     T: DepthFirstIterable<R, I>,
//     I: Eq + Clone + Hash + Debug,
//     R: Rigid<FloatType = F>,
//     F: Float,
// {
//     // Document this (blog). It's required for returning a reference to internal data
//     // type Data<'a>
//     // where
//     //     Self: 'a; // https://github.com/rust-lang/rust/issues/87479

//     /*
//     /// returns a reference to the internal data type . Call [Differentiable::compute] first. Column-major!
//     fn jacobian(&self) -> &[F];
//     /// Result of the forward kinematics stored in a flat `Vec` to be used in a gradient decent.
//     /// Call [Differentiable::compute] first.
//     fn flat_effectors(&self) -> &[F];
//     /// Result of the forward kinematics as a nested `Vec`. Call [Differentiable::setup] first.
//     fn effectors(&self) -> Vec<&[F]>;

//     /// Prepare algorithms for computation. This avoids memory allocation when calling [Differentiable::compute].
//     ///
//     /// Warning! The order of the IDs in `selected_joints` and `selected_effectors` does not matter (they are may be
//     /// converted into a [HashSet] immediately). The methods [Differentiable::effectors], [Differentiable::flat_effectors],
//     /// [Differentiable::jacobian], is determined only by the order of the nodes in the tree!
//     fn setup<T, R, I>(&mut self, tree: &T, selected_joints: &[&I], selected_effectors: &[&I])
//     where
//         T: DepthFirstIterable<R, I>,
//         R: Rigid<FloatType = F>,
//         I: Eq + Clone + Hash + Debug;
//     */
//     fn differentiable<'a, 'b>(
//         &'a self,
//         params: &'b [<R as Rigid>::FloatType],
//         selected_joints: &[&I],
//         selected_effectors: &[&I],
//     ) -> DifferentiableModel<'a, 'b, F, T, R, I>;
// }

/* impl<F, R, I, T> DifferentiableOld<F, T, I, R> for T
where
    T: DepthFirstIterable<R, I>,
    R: Rigid<FloatType = F>,
    I: Eq + Clone + Hash + Debug,
    F: Float,
{
    fn differentiable<'a, 'b>(
        &'a self,
        params: &'b [<R as Rigid>::FloatType],
        selected_joints: &[&I],
        selected_effectors: &[&I],
    ) -> DifferentiableModel<'a, 'b, F, T, R, I> {
        // fn setup<T, R, I>(&mut self, tree: &T, selected_joints: &[&I], selected_effectors: &[&I])
        // where
        //     T: DepthFirstIterable<R, I>,
        //     R: Rigid<FloatType = F>,
        //     I: Eq + Clone + Hash + Debug,
        // {
        // todo: fail if node not in tree.

        let selected_effectors_map: HashSet<&I> = HashSet::from_iter(selected_effectors.iter().copied());
        let selected_effectors = self.iter().map(|n| selected_effectors_map.contains(&n.id())).collect();

        let selected_joints = if selected_joints.is_empty() {
            vec![true; self.len()]
        } else {
            let selected_joints: HashSet<&I> = HashSet::from_iter(selected_joints.iter().copied());
            self.iter().map(|n| selected_joints.contains(&n.id())).collect()
        };

        let offsets = self
            .iter()
            .scan(0, |offset, node| {
                let result = Some(*offset);
                if selected_effectors_map.contains(&node.id()) {
                    *offset += node.get().effector_size();
                }
                result
            })
            .collect();

        let sizes = self.iter().map(|n| n.get().effector_size()).collect();

        let rows = self
            .iter()
            .map(|node| {
                if selected_effectors_map.contains(&node.id()) {
                    node.get().effector_size()
                } else {
                    0
                }
            })
            .sum();

        let cols = selected_joints.iter().filter(|&selected| *selected).count();
        dbg!((&selected_joints, rows, cols));

        // Compute forward kinematics.
        let transformations = self
            .iter()
            .accumulate(params, 42)
            // TODO remove --> enumeration and iteration over nodes can be done later.
            // .enumerate()
            // .map(|(idx, (node, trafo))| (idx, node, trafo)) // flatten
            .map(|(_, trafo)| trafo)
            .collect_vec();

        DifferentiableModel {
            _i: PhantomData,
            _r: PhantomData,
            cols,
            offsets,
            rows,
            selected_effectors,
            selected_joints,
            sizes,
            tree: &self,
            params: &params,
            transformations,
        }
    }
} */

/*
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
*/

/* /// Helper trait that is implemented for all iterators. Is used
/// to filter a sequence by the output of [Differentiable::active].
///
/// Example:
///
/// ```rs
/// current_angles
///    .iter_mut() // iterate over all to update
///    .filter_active(self.differentiable.active()) // modify only active joints
///    .zip(&result) // get updates and update
///    .for_each(|(angle, update)| { *angle += update });
/// ```
pub trait Filterable<T> {
    fn filter_active(self, active: &[bool]) -> impl Iterator<Item = T>;
}

impl<T, I> Filterable<T> for I
where
    I: Iterator<Item = T>,
{
    fn filter_active(self, active: &[bool]) -> impl Iterator<Item = T> {
        self.zip(active.iter())
            .filter_map(|(a, b)| if *b { Some(a) } else { None })
    }
} */

// Note: Won't make the trait itself generic. That would be cleaner but mean more overhead
// (i.e., requiring full qualifiers in compositions)

/* /// Backend-agnostic implementation of algorithms for computing the forward kinematics
/// and partial derivatives (i.e, Jacobian matrix) or the application in inverse kinematics
/// solvers. Generic in the floating point representation.
#[derive(Debug)]
pub struct DifferentiableModel<'a, 'b, F, T, R, I>
where
    F: Float,
    I: Eq + Clone + Hash + Debug,
    T: DepthFirstIterable<R, I>,
    R: Rigid<FloatType = F>,
{
    _i: PhantomData<I>,
    _r: PhantomData<R>,
    cols: usize,
    /// row index at which each effector node starts. Same length as nodes!
    offsets: Vec<usize>,
    // matrix: Vec<F>,
    // configuration: Vec<F>,
    rows: usize,
    /// For each node a bool which decides whether its effector will be used. Same length as nodes!
    selected_effectors: Vec<bool>,
    /// For each node a bool which decides whether its joint will be used. Same length as nodes!
    selected_joints: Vec<bool>,
    /// Widths
    sizes: Vec<usize>,
    tree: &'a T,
    params: &'b [<R as Rigid>::FloatType],
    transformations: Vec<R::Transformation>,
}

impl<'a, 'b, F, T, R, I> DifferentiableModel<'a, 'b, F, T, R, I>
where
    F: Float,
    I: Eq + Clone + Hash + Debug,
    T: DepthFirstIterable<R, I>,
    R: Rigid<FloatType = F>,
{
    // Implement on the result types of differential model or forward model
    // fn effectors(&self) -> Vec<&[F]> {
    //     // &mut col[*offset..*offset + effector_node.get().effector_size()],

    //     izip!(&self.selected_effectors, &self.offsets, &self.sizes)
    //         .filter_map(|(&s, &i, &n)| if s { Some(&self.configuration[i..i + n]) } else { None })
    //         .collect_vec()
    // }

    fn active(&self) -> &[bool] {
        &self.selected_joints
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

    fn compute(&self) -> Differentiated<F> {
        debug_assert_eq!(self.params.len(), self.selected_joints.len());

        // compute transformations only once
        //
        // TODO refactoring: Intermediate object with the transformations.
        let nodes_trafos = self
            .tree
            .iter()
            .accumulate(self.params, 42)
            .enumerate()
            .map(|(idx, (node, trafo))| (idx, node, trafo)) // flatten
            .collect_vec();

        // Compute the forward kinematics

        let mut effectors = vec![F::zero(); self.rows()];

        izip!(&nodes_trafos, &self.selected_effectors, &self.offsets)
            .filter_map(|(x, selected, offset)| if *selected { Some((x, offset)) } else { None })
            .for_each(|((_, node, pose), offset)| {
                node.get().effector(pose, &mut effectors, *offset);
            });

        let mut jacobian = vec![F::zero(); self.rows * self.cols];

        jacobian
            // .iter_mut()
            // FIXME: Row below can panic .. handle errors
            // Column-major!
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
                    self.tree.iter_sub(joint_node), // iterating over the child tree
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

        Differentiated { jacobian, effectors }
    }
}
    */

#[cfg(feature = "ndarray")]
#[cfg(test)]
mod tests {

    // The `ndarray` as a reference implementation is used for testing

    use super::*;
    use crate::ndarray::robot::{Axis, LinkNodeId, Segment};
    use crate::{Articulated, DepthFirstArenaTree, DirectedArenaTree, DirectionIterable};
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

        let config = tree.config(
            vec![
                &"link1".to_string(),
                &"link2".to_string(),
                &"link3".to_string(),
                &"link4".to_string(),
            ],
            vec![&"link2".to_string(), &"link4".to_string()],
            42,
        );

        let pose = tree.pose(
            &[0.0, 0.0, std::f64::consts::FRAC_PI_2, std::f64::consts::FRAC_PI_2, 0.0],
            &config,
        );

        let jacobian = pose.jacobian();

        let result = ArrayView1::<f64>::from(&jacobian)
            .into_shape_with_order(((config.rows, config.cols), Order::ColumnMajor))
            .unwrap();

        let target = array![
            [0.0, 0.0, 0.0, 0.0],
            [20.0, 10.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0,],
            [-10.0, 0.0, -10.0, 0.0],
            [0.0, 0.0, -10.0, -10.0],
            [0.0, 0.0, 0.0, 0.0,]
        ];

        assert_eq!((config.rows, config.cols), (6, 4));
        assert_abs_diff_eq!(result, target, epsilon = 1e-6);
    }
}
