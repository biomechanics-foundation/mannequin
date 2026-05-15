//! Algorithms useful for implementing differential kinematics (such as forward kinematics
//! and partial derivatives (i.e., the Jacobian matrix).
//!
//! The algorithms are independent of
//! the numerical backend and support [f32] and [f64] floating point representations.

use crate::{DepthFirstIterable, ForwardModel, NodeLike, Rigid};
use itertools::izip;
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
                    // zipping the corresponding transformations (by skipping until the current node) and the offsets in the column
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
