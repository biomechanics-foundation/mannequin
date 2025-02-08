use crate::{DepthFirst, Nodelike, Rigid, TransformationAccumulation, TreeIterable};
use itertools::{izip, Itertools};
#[cfg(feature = "rayon")]
use rayon::prelude::*;
use std::{collections::HashSet, hash::Hash};

// TODO maybe make precision generic
pub trait Differentiable {
    type Data<'a>
    where
        Self: 'a;

    /// returns a reference to the internal data type
    fn jacobian(&self) -> Self::Data<'_>;

    // TODO
    /// compute the jacobian matrix
    fn setup<T, R, I>(&mut self, tree: &T, active_joints: &HashSet<I>, active_points: &HashSet<I>)
    where
        T: TreeIterable<R, I>,
        R: Rigid,
        I: Eq + Clone + Hash;
    fn compute<T, R, I>(&mut self, tree: &T, params: &[R::Parameter])
    where
        T: TreeIterable<R, I>,
        R: Rigid,
        I: Eq + Clone + Hash; //, active_joints: &[bool], active_points: &[bool]);

    /// provide a reference to the raw data and the dimensions of its axes (so first the major axis)
    fn data(&mut self) -> &mut [f64];
    fn rows(&self) -> usize;
    fn cols(&self) -> usize;
}

/// Base implementation that should be used in backend implementation as a composite
/// Call methods in children like that: `<VecJacobian as Differentiable<T, R, I>>::compute`
#[derive(Debug, Default)]
pub struct VecJacobian {
    data: Vec<f64>,
    rows: usize,
    cols: usize,
    /// row index at which each effector note starts
    offsets: Vec<usize>,
    active_joints: Vec<bool>,
    active_points: Vec<bool>,
}

impl VecJacobian {
    pub fn new() -> Self {
        Self { ..Default::default() }
    }
}

impl Differentiable for VecJacobian {
    type Data<'a> = &'a Vec<f64>;

    fn jacobian(&self) -> Self::Data<'_> {
        &self.data
    }

    fn setup<T, R, I>(&mut self, tree: &T, active_joints: &HashSet<I>, active_points: &HashSet<I>)
    where
        T: TreeIterable<R, I>,
        R: Rigid,
        I: Eq + Clone + Hash,
    {
        self.active_joints = tree
            .iter(DepthFirst, None)
            .map(|n| active_joints.get(&n.id()).is_some())
            .collect();
        self.active_points = tree
            .iter(DepthFirst, None)
            .map(|n| active_points.get(&n.id()).is_some())
            .collect();
        self.offsets = tree
            .iter(DepthFirst, None)
            .scan(0, |offset, node| {
                let result = Some(*offset);
                if active_points.get(&node.id()).is_some() {
                    *offset += node.get().effector_size();
                }
                result
            })
            .collect();

        self.rows = self.offsets.iter().sum();
        self.cols = self.active_joints.iter().filter(|&active| *active).count();

        self.data.clear();
        self.data.resize(self.rows * self.cols, 0.0f64);
    }

    fn data(&mut self) -> &mut [f64] {
        &mut self.data
    }

    fn rows(&self) -> usize {
        self.rows
    }

    fn cols(&self) -> usize {
        self.cols
    }

    fn compute<T, R, I>(&mut self, tree: &T, params: &[<R as Rigid>::Parameter])
    where
        T: TreeIterable<R, I>,
        R: Rigid,
        I: Eq + Clone + Hash,
    {
        // compute transformations only once
        let nodes_trafos = tree
            .iter(DepthFirst, None)
            .accumulate_transformations(params, 42)
            .enumerate()
            .map(|(idx, (node, trafo))| (idx, node, trafo)) // flatten
            .collect_vec();

        // nomenclature (as in https://stats.stackexchange.com/a/588492): row-, column-, tube fibers

        self.data
            // .iter_mut()
            .chunks_mut(self.rows)
            // TODO use rayon: into_par_iter()
            .zip(
                nodes_trafos
                    .iter()
                    .zip(self.active_joints.iter()) // Add the active joint lists
                    .filter_map(|(x, active)| if *active { Some(x) } else { None }), // filter inactive joints and remove flag
                                                                                     // .par_iter()
            )
            .for_each(|(col, (idx, joint_node, joint_pose))| {
                izip!(
                    tree.iter(DepthFirst, Some(joint_node)), // iterating over the child tree
                    // zipping the corresponding trafos (by skipping until the current node) and the offsets in the column
                    // Using the index here is ok, keeping an iterator is to hard (gets mutated in a different closure)
                    nodes_trafos.iter().skip(*idx).map(|(_, _, trafo)| trafo),
                    self.offsets.iter().skip(*idx),
                    self.active_points.iter()
                )
                .filter(|(_, _, _, active)| **active)
                .for_each(|(effector_node, effector_pose, offset, _)| {
                    // The slice of the colum is itself a column-first matrix
                    effector_node.get().partial_derivative(
                        effector_pose,
                        joint_node.get(),
                        joint_pose,
                        &mut col[*offset..*offset + effector_node.get().effector_size()],
                    );
                });
            });
    }
}

#[cfg(feature = "ndarray")]
mod ndarray {
    use super::{Differentiable, VecJacobian};
    use crate::{Rigid, TreeIterable};
    use ndarray::Array2;
    use std::{collections::HashSet, hash::Hash};
}

#[cfg(feature = "nalgebra")]
mod nalgebra {
    use super::Differentiable;
    use crate::{Rigid, TreeIterable};
    use nalgebra::DMatrix;
    use std::hash::Hash;

    struct NAlgebraJacobian {
        data: DMatrix<f64>,
    }

    impl<T, R, I> Differentiable<T, R, I> for NAlgebraJacobian
    where
        T: TreeIterable<R, I>,
        R: Rigid,
        I: Eq + Clone + Hash,
    {
        // Note: could ArrayView too
        type Data = DMatrix<f64>;

        fn jacobian(&self) -> &Self::Data {
            &self.data
        }

        fn data(&mut self) -> (&mut [f64], usize, usize) {
            let (m, n) = (self.data.ncols(), self.data.nrows());
            (self.data.as_mut_slice(), m, n)
        }
    }
}
