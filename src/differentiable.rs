use crate::{DepthFirstIterable, Nodelike, Rigid, TransformationAccumulation};
use itertools::{izip, Itertools};
#[cfg(feature = "rayon")]
use rayon::prelude::*;
use std::{collections::HashSet, fmt::Debug, hash::Hash};

pub trait Differentiable {
    type Data<'a>
    where
        Self: 'a; // https://github.com/rust-lang/rust/issues/87479

    /// returns a reference to the internal data type
    fn jacobian(&self) -> Self::Data<'_>;

    /// compute the jacobian matrix
    fn setup<T, R, I>(&mut self, tree: &T, selected_joints: &HashSet<I>, selected_effectors: &HashSet<I>)
    where
        T: DepthFirstIterable<R, I>,
        R: Rigid,
        I: Eq + Clone + Hash + Debug;
    fn compute<T, R, I>(&mut self, tree: &T, params: &[R::Parameter])
    where
        T: DepthFirstIterable<R, I>,
        R: Rigid,
        I: Eq + Clone + Hash + Debug;

    /// provide a reference to the raw data and the dimensions of its axes (so first the major axis)
    fn data(&mut self) -> &mut [f64];
    fn rows(&self) -> usize;
    fn cols(&self) -> usize;
}

/// Base implementation that should be used in backend implementation as a composite.
/// The composition of the Jacobian does not require a specific backend and can directly operate on
/// a rust vector instead. All backends can operate on this structure without copying the data.
#[derive(Debug, Default)]
pub struct VecJacobian {
    data: Vec<f64>,
    rows: usize,
    cols: usize,
    /// row index at which each effector note starts
    offsets: Vec<usize>,
    selected_joints: Vec<bool>,
    selected_effectors: Vec<bool>,
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

    fn setup<T, R, I>(&mut self, tree: &T, selected_joints: &HashSet<I>, selected_effectors: &HashSet<I>)
    where
        T: DepthFirstIterable<R, I>,
        R: Rigid,
        I: Eq + Clone + Hash + Debug,
    {
        self.selected_joints = tree.iter().map(|n| selected_joints.get(&n.id()).is_some()).collect();
        self.selected_effectors = tree.iter().map(|n| selected_effectors.get(&n.id()).is_some()).collect();
        self.offsets = tree
            .iter()
            .scan(0, |offset, node| {
                let result = Some(*offset);
                if selected_effectors.get(&node.id()).is_some() {
                    *offset += node.get().effector_size();
                }
                result
            })
            .collect();

        self.rows = self.offsets.iter().sum();
        self.cols = self.selected_joints.iter().filter(|&selected| *selected).count();

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
        T: DepthFirstIterable<R, I>,
        R: Rigid,
        I: Eq + Clone + Hash + Debug,
    {
        // compute transformations only once
        let nodes_trafos = tree
            .iter()
            .accumulate_transformations(params, 42)
            .enumerate()
            .map(|(idx, (node, trafo))| (idx, node, trafo)) // flatten
            .collect_vec();

        // nomenclature (as in https://stats.stackexchange.com/a/588492): row-, column-, tube fibers

        self.data
            // .iter_mut()
            .chunks_mut(self.rows)
            // TODO use rayon
            .zip(
                nodes_trafos
                    .iter()
                    .zip(self.selected_joints.iter()) // Add the selected joint lists
                    .filter_map(|(x, selected)| if *selected { Some(x) } else { None }), // filter inactive joints and remove flag
                                                                                         // .par_iter()
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
