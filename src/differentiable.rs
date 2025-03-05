use crate::{forward::TransformationAccumulation, DepthFirstIterable, Nodelike, Rigid};
use itertools::{izip, Itertools};
use num_traits::Float;
#[cfg(feature = "rayon")]
use rayon::prelude::*;
use std::{collections::HashSet, fmt::Debug, hash::Hash};

pub enum ComputeSelection {
    EffectorsOnly,
    JacobianOnly,
    All,
}

/// A kinematic model that can compute a configuration and its partial derivties (Jacobian matrix)
pub trait Differentiable<F: Float> {
    // Document this (blog). It's required for returning a reference to internal data
    // type Data<'a>
    // where
    //     Self: 'a; // https://github.com/rust-lang/rust/issues/87479

    /// returns a reference to the internal data type
    fn jacobian(&self) -> &[F];
    fn flat_effectors(&self) -> &[F];
    fn effectors(&self) -> Vec<&[F]>;

    /// compute the jacobian matrix
    fn setup<T, R, I>(&mut self, tree: &T, selected_joints: &HashSet<&I>, selected_effectors: &HashSet<&I>)
    where
        T: DepthFirstIterable<R, I>,
        R: Rigid<FloatType = F>,
        I: Eq + Clone + Hash + Debug;

    /// Compute is necessary as the structure holds the memory for the jacobian and the forward vector
    fn compute<T, R, I>(&mut self, tree: &T, params: &[R::FloatType], selection: ComputeSelection)
    where
        T: DepthFirstIterable<R, I>,
        R: Rigid<FloatType = F>,
        I: Eq + Clone + Hash + Debug;

    fn rows(&self) -> usize;
    fn cols(&self) -> usize;
    fn dims(&self) -> (usize, usize);
}
// Note: Won't make the trait itself generic. That would be cleaner but mean more overhead (i.e., requiring full qualifiers in compositions)

/// A kinematic model that can compute a configuration
/// And partial derivatives based on Vec
///
/// Base implementation that should be used in backend implementation as a composite.
/// The composition of the Jacobian does not require a specific backend and can directly operate on
/// a rust vector instead. All backends can operate on this structure without copying the data.
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

        izip!(&self.offsets, &self.sizes)
            .map(|(&i, &n)| &self.configuration[i..i + n])
            .collect_vec()
    }
    fn flat_effectors(&self) -> &[F] {
        &self.configuration
    }

    // TODO accept slice and create the hashset internally
    fn setup<T, R, I>(&mut self, tree: &T, selected_joints: &HashSet<&I>, selected_effectors: &HashSet<&I>)
    where
        T: DepthFirstIterable<R, I>,
        R: Rigid<FloatType = F>,
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

        self.sizes = tree.iter().map(|n| n.get().effector_size()).collect();
        dbg!(&self.offsets);
        dbg!(&self.selected_effectors);

        self.rows = tree
            .iter()
            .map(|node| {
                if selected_effectors.get(&node.id()).is_some() {
                    node.get().effector_size()
                } else {
                    0
                }
            })
            .sum();

        self.cols = self.selected_joints.iter().filter(|&selected| *selected).count();
        dbg!((self.rows, self.cols));

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

    fn dims(&self) -> (usize, usize) {
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
mod tests {}
