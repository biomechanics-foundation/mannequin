//! Interface and basic implementor for the forward kinematic model.
//!
//! Contains additional useful extension to the iterators over a tree
//! that can be shared by implementers of the trait.

use std::marker::PhantomData;

use num_traits::Float;

use crate::{differentiable::ComputeSelection, DepthFirstIterable, Differentiable, NodeLike, Rigid};

/// Trait representing a stateful forward kinematics algorithm. It allows selecting the effectors to be
/// computed and thus a specific (or multiple) kinematic chain(s).
pub trait Forward<IT, RB>
where
    IT: DepthFirstIterable<RB, RB::NodeId>,
    RB: Rigid,
{
    fn setup(&mut self, tree: &IT, selected_effectors: &[&<RB as Rigid>::NodeId]);
    fn solve(&mut self, tree: &IT, params: &[RB::FloatType]) -> Vec<&[RB::FloatType]>;
}

/// Default forward kinematics that is only a thin wrapper around an [Differentiable] instance.
pub struct ForwardModel<F, D>
where
    F: Float,
    D: Differentiable<F>,
{
    differential_model: D,
    p: PhantomData<F>,
}

impl<F, D> ForwardModel<F, D>
where
    F: Float,
    D: Differentiable<F>,
{
    pub fn new(differential_model: D) -> Self {
        Self {
            differential_model,
            p: PhantomData,
        }
    }
}

impl<IT, RB, F, D> Forward<IT, RB> for ForwardModel<F, D>
where
    IT: DepthFirstIterable<RB, RB::NodeId>,
    RB: Rigid<FloatType = F>,
    F: Float,
    D: Differentiable<F>,
{
    fn solve(&mut self, tree: &IT, params: &[<RB as Rigid>::FloatType]) -> Vec<&[F]> {
        self.differential_model
            .compute(tree, params, ComputeSelection::EffectorsOnly);
        self.differential_model.effectors()
    }

    fn setup(&mut self, tree: &IT, selected_effectors: &[&<RB as Rigid>::NodeId]) {
        self.differential_model.setup(tree, &[], selected_effectors);
    }
}

/// Trait that adds an `accumulate` functions for accumulating transformations from direct path from a root to a node.
/// Implemented for an iterator over nodes but should only be used on a depth-first iteration (not enforced!)
pub trait TransformationAccumulation<'a, Node, Load, NodeRef>
where
    Load: Rigid,
    Node: NodeLike<Load, NodeRef> + 'a,
{
    fn accumulate(
        self,
        params: &[Load::FloatType],
        max_depth: usize,
    ) -> impl Iterator<Item = (&'a Node, Load::Transformation)>;
}

impl<'a, Node, Load, NodeRef, T> TransformationAccumulation<'a, Node, Load, NodeRef> for T
where
    Node: NodeLike<Load, NodeRef> + 'a,
    Load: Rigid,
    T: Iterator<Item = &'a Node>,
{
    fn accumulate(
        self,
        params: &[Load::FloatType],
        max_depth: usize,
    ) -> impl Iterator<Item = (&'a Node, <Load as Rigid>::Transformation)> {
        self.into_iter().enumerate().scan(
            Vec::<Load::Transformation>::with_capacity(max_depth),
            |stack, (index, node)| {
                while node.depth() < stack.len() {
                    stack.pop();
                }
                let current = Load::concat(
                    stack.last().unwrap_or(&Load::neutral_element()),
                    &node.get().transform(params, index),
                );
                stack.push(current.clone());
                Some((node, current))
            },
        )
    }
}

#[cfg(test)]
mod tests {

    // The `ndarray` as a reference implementation is used for testing

    use super::*;
    use crate::ndarray::robot::{Axis, Segment};
    use crate::{DepthFirstArenaTree, DifferentiableModel, DirectedArenaTree, DirectionIterable};
    use itertools::Itertools;
    use ndarray::prelude::*;

    #[test]
    fn test_fk() {
        let mut tree = DirectedArenaTree::new();
        let mut fk = ForwardModel::new(DifferentiableModel::new());

        let mut trafo = Segment::neutral_element();
        trafo.slice_mut(s![..3, 3]).assign(&array![10.0, 0.0, 0.0]);

        let link1 = Segment::new(&trafo, Axis::RotationZ, None);
        let link2 = Segment::new(&trafo, Axis::RotationZ, Some(Segment::neutral_element()));
        let link3 = Segment::new(&trafo, Axis::RotationZ, Some(Segment::neutral_element()));
        let link4 = Segment::new(&trafo, Axis::RotationZ, Some(Segment::neutral_element()));

        // TODO .. can we make the refs fix in a way they don't get optimized away?
        // Then these could be strings even!
        let ref1 = tree.set_root(link1, "link1".to_string());
        let ref2 = tree.add(link2, "link2".to_string(), &ref1).unwrap();
        let ref3 = tree.add(link3, "link3".to_string(), &ref1).unwrap();
        let ref4 = tree.add(link4, "link4".to_string(), &ref3).unwrap();

        let tree: DepthFirstArenaTree<_, _> = tree.into();

        fk.setup(&tree, &[&ref2, &ref3, &ref4]);
        let res = fk.solve(&tree, &[0.0, 0.0, std::f64::consts::FRAC_PI_2, 0.0]);
        let res = res.iter().map(|&el| el.to_owned()).collect_vec();

        assert_eq!(
            res,
            vec![vec![20.0, 0.0, 0.0], vec![20.0, 0.0, 0.0], vec![20.0, 10.0, 0.0]]
        );
    }
}
