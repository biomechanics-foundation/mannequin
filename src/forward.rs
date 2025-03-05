/*! Defines the payload carried by [Nodelike] in the context of kinematics/character animation */
// FIXME
#![allow(unused_variables, dead_code)]

use std::{collections::HashSet, marker::PhantomData};

use num_traits::Float;

use crate::{differentiable::ComputeSelection, DepthFirstIterable, Differentiable, Nodelike, Rigid};

/// Trait representing a stateful forward kinematics algorithm. For instance, it can represent a rigid body mannequin
/// (e.g., a robot) or softbody/skinning for character animation.
pub trait Forward<IT, RB>
where
    IT: DepthFirstIterable<RB, RB::NodeId>,
    RB: Rigid,
{
    fn initialize(
        &mut self,
        tree: &IT,
        selected_joints: &[&<RB as Rigid>::NodeId],
        selected_effectors: &[&<RB as Rigid>::NodeId],
    );
    fn solve(&mut self, tree: &IT, params: &[RB::FloatType]) -> Vec<&[RB::FloatType]>;
}

/// Default forward kinematics that wraps a [Differentiable] kinematics model.
pub struct ForwardModel<F, D>
where
    F: Float,
    D: Differentiable<F>,
{
    _max_depth: usize,
    differential_model: D,
    p: PhantomData<F>,
}

impl<F, D> ForwardModel<F, D>
where
    F: Float,
    D: Differentiable<F>,
{
    pub fn new(_max_depth: usize, model: D) -> Self {
        Self {
            _max_depth,
            differential_model: model,
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
}

/// Trait that adds an `accumulate` functions for accumulating transformations from direct path from a root to a node.
/// Implemented for an iterator over nodes but should only be used on a depth-first iteration (not enforced!)
pub trait TransformationAccumulation<'a, Node, Load, NodeRef>
where
    Load: Rigid,
    Node: Nodelike<Load, NodeRef> + 'a,
{
    fn accumulate(
        self,
        params: &[Load::FloatType],
        max_depth: usize,
    ) -> impl Iterator<Item = (&'a Node, Load::Transformation)>;
}

impl<'a, Node, Load, NodeRef, T> TransformationAccumulation<'a, Node, Load, NodeRef> for T
where
    Node: Nodelike<Load, NodeRef> + 'a,
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
mod tests {}
