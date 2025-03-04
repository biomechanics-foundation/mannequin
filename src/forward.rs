/*! Defines the payload carried by [Nodelike] in the context of kinematics/character animation */

use std::collections::HashSet;

use itertools::Itertools;

use crate::{differentiable, mannequin::Rigid, DepthFirstIterable, Differentiable, Nodelike};

/// Trait representing a stateful forward kinematics algoritm. For instance, it can represent a rigid body mannequin
/// (e.g., a robot) or softbody/skinning for character animation.
pub trait Forward<IT, RB>
where
    IT: DepthFirstIterable<RB, RB::NodeId>,
    RB: Rigid,
{
    fn initialize(&mut self, tree: &IT, target_refs: &[RB::NodeId]);
    fn solve(&mut self, tree: &IT, params: &[RB::FloatType]) -> Vec<RB::Transformation>;
}

/// Default forward kinematics that wraps a [Differentiable] kinematics model.
pub struct ForwardsKinematics<RB>
where
    RB: Rigid,
{
    max_depth: usize,
    target_refs: Vec<<RB as Rigid>::NodeId>,
}

impl<RB> ForwardsKinematics<RB>
where
    RB: Rigid,
{
    pub fn new(max_depth: usize) -> Self {
        Self {
            max_depth,
            target_refs: vec![],
        }
    }
}

impl<IT, RB> Forward<IT, RB> for ForwardsKinematics<RB>
where
    IT: DepthFirstIterable<RB, RB::NodeId>,
    RB: Rigid,
{
    fn solve(&mut self, tree: &IT, params: &[<RB as Rigid>::FloatType]) -> Vec<<RB as Rigid>::Transformation> {
        todo!()
        // not the same retun type as DifferentiableModel
    }

    fn initialize(&mut self, tree: &IT, selected_effectors: &[<RB as Rigid>::NodeId]) {
        todo!()
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
