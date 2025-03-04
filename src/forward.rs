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
    fn solve(&mut self, tree: &IT, params: &[RB::Parameter]) -> Vec<RB::Transformation>;
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
    fn solve(&mut self, tree: &IT, params: &[<RB as Rigid>::Parameter]) -> Vec<<RB as Rigid>::Transformation> {
        todo!()
        // not the same retun type as DifferentiableModel
    }

    fn initialize(&mut self, tree: &IT, selected_effectors: &[<RB as Rigid>::NodeId]) {
        todo!()
    }
}

/// Helper function to be used in [std::iter::Scan] for accumulating transformations from direct path from a root to a node
pub fn accumulate<'a, Node, Load, NodeRef>(
    stack: &mut Vec<Load::Transformation>,
    arg: (&'a Node, &Load::Parameter),
) -> Option<(&'a Node, Load::Transformation)>
where
    Node: Nodelike<Load, NodeRef>,
    Load: Rigid,
{
    let (node, param) = arg;
    while node.depth() < stack.len() {
        stack.pop();
    }
    let current = Load::concat(
        stack.last().unwrap_or(&Load::neutral_element()),
        &node.get().transform(param),
    );
    stack.push(current.clone());
    Some((node, current))
}

// TODO: Move to forward.rs module

///  Trait that adds an `accumulate` functions for accumulating transformations from direct path from a root to a node.
/// Wraps a zip and scan operation
///
pub trait TransformationAccumulation<'a, Node, Load, NodeRef>
where
    Load: Rigid,
    Node: Nodelike<Load, NodeRef> + 'a,
{
    fn accumulate_transformations(
        self,
        param: &[Load::Parameter],
        max_depth: usize,
    ) -> impl Iterator<Item = (&'a Node, Load::Transformation)>;
}

impl<'a, Node, Load, NodeRef, T> TransformationAccumulation<'a, Node, Load, NodeRef> for T
where
    Node: Nodelike<Load, NodeRef> + 'a,
    Load: Rigid,
    T: Iterator<Item = &'a Node>,
{
    fn accumulate_transformations(
        self,
        param: &[Load::Parameter],
        max_depth: usize,
    ) -> impl Iterator<Item = (&'a Node, <Load as Rigid>::Transformation)> {
        self.into_iter()
            .zip(param.iter())
            .scan(Vec::<Load::Transformation>::with_capacity(max_depth), accumulate)
    }
}
#[cfg(test)]
mod tests {}
