//! Interface and basic implementer for the forward kinematic model.
//!
//! Contains additional useful extension to the iterators over a tree
//! that can be shared by implementers of the trait.

use itertools::{izip, Itertools};

use crate::{DepthFirstIterable, NodeLike, Rigid};
use std::{collections::HashSet, fmt::Debug, hash::Hash, marker::PhantomData};

/// Trait representing a stateful forward kinematics algorithm. It allows selecting the effectors to be
/// computed and thus a specific (or multiple) kinematic chain(s).
/// Implemented for DepthFirstIterable; make a newtype to implement an ANN based IK for instance.
pub trait Articulated<NodeType, LoadType, TreeType, IdType>:
    DepthFirstIterable<LoadType, IdType, Node = NodeType>
where
    LoadType: Rigid,
    NodeType: NodeLike<LoadType, IdType>,
    IdType: Eq + Clone + Hash + Debug,
{
    // Comments (here, in differential and inverse):
    // - we want to avoid copying the config but rather store it as a reference in model
    // - we could add another lifetime(s) to the trait, but semantically it
    //   fits better to lifetimes the `pose` method (probably not the only advantage)
    // - we need to use GAT with lifetime(s)
    // - we need to make sure that 'x is the object's lifetime: https://github.com/rust-lang/rust/issues/87479
    type Model<'x, 'y>: Forward<LoadType::FloatType>
    where
        Self: 'x;
    type Config;

    fn accumulate(
        &self,
        params: &[LoadType::FloatType],
        max_depth: usize,
    ) -> impl Iterator<Item = LoadType::Transformation>;

    fn pose<'a, 'b>(&'a self, params: &[LoadType::FloatType], config: &'b Self::Config) -> Self::Model<'a, 'b>;

    fn config(
        &self,
        selected_joints: Vec<&IdType>,
        selected_effectors: Vec<&IdType>,
        max_depth: usize,
    ) -> ForwardConfig;
    // TODO second forward that copies from old Self::Model
}

impl<NodeType, LoadType, IdType, TreeType, FloatType> Articulated<NodeType, LoadType, TreeType, IdType> for TreeType
where
    NodeType: NodeLike<LoadType, IdType>,
    LoadType: Rigid<FloatType = FloatType>,
    IdType: Eq + Clone + Hash + Debug,
    TreeType: DepthFirstIterable<LoadType, IdType, Node = NodeType>,
{
    type Model<'x, 'y>
        = ForwardModel<'x, 'y, NodeType, LoadType, TreeType, IdType>
    where
        Self: 'x;
    type Config = ForwardConfig;
    fn accumulate(
        &self,
        params: &[LoadType::FloatType],
        max_depth: usize,
    ) -> impl Iterator<Item = <LoadType as Rigid>::Transformation> {
        self.iter().enumerate().scan(
            Vec::<LoadType::Transformation>::with_capacity(max_depth),
            |stack, (index, node)| {
                while node.depth() < stack.len() {
                    stack.pop();
                }
                let current = LoadType::concat(
                    stack.last().unwrap_or(&LoadType::neutral_element()),
                    &node.get().transform(params, index),
                );
                stack.push(current.clone());
                Some(current)
            },
        )
    }

    /// Computes local coordinate frames.
    fn pose<'a, 'b>(&'a self, params: &[LoadType::FloatType], config: &'b Self::Config) -> Self::Model<'a, 'b> {
        // ForwardModel<'a, 'b, NodeType, LoadType, TreeType, IdType> {
        let transformations = self.accumulate(params, config.max_depth).collect_vec();

        // let sizes = self.iter().map(|n| n.get().effector_size()).collect();
        ForwardModel {
            transformations,
            tree: self,
            _nodeid: PhantomData,
            config,
        }
    }

    /// Computes values that typically don't change often compared to the `params` in [forward()](Forward::forward).
    fn config(&self, selected_joints: Vec<&IdType>, selected_effectors: Vec<&IdType>, max_depth: usize) -> ForwardConfig
    where
        TreeType: DepthFirstIterable<LoadType, IdType>,
        IdType: Eq + Clone + Hash + Debug,
    {
        let selected_joints = if selected_joints.is_empty() {
            vec![true; self.len()]
        } else {
            let selected_joints: HashSet<&IdType> = HashSet::from_iter(selected_joints.iter().copied());

            self.iter().map(|n| selected_joints.contains(&n.id())).collect()
        };
        let selected_effectors_map: HashSet<&IdType> = HashSet::from_iter(selected_effectors.iter().copied());
        let selected_effectors = self.iter().map(|n| selected_effectors_map.contains(&n.id())).collect();
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
        let cols = selected_joints.iter().filter(|&selected| *selected).count();

        let sizes = self.iter().map(|n| n.get().effector_size()).collect();
        ForwardConfig {
            max_depth,
            selected_joints,
            selected_effectors,
            rows,
            cols,
            offsets,
            sizes,
        }
    }
}

#[derive(Default)]
pub struct ForwardConfig {
    pub max_depth: usize,
    pub selected_joints: Vec<bool>,
    pub selected_effectors: Vec<bool>,
    pub offsets: Vec<usize>,
    pub sizes: Vec<usize>,
    pub rows: usize,
    pub cols: usize, // Selection of active joints and effectors
}

/// Default forward kinematics that is only a thin wrapper around an [Differentiable] instance.
/// Holds references to a configuration and the tree.
pub struct ForwardModel<'a, 'b, NodeType, LoadType, TreeType, IdType>
where
    LoadType: Rigid,
    NodeType: NodeLike<LoadType, IdType>,
    TreeType: 'a + DepthFirstIterable<LoadType, IdType, Node = NodeType>,
    IdType: Eq + Clone + Hash + Debug,
{
    _nodeid: PhantomData<IdType>,
    pub transformations: Vec<LoadType::Transformation>,
    // pub transformations: Vec<(&'a NodeType, LoadType::Transformation)>,
    pub tree: &'a TreeType,
    pub config: &'b ForwardConfig,
}

pub trait Forward<F> {
    fn effectors(&self) -> Vec<&[F]>;
    fn flat_effectors(&self) -> Vec<F>;
}

impl<'a, 'b, NodeType, LoadType, TreeType, IdType, FloatType> Forward<FloatType>
    for ForwardModel<'a, 'b, NodeType, LoadType, TreeType, IdType>
where
    LoadType: Rigid<FloatType = FloatType>,
    NodeType: NodeLike<LoadType, IdType>,
    TreeType: DepthFirstIterable<LoadType, IdType, Node = NodeType>,
    IdType: Eq + Clone + Hash + Debug,
{
    fn effectors(&self) -> Vec<&[LoadType::FloatType]> {
        //     // &mut col[*offset..*offset + effector_node.get().effector_size()],

        //     izip!(&self.selected_effectors, &self.offsets, &self.sizes)
        //         .filter_map(|(&s, &i, &n)| if s { Some(&self.configuration[i..i + n]) } else { None })
        //         .collect_vec()
        todo!()
    }
    fn flat_effectors(&self) -> Vec<LoadType::FloatType> {
        // let mut effectors = vec![LoadType::FloatType::zero(); self.rows()];

        // izip!(
        //     &self.tree,
        //     &self.transformations,
        //     &self.selected_effectors,
        //     &self.offsets
        // )
        // .filter_map(|(node, pose, selected, offset)| if *selected { Some((node, pose, offset)) } else { None })
        // .for_each(|(node, pose, offset)| {
        //     node.get().effector(pose, &mut effectors, *offset);
        // });
        // effectors
        todo!()
    }
}

// impl<F, D> ForwardModel<F, D>
// where
//     F: Float,
//     D: Differentiable<F>,
// {
//     pub fn new(differential_model: D) -> Self {
//         Self {
//             differential_model,
//             p: PhantomData,
//         }
//     }
// }

// impl<IT, RB, F, D> Forward<IT, RB> for ForwardModel<F, D>
// where
//     IT: DepthFirstIterable<RB, RB::NodeId>,
//     RB: Rigid<FloatType = F>,
//     F: Float,
//     D: Differentiable<F>,
// {
//     fn accumulate(
//         self,
//         params: &[LoadType::FloatType],
//         max_depth: usize,
//     ) -> impl Iterator<Item = (&'a Node, <Load as Rigid>::Transformation)> {
//         self.into_iter().enumerate().scan(
//             Vec::<Load::Transformation>::with_capacity(max_depth),
//             |stack, (index, node)| {
//                 while node.depth() < stack.len() {
//                     stack.pop();
//                 }
//                 let current = Load::concat(
//                     stack.last().unwrap_or(&Load::neutral_element()),
//                     &node.get().transform(params, index),
//                 );
//                 stack.push(current.clone());
//                 Some((node, current))
//             },
//         )
//     }

//     fn forward() -> ForwardModel {
//         todo!()
//     }

//     // fn solve(&mut self, tree: &IT, params: &[<RB as Rigid>::FloatType]) -> Vec<&[F]> {
//     //     self.differential_model
//     //         .compute(tree, params, ComputeSelection::EffectorsOnly);
//     //     self.differential_model.effectors()
//     // }

//     // fn setup(&mut self, tree: &IT, selected_effectors: &[&<RB as Rigid>::NodeId]) {
//     //     self.differential_model.setup(tree, &[], selected_effectors);
//     // }
// }

// /// Trait that adds an `accumulate` functions for accumulating transformations from direct path from a root to a node.
// /// Implemented for an iterator over nodes but should only be used on a depth-first iteration (not enforced!)
// pub trait TransformationAccumulation<'a, Node, Load, NodeRef>
// where
//     Load: Rigid,
//     Node: NodeLike<Load, NodeRef> + 'a,
// {
//     // fn accumulate(
//     //     self,
//     //     params: &[Load::FloatType],
//     //     max_depth: usize,
//     // ) -> impl Iterator<Item = (&'a Node, Load::Transformation)>;
// }

// impl<'a, Node, Load, NodeRef, T> TransformationAccumulation<'a, Node, Load, NodeRef> for T
// where
//     Node: NodeLike<Load, NodeRef> + 'a,
//     Load: Rigid,
//     T: Iterator<Item = &'a Node>,
// {
//     // fn accumulate(
//     //     self,
//     //     params: &[Load::FloatType],
//     //     max_depth: usize,
//     // ) -> impl Iterator<Item = (&'a Node, <Load as Rigid>::Transformation)> {
//     //     self.into_iter().enumerate().scan(
//     //         Vec::<Load::Transformation>::with_capacity(max_depth),
//     //         |stack, (index, node)| {
//     //             while node.depth() < stack.len() {
//     //                 stack.pop();
//     //             }
//     //             let current = Load::concat(
//     //                 stack.last().unwrap_or(&Load::neutral_element()),
//     //                 &node.get().transform(params, index),
//     //             );
//     //             stack.push(current.clone());
//     //             Some((node, current))
//     //         },
//     //     )
//     // }
// }

#[cfg(feature = "ndarray")]
#[cfg(test)]
mod tests {

    // The `ndarray` as a reference implementation is used for testing

    use super::*;
    use crate::ndarray::robot::{Axis, Segment};
    use crate::{DepthFirstArenaTree, DirectedArenaTree, DirectionIterable};
    use itertools::Itertools;
    use ndarray::prelude::*;

    #[test]
    fn test_fk() {
        let mut tree = DirectedArenaTree::new();

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

        let selected_effectors = { todo!() };
        let config = tree.config(vec![&ref2, &ref3, &ref4], selected_effectors, 32);

        let pose = tree.pose(&[0.0, 0.0, std::f64::consts::FRAC_PI_2, 0.0], &config);

        let res = pose.effectors();
        let res = res.iter().map(|&el| el.to_owned()).collect_vec();

        assert_eq!(
            res,
            vec![vec![20.0, 0.0, 0.0], vec![20.0, 0.0, 0.0], vec![20.0, 10.0, 0.0]]
        );
    }
}
