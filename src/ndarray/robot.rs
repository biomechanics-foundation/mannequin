//! Implementation of traits as specializations for ArenaTree using ndarray
#![allow(unused_variables)]

use crate::{
    accumulate, ArenaTree, DepthFirst, Forward, Inverse, Mannequin, Nodelike, Rigid, TransformationAccumulation,
    TreeIterable,
};
use core::fmt;
use itertools::Itertools;
use ndarray::parallel::prelude::*;
use ndarray::prelude::*;
use ndarray::{Array1, Array2};
use smallvec::SmallVec;

#[derive(Debug, PartialEq)]

pub enum Axes {
    RotationX,
    RotationY,
    RotationZ,
    Rotation(Array1<f64>),
    TranslationX,
    TranslationY,
    TranslationZ,
    Translation(Array1<f64>),
}

#[derive(Debug, Default, PartialEq)]
pub struct Bone {
    link: Array2<f64>,
    // TODO: I assume that having the axis at the same memory location is faster
    axes: SmallVec<[Axes; 3]>,
}

impl Bone {
    pub fn new(from_parent: &Array2<f64>) -> Self {
        Self {
            link: from_parent.clone(),
            ..Default::default()
        }
    }
}

impl fmt::Display for Bone {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "Just a Bone")
        // TODO add axes type
    }
}

impl Rigid for Bone {
    type Transformation = Array2<f64>;

    type Point = Array1<f64>;

    type Parameter = f64;

    type NodeId = String;

    fn transform(&self, param: &Self::Parameter) -> Self::Transformation {
        array![
            [param.cos(), -1.0 * param.sin(), 0.0, 0.0],
            [param.sin(), param.cos(), 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0]
        ]
    }

    fn globalize(&self, other: &Self::Point) -> Self::Point {
        todo!()
    }

    fn localize(&self, other: &Self::Point) -> Self::Point {
        todo!()
    }

    fn n_dof(&self) -> usize {
        todo!()
    }

    fn neutral_element() -> Self::Transformation {
        Array2::<f64>::eye(4)
    }

    fn concat(first: &Self::Transformation, second: &Self::Transformation) -> Self::Transformation {
        first.dot(second)
    }

    fn invert(trafo: &Self::Transformation) -> Self::Transformation {
        let mut result = Self::neutral_element();
        let rot = trafo.slice(s![..3, ..3]);
        result.slice_mut(s![..3, ..3]).assign(&rot.t());
        let ipos = &trafo.slice(s![..3, 3]) * -1.0;
        result.slice_mut(s![..3, 3]).assign(&ipos);
        result
    }
}

type LinkNodeId = <Bone as Rigid>::NodeId;

/// Specialization of a Forward Kinematics on an [ArenaTree]
pub struct ForwardsKinematics {
    max_depth: usize,
}
impl Forward<ArenaTree<Bone, LinkNodeId>, Bone> for ForwardsKinematics {
    type Parameter = Array1<f64>;

    type Transformation = Array2<f64>;

    fn solve(
        &mut self,
        tree: &ArenaTree<Bone, LinkNodeId>,
        params: Self::Parameter,
        target_refs: &[LinkNodeId],
    ) -> Vec<Self::Transformation> {
        tree.iter(DepthFirst, None)
            .accumulate_transformations(params.as_slice().unwrap(), self.max_depth)
            .filter_map(|(node, trafo)| {
                if target_refs.is_empty() || target_refs.contains(&node.id()) {
                    Some(trafo)
                } else {
                    None
                }
            })
            .collect_vec()
    }
}

pub struct DifferentialIK {
    max_depth: usize,
}

pub type DifferentialIKParameter =
    <DifferentialIK as Inverse<ArenaTree<Bone, LinkNodeId>, Bone, ForwardsKinematics>>::Parameter;
pub type DifferentialIKArray =
    <DifferentialIK as Inverse<ArenaTree<Bone, LinkNodeId>, Bone, ForwardsKinematics>>::Array;

impl DifferentialIK {
    pub fn jacobian(&self, tree: &ArenaTree<Bone, LinkNodeId>, params: DifferentialIKParameter) -> DifferentialIKArray {
        // jacobian
        //     .axis_iter_mut(Axis(0))
        //     .into_par_iter()
        //     .zip(tree.iter(DepthFirst, &[]).accumulate(param, max).into_par_iter())
        //     .for_each(|(row, (node, trafo))| {
        //         println!("{row}, {node}, {trafo}");

        //         // tree
        //         //     .iter(DepthFirst, &[tree.node_ref(node)])
        //         //     .accumulate(param, max)
        //         //     .zip(row.iter_chunks(3))
        //         //     .for_each(|(cell, (node, transformation)| {
        //         //         // compute displacement
        //         //     });
        //     });

        // Added annotation for IDE
        let nodes_trafos = tree
            .iter(DepthFirst, None)
            .accumulate_transformations(params.as_slice().unwrap(), self.max_depth)
            .enumerate()
            .collect_vec();

        let n_dof = nodes_trafos.len(); // number of degrees of freedom
        let n_tcp = nodes_trafos.len(); // number of tool center points

        // Trick we reduce a 3D (n x m x o) array to a two dimensional one (x*m,o) after iteration
        let mut jacobian = Array3::<f64>::zeros((n_dof, 3, n_tcp));

        // Warning filtering could be interesting! Cannot do it after `into_par_iter`

        jacobian
            .axis_iter_mut(Axis(0))
            // .into_par_iter()
            .zip(
                nodes_trafos.iter(), // .par_iter()
            )
            .for_each(|(mut dof, (idx, (node, trafo)))| {
                // dof: (3,n_tcp)
                println!("row: {dof}, idx: {idx}, depth: {}", node.depth());
                let x = nodes_trafos
                    .iter()
                    .skip(*idx + 1)
                    .map(|(idx, (node, _))| (idx, node.depth()))
                    .collect_vec();
                println!("{x:?}");
                nodes_trafos
                    .iter()
                    .skip(*idx + 1)
                    .take_while(|(_, (child, _))| child.depth() > node.depth())
                    .for_each(|(idx, (child, child_trafo))| {
                        let local = Bone::concat(&Bone::invert(trafo), child_trafo);
                        let pos = local.slice(s![..3, 3]);

                        dof.slice_mut(s![..3, *idx]).assign(&pos);

                        println!("column:{}, {}, {dof}", idx + 1, child.depth())
                    })
            });
        // is this slower?
        jacobian.into_shape_with_order((n_dof * 3, n_tcp)).unwrap()
    }
}

impl Inverse<ArenaTree<Bone, LinkNodeId>, Bone, ForwardsKinematics> for DifferentialIK {
    type Parameter = Array1<f64>;

    type Array = Array2<f64>;

    fn solve(
        &mut self,
        tree: &ArenaTree<Bone, LinkNodeId>,
        fk: &ForwardsKinematics,
        param: Self::Parameter,
        target_refs: &[LinkNodeId],
        target_val: &[Self::Array],
    ) -> Self::Parameter {
        todo!()
    }
}

pub type Robot = Mannequin<ArenaTree<Bone, LinkNodeId>, Bone, ForwardsKinematics, DifferentialIK>;

#[cfg(test)]
mod tests {
    use std::collections::HashSet;

    use itertools::{izip, Itertools};
    use ndarray::prelude::*;

    use crate::{
        ndarray::robot::LinkNodeId, ArenaTree, Forward, Nodelike, Order::DepthFirst, Rigid, TransformationAccumulation,
        TreeIterable,
    };

    use super::{Bone, DifferentialIK, ForwardsKinematics};

    #[test]
    fn test_fk() {
        let mut fk = ForwardsKinematics { max_depth: 10 };
        let mut tree = ArenaTree::<Bone, LinkNodeId>::new();

        let mut trafo = Bone::neutral_element();
        trafo.slice_mut(s![..3, 3]).assign(&array![10.0, 0.0, 0.0]);

        let link1 = Bone::new(&trafo);
        let link2 = Bone::new(&trafo);
        let link3 = Bone::new(&trafo);
        let link4 = Bone::new(&trafo);

        // TODO .. can we make the refs fix in a way they don't get optimized away?
        // Then these could be strings even!
        let ref1 = tree.set_root(link1, "link1".to_string());
        let ref2 = tree.add(link2, "link2".to_string(), &ref1).unwrap();
        let ref3 = tree.add(link3, "link3".to_string(), &ref1).unwrap();
        let ref4 = tree.add(link4, "link4".to_string(), &ref3).unwrap();

        tree.optimize(DepthFirst);

        let res = fk.solve(
            &tree,
            array![0.0, 0.0, std::f64::consts::FRAC_PI_2, 0.0],
            &[ref2, ref3, ref4],
        );
        let res = res.iter().map(|el| el.slice(s![..3, 3]).to_vec()).collect_vec();
        println!("{:?}", res);
        assert_eq!(
            res,
            vec![vec![20.0, 0.0, 0.0], vec![20.0, 0.0, 0.0], vec![20.0, 10.0, 0.0]]
        );
    }

    #[test]
    fn test_jacobian() {
        let mut ik = DifferentialIK { max_depth: 10 };
        let mut tree = ArenaTree::<Bone, LinkNodeId>::new();

        let mut trafo = Bone::neutral_element();
        trafo.slice_mut(s![..3, 3]).assign(&array![10.0, 0.0, 0.0]);

        let link1 = Bone::new(&trafo);
        let link2 = Bone::new(&trafo);
        let link3 = Bone::new(&trafo);
        let link4 = Bone::new(&trafo);

        // TODO .. can we make the refs fix in a way they don't get optimized away?
        // Then these could be strings even!

        let ref1 = tree.set_root(link1, "link1".to_string());
        let ref2 = tree.add(link2, "link2".to_string(), &ref1).unwrap();
        let ref3 = tree.add(link3, "link3".to_string(), &ref1).unwrap();
        let ref4 = tree.add(link4, "link4".to_string(), &ref3).unwrap();

        tree.optimize(DepthFirst);

        let jacobian = ik.jacobian(&tree, array![0.0, 0.0, std::f64::consts::FRAC_PI_2, 0.0]);

        println!("jacobian: {jacobian:?}");
    }

    #[test]
    fn construct_jacobian_with_node_filter() {
        fn ik<T>(tree: &T, params: &Array1<f64>, selected_joints: &HashSet<String>, selected_points: &HashSet<String>)
        where
            T: TreeIterable<Bone, LinkNodeId>,
        {
            let active_joints = tree
                .iter(DepthFirst, None)
                .map(|n| selected_joints.get(&n.id()).is_some())
                .collect_vec();
            let active_points = tree
                .iter(DepthFirst, None)
                .map(|n| selected_points.get(&n.id()).is_some())
                .collect_vec();

            // TODO need to convert 2D to this. or accept an iterator over an array view in `accumulate_transformations` (better!)

            // compute transformations only once
            let nodes_trafos = tree
                .iter(DepthFirst, None)
                .accumulate_transformations(params.as_slice().unwrap(), 42)
                .enumerate()
                .map(|(idx, (node, trafo))| (idx, node, trafo)) // flatten
                .collect_vec();

            let (m, n) = (selected_joints.len(), selected_points.len());
            // I guess either column layout or the transpose if we want to go with the 3 trick
            // now go with transpose
            let mut jacobian = Array3::<f64>::zeros((m, n, 3));

            // nomenclature (as in https://stats.stackexchange.com/a/588492): row-, column-, tube fibers

            // TODO this does not jet respect multiple axes!
            jacobian
                .axis_iter_mut(Axis(0))
                // .into_par_iter()
                .zip(
                    nodes_trafos
                        .iter()
                        .zip(active_joints.iter()) // Add the active joint lists
                        .filter_map(|(x, active)| if *active { Some(x) } else { None }), // filter inactive joints and remove flag
                                                                                         // .par_iter()
                )
                /* .Initially I wanted to iterate over the degrees of freedom, however each
                axis should have its own `trafo` including the previous axis's tranformation
                this makes it much more complex than anticipated.
                Rather handle this internally. E.g. NodeID being a tuple data type of <string,usize> and implement hasfunction for this
                map(|(idx, node, trafo)| ()),
                That works! (surprisingly enough)
                However, a 2D spline joint would be difficult to achieve (but doable with tuples of float as parameter type and a bit of duplication)
                */
                .for_each(|(mut row, (idx, joint_node, joint_trafo))| {
                    izip!(
                        tree.iter(DepthFirst, Some(joint_node)), // iterating over the child tree
                        // zipping the corresponding trafos (by skipping until the current node)
                        // Using the index here is ok, keeping an iterator is to hard (gets mutated in a different closure)
                        nodes_trafos.iter().skip(*idx).map(|(_, _, trafo)| trafo),
                        row.axis_iter_mut(Axis(0)),
                        active_points.iter()
                    )
                    .filter(|(_, _, _, active)| **active)
                    .for_each(|(point_node, point_trafo, mut tube, _)| {
                        // joint_trafo^{-1} * point_trafo is the argument to compute the displacement
                        tube.assign(&Array1::<f64>::zeros(3));
                    });
                });
        }

        let selected_joints = HashSet::<String>::new();
        let selected_points = HashSet::<String>::new();

        let tree = ArenaTree::<Bone, LinkNodeId>::new();

        // TODO That's what I want to put in
        let params = Array1::<f64>::ones(42);
        ik(&tree, &params, &selected_joints, &selected_points);
    }

    #[test]
    fn hash_tuple() {
        let mut hs = HashSet::<(String, usize)>::new();

        hs.insert(("Shoulder".to_string(), 1));
    }
}
