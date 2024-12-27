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
use ndarray::{Array1, Array2, Array4};
use std::marker::PhantomData;

#[derive(Debug, Default, PartialEq)]
pub struct Link {
    from_parent: Array2<f64>,
}

impl Link {
    pub fn new(from_parent: &Array2<f64>) -> Self {
        Self {
            from_parent: from_parent.clone(),
        }
    }
}

impl fmt::Display for Link {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "Just a link")
    }
}

impl Rigid for Link {
    type Transformation = Array2<f64>;

    type Point = Array1<f64>;

    type Parameter = f64;

    type NodeId = String;

    fn transformation(&self, param: &Self::Parameter) -> Self::Transformation {
        Self::concat(
            &self.from_parent,
            &array![
                [param.cos(), -1.0 * param.sin(), 0.0, 0.0],
                [param.sin(), param.cos(), 0.0, 0.0],
                [0.0, 0.0, 1.0, 0.0],
                [0.0, 0.0, 0.0, 1.0]
            ],
        )
    }

    fn globalize(&self, other: &Self::Point) -> Self::Point {
        todo!()
    }

    fn localize(&self, other: &Self::Point) -> Self::Point {
        todo!()
    }

    fn derivative(&self, param: Self::Parameter) -> Self::Transformation {
        todo!()
    }

    fn n_dof(&self) -> i32 {
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

type LinkNodeId = <Link as Rigid>::NodeId;

/// Specialization of a Forward Kinematics on an [ArenaTree]
pub struct ForwardsKinematics {
    max_depth: usize,
}
impl Forward<ArenaTree<Link, LinkNodeId>, Link> for ForwardsKinematics {
    type Parameter = Array1<f64>;

    type Transformation = Array2<f64>;

    fn solve(
        &mut self,
        tree: &ArenaTree<Link, LinkNodeId>,
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
    <DifferentialIK as Inverse<ArenaTree<Link, LinkNodeId>, Link, ForwardsKinematics>>::Parameter;
pub type DifferentialIKArray =
    <DifferentialIK as Inverse<ArenaTree<Link, LinkNodeId>, Link, ForwardsKinematics>>::Array;

impl DifferentialIK {
    pub fn jacobian(&self, tree: &ArenaTree<Link, LinkNodeId>, params: DifferentialIKParameter) -> DifferentialIKArray {
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
                        let local = Link::concat(&Link::invert(trafo), child_trafo);
                        let pos = local.slice(s![..3, 3]);

                        dof.slice_mut(s![..3, *idx]).assign(&pos);

                        println!("column:{}, {}, {dof}", idx + 1, child.depth())
                    })
            });
        // is this slower?
        jacobian.into_shape_with_order((n_dof * 3, n_tcp)).unwrap()
    }
}

impl Inverse<ArenaTree<Link, LinkNodeId>, Link, ForwardsKinematics> for DifferentialIK {
    type Parameter = Array1<f64>;

    type Array = Array2<f64>;

    fn solve(
        &mut self,
        tree: &ArenaTree<Link, LinkNodeId>,
        fk: &ForwardsKinematics,
        param: Self::Parameter,
        target_refs: &[LinkNodeId],
        target_val: &[Self::Array],
    ) -> Self::Parameter {
        todo!()
    }
}

pub type Robot = Mannequin<ArenaTree<Link, LinkNodeId>, Link, ForwardsKinematics, DifferentialIK>;

#[cfg(test)]
mod tests {
    use itertools::Itertools;
    use ndarray::prelude::*;

    use crate::{ndarray::robot::LinkNodeId, ArenaTree, Forward, Order::DepthFirst, Rigid, TreeIterable};

    use super::{DifferentialIK, ForwardsKinematics, Link};

    #[test]
    fn test_fk() {
        let mut fk = ForwardsKinematics { max_depth: 10 };
        let mut tree = ArenaTree::<Link, LinkNodeId>::new();

        let mut trafo = Link::neutral_element();
        trafo.slice_mut(s![..3, 3]).assign(&array![10.0, 0.0, 0.0]);

        let link1 = Link::new(&trafo);
        let link2 = Link::new(&trafo);
        let link3 = Link::new(&trafo);
        let link4 = Link::new(&trafo);

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
        let mut tree = ArenaTree::<Link, LinkNodeId>::new();

        let mut trafo = Link::neutral_element();
        trafo.slice_mut(s![..3, 3]).assign(&array![10.0, 0.0, 0.0]);

        let link1 = Link::new(&trafo);
        let link2 = Link::new(&trafo);
        let link3 = Link::new(&trafo);
        let link4 = Link::new(&trafo);

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
}
