//! Implementation of traits as specializations for ArenaTree using ndarray
#![allow(unused_variables)]

use crate::{accumulate, Accumulator, ArenaTree, DepthFirst, Forward, Inverse, Mannequin, Rigid, TreeIterable};
use core::fmt;
use itertools::Itertools;
use ndarray::parallel::prelude::*;
use ndarray::prelude::*;
use ndarray::{Array1, Array2, Array4};

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
        let mut neutral = Self::neutral_element();
        let rot = trafo.slice(s![..3, ..3]);
        neutral.slice_mut(s![..3, ..3]).assign(&rot.t());
        let ipos = &trafo.slice(s![..3, 3]) * -1.0;
        neutral.slice_mut(s![..3, 3]).assign(&ipos);
        todo!()
    }
}

/// Specialization of a Forward Kinematics on an [ArenaTree]
pub struct SimpleFK {
    max_depth: usize,
}
impl Forward<ArenaTree<Link>, Link> for SimpleFK {
    type Parameter = Array1<f64>;

    type Transformation = Array2<f64>;

    fn solve(
        &mut self,
        tree: &ArenaTree<Link>,
        params: Self::Parameter,
        target_refs: &[<ArenaTree<Link> as crate::TreeIterable<Link>>::NodeRef],
    ) -> Vec<Self::Transformation> {
        tree.iter(DepthFirst, &[])
            .accumulate(params.as_slice().unwrap(), self.max_depth)
            .filter_map(|(node, trafo)| {
                if target_refs.contains(&tree.get_ref(node)) {
                    Some(trafo)
                } else {
                    None
                }
            })
            .collect_vec()
    }
}

pub struct DifferentialIK {}

pub type DifferentialIKParameter = <DifferentialIK as Inverse<ArenaTree<Link>, Link, SimpleFK>>::Parameter;
pub type DifferentialIKArray = <DifferentialIK as Inverse<ArenaTree<Link>, Link, SimpleFK>>::Array;

impl DifferentialIK {
    pub fn jacobian(&self, tree: &ArenaTree<Link>, param: DifferentialIKParameter) -> DifferentialIKArray {
        let ndof = 3;
        let ntcp = 2;
        let max = 42;
        // Assumption: row-based
        let mut jacobian = Array2::<f64>::zeros((ndof, ntcp));

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

        {
            jacobian
                .axis_iter_mut(Axis(0))
                .into_par_iter()
                .zip(
                    tree.iter(DepthFirst, &[])
                        .accumulate(param.as_slice().expect("Cannot convert parameters to slice!"), 42)
                        //.into_par_iter() // not implemented yet
                        .collect_vec()
                        .into_par_iter(),
                )
                .for_each(|(row, (node, trafo))| println!("{row}, {node}, {trafo}"));
        }
        jacobian
    }
}

impl Inverse<ArenaTree<Link>, Link, SimpleFK> for DifferentialIK {
    type Parameter = Array1<f64>;

    type Array = Array2<f64>;

    fn solve(
        &mut self,
        tree: &ArenaTree<Link>,
        fk: &SimpleFK,
        param: Self::Parameter,
        target_refs: &[<ArenaTree<Link> as crate::TreeIterable<Link>>::NodeRef],
        target_val: &[Self::Array],
    ) -> Self::Parameter {
        todo!()
    }
}

pub type Robot = Mannequin<ArenaTree<Link>, Link, SimpleFK, DifferentialIK>;

#[cfg(test)]
mod tests {
    use ndarray::prelude::*;

    use crate::{ArenaTree, Forward, Order::DepthFirst, Rigid, TreeIterable};

    use super::{Link, SimpleFK};

    #[test]
    fn test_fk() {
        let mut fk = SimpleFK { max_depth: 10 };
        let mut tree = ArenaTree::<Link>::new();

        let mut trafo = Link::neutral_element();
        trafo.slice_mut(s![..3, 3]).assign(&array![10.0, 0.0, 0.0]);

        let link1 = Link::new(&trafo);
        let link2 = Link::new(&trafo);
        let link3 = Link::new(&trafo);

        // TODO .. can we make the refs fix in a way they don't get optimized away?
        // Then these could be strings even!
        let ref1 = tree.add(link1, None).unwrap();
        let ref2 = tree.add(link2, Some(ref1)).unwrap();
        let ref3 = tree.add(link3, Some(ref1)).unwrap();

        let node1 = tree.get_node_by_ref(&ref1).unwrap();
        let node2 = tree.get_node_by_ref(&ref2).unwrap();
        let node3 = tree.get_node_by_ref(&ref2).unwrap();

        tree.optimize(DepthFirst);

        let res = fk.solve(&tree, array![0.0, 0.0, std::f64::consts::FRAC_PI_2], &[ref2, ref3]);
        println!("{:?}", res)
    }
    #[test]
    fn test_ik() {}
}
