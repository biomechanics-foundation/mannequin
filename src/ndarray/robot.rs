#![allow(unused_variables)]

use core::fmt;

use crate::{accumulate, Accumulator, ArenaTree, DepthFirst, Forward, Inverse, Mannequin, Rigid, TreeIterable};
use itertools::Itertools;
use ndarray::parallel::prelude::*;
use ndarray::prelude::*;
use ndarray::{Array1, Array2, Array4};

#[derive(Debug, Default)]
struct Link {}

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
        todo!()
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
        todo!()
    }

    fn concat(first: &Self::Transformation, second: &Self::Transformation) -> Self::Transformation {
        todo!()
    }
}

struct SimpleFK {}
impl Forward<ArenaTree<Link>, Link> for SimpleFK {
    type Parameter = Array1<f64>;

    type Array = Array2<f64>;

    fn solve(
        &mut self,
        tree: &ArenaTree<Link>,
        param: Self::Parameter,
        target_refs: &[<ArenaTree<Link> as crate::TreeIterable<Link>>::NodeRef],
    ) -> Self::Array {
        todo!()
    }
}

struct DifferentialIK {}

type DifferentialIKParameter = <DifferentialIK as Inverse<ArenaTree<Link>, Link, SimpleFK>>::Parameter;
type DifferentialIKArray = <DifferentialIK as Inverse<ArenaTree<Link>, Link, SimpleFK>>::Array;

impl DifferentialIK {
    pub fn jacobian(&self, tree: &ArenaTree<Link>, param: DifferentialIKParameter) -> DifferentialIKArray {
        let ndof = 3;
        let ntcp = 2;
        let max = 42;
        // Assumption: row-based
        let mut jacobian = Array2::<f64>::zeros((ndof, ntcp));

        jacobian
            .axis_iter_mut(Axis(0))
            .into_par_iter()
            .zip(tree.iter(DepthFirst, &[]).accumulate(param, max).into_par_iter())
            .for_each(|(row, (node, trafo))| {
                println!("{row}, {node}, {trafo}");

                // tree
                //     .iter(DepthFirst, &[tree.node_ref(node)])
                //     .accumulate(param, max)
                //     .zip(row.iter_chunks(3))
                //     .for_each(|(cell, (node, transformation)| {
                //         // compute displacement
                //     });
            });

        #[cfg(not(feature = "accumulate"))]
        {
            jacobian
                .axis_iter_mut(Axis(0))
                .into_par_iter()
                .zip(
                    tree.iter(DepthFirst, &[])
                        .zip(param.iter())
                        .scan(Vec::<<Link as Rigid>::Transformation>::with_capacity(42), accumulate)
                        //.into_par_iter() // not implemented yet
                        .collect_vec()
                        .into_par_iter(),
                )
                .for_each(|(row, (node, trafo))| {
                    println!("{row}, {node}, {trafo}");
                    // TODO iterte over subtree and fill the columns (non-parallel iteration in chunks of three)
                });
        }
        #[cfg(feature = "accumulate")]
        {
            jacobian
                .axis_iter_mut(Axis(0))
                .into_par_iter()
                .zip(
                    tree.iter(DepthFirst, &[])
                        .accumulate(param, 42)
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

type Robot = Mannequin<ArenaTree<Link>, Link, SimpleFK, DifferentialIK>;
