#![allow(unused_variables)]

use ndarray::{Array1, Array2, Array4};

use crate::{accumulate, ArenaTree, DepthFirst, Forward, Inverse, Mannequin, Rigid, TreeIterable};

struct Link {}

impl Rigid for Link {
    type Transformation = Array2<f64>;

    type Point = Array1<f64>;

    type Parameter = Array1<f64>;

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

impl DifferentialIK {
    pub fn jacobian(&self, tree: &ArenaTree<Link>) {
        #[cfg(not(feature = "accumulate"))]
        {
            let param = &[1.0, 2.0, 3.0];
            let zip = tree
                .iter(DepthFirst, &[])
                .zip(param.iter())
                .scan(Vec::<<Link as Rigid>::Transformation>::with_capacity(42), accumulate)
                .for_each(|a| {});
        }
        #[cfg(feature = "accumulate")]
        {
            // Want to use this!
            tree.iter(DepthFirst, &[]).accumulate(param, 32).collect_vec();
        }
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
