use ndarray::{Array1, Array2, Array4};

use crate::{ArenaTree, Forward, Inverse, Mannequin, Rigid};

struct RobotLink {}

impl Rigid for RobotLink {
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

struct RobotFK {}
impl Forward<ArenaTree<RobotFK>, RobotLink> for RobotFK {
    type Parameter = Array1<f64>;

    type Array = Array2<f64>;

    fn solve(
        &mut self,
        tree: &ArenaTree<RobotFK>,
        param: Self::Parameter,
        target_refs: &[<ArenaTree<RobotFK> as crate::TreeIterable>::NodeRef],
    ) -> Self::Array {
        todo!()
    }
}

struct RobotIK {}
impl Inverse for RobotIK {
    type Parameter;

    type Array;

    fn solve(
        &mut self,
        tree: &IT,
        fk: &FK,
        param: Self::Parameter,
        target_refs: &[IT::NodeRef],
        target_val: &[Self::Array],
    ) -> Self::Parameter {
        todo!()
    }
}

type Robot = Mannequin<ArenaTree<RobotLink>, RobotLink, RobotFK, RobotIK>;
