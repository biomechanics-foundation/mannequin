/*! Dummy implementation of a rigid body used for testing */

// TODO move to the test directory once development is done

use crate::Rigid;

#[derive(Debug, PartialEq)]
pub struct DummyBody {
    value: f64,
}
impl DummyBody {
    pub fn new(value: f64) -> Self {
        DummyBody { value }
    }
}
impl Rigid for DummyBody {
    type Transformation = f64;

    type Point = i32;

    type Parameter = f64;

    type NodeId = i32;

    fn transform(&self, param: &Self::Parameter) -> Self::Transformation {
        self.value * param
    }

    fn neutral_element() -> Self::Transformation {
        todo!()
    }

    fn concat(first: &Self::Transformation, second: &Self::Transformation) -> Self::Transformation {
        first + second
    }

    fn globalize(&self, _other: &Self::Point) -> Self::Point {
        todo!()
    }

    fn localize(&self, _other: &Self::Point) -> Self::Point {
        todo!()
    }

    fn invert(_trafo: &Self::Transformation) -> Self::Transformation {
        todo!()
    }

    fn dim(&self) -> usize {
        todo!()
    }

    fn effector_count(&self) -> usize {
        todo!()
    }

    fn partial_derivative(
        &self,
        _pose: &Self::Transformation,
        _joint: &Self,
        _joint_pose: &Self::Transformation,
        _target_buffer: &mut [f64],
        offset: usize,
    ) {
        todo!()
    }

    fn configuration(&self, pose: &Self::Transformation, target_buffer: &mut [f64], offset: usize) {
        todo!()
    }
}
