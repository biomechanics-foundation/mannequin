use crate::RigidBody;

pub struct DummyBody {
    value: f64,
}
impl DummyBody {
    fn new(value: f64) -> Self {
        DummyBody { value }
    }
}
impl RigidBody for DummyBody {
    type Transformation = f64;

    type Point = i32;

    type Parameter = f64;

    fn transformation(&self, param: &Self::Parameter) -> Self::Transformation {
        self.value * param
    }

    fn derivative(&self, param: Self::Parameter) -> Self::Transformation {
        todo!()
    }

    fn neutral_element() -> Self::Transformation {
        todo!()
    }

    fn n_dof(&self) -> i32 {
        1
    }

    fn concat(first: &Self::Transformation, second: &Self::Transformation) -> Self::Transformation {
        first + second
    }

    fn globalize(&self, other: &Self::Point) -> Self::Point {
        todo!()
    }

    fn localize(&self, other: &Self::Point) -> Self::Point {
        todo!()
    }
}
