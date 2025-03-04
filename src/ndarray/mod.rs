//! Module for the implementations using the ndarray backend. Coontains the basic calculus required
use crate::differentiable::ComputeSelection;
use crate::{DepthFirstIterable, Differentiable, MannequinError, Rigid, VecJacobian};
use ndarray::Order;
use ndarray::{prelude::*, ErrorKind::IncompatibleShape, ShapeError};
use std::collections::HashSet;
use std::fmt::Debug;
use std::hash::Hash;

pub mod default;

/// Jacobian using ndarray for numerics
#[derive(Debug, Default)]
pub struct Jacobian {
    base: VecJacobian,
}

/// Thin wrapper around the default implementation
impl Jacobian {
    pub fn new() -> Self {
        Self::default()
    }
}

impl Differentiable for Jacobian {
    // Note: could ArrayView too
    type Data<'a> = ArrayView2<'a, f64>;

    fn jacobian(&self) -> Self::Data<'_> {
        let data = self.base.jacobian();
        let result = ArrayView1::<f64>::from(data.as_slice())
            .into_shape_with_order(((self.base.rows(), self.base.cols()), Order::ColumnMajor))
            .unwrap();
        result
    }

    // fn data(&mut self) -> &mut [f64] {
    //     self.base.data()
    // }

    fn setup<T, R, I>(&mut self, tree: &T, active_joints: &HashSet<&I>, active_points: &HashSet<&I>)
    where
        T: DepthFirstIterable<R, I>,
        R: Rigid,
        I: Eq + Clone + Hash + Debug,
    {
        self.base.setup(tree, active_joints, active_points)
    }

    fn rows(&self) -> usize {
        self.base.rows()
    }

    fn cols(&self) -> usize {
        self.base.cols()
    }

    fn compute<T, R, I>(&mut self, tree: &T, params: &[<R as Rigid>::Parameter], selection: ComputeSelection)
    where
        T: DepthFirstIterable<R, I>,
        R: Rigid,
        I: Eq + Clone + Hash + Debug,
    {
        self.base.compute(tree, params, selection)
    }

    fn configuration(&self) -> Self::Data<'_> {
        todo!()
    }
}

/// Creates a homogeneous, 4x4 rotation matrix around the x axis.
pub fn rotate_x_4x4(param: f64) -> Array2<f64> {
    array![
        [1.0, 0.0, 0.0, 0.0],
        [0.0, param.cos(), -1.0 * param.sin(), 0.0],
        [0.0, param.sin(), param.cos(), 0.0],
        [0.0, 0.0, 0.0, 1.0]
    ]
}

/// Creates a homogeneous, 4x4 rotation matrix around the y axis.
pub fn rotate_y_4x4(param: f64) -> Array2<f64> {
    array![
        [param.cos(), 0.0, param.sin(), 0.0],
        [0.0, 1.0, 0.0, 0.0],
        [(-1.0) * param.sin(), 0.0, param.cos(), 0.0],
        [0.0, 0.0, 0.0, 1.0]
    ]
}

/// Creates a homogeneous, 4x4 rotation matrix around the z axis.
pub fn rotate_z_4x4(param: f64) -> Array2<f64> {
    array![
        [param.cos(), -1.0 * param.sin(), 0.0, 0.0],
        [param.sin(), param.cos(), 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0]
    ]
}

/// Creates a homogeneous, 4x4 translation matrix along the x axis.
pub fn translate_x_4x4(param: f64) -> Array2<f64> {
    array![
        [1.0, 0.0, 0.0, param],
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]
}

/// Creates a homogeneous, 4x4 translation matrix along the y axis.
pub fn translate_y_4x4(param: f64) -> Array2<f64> {
    array![
        [1.0, 0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0, param],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]
}

/// Creates a homogeneous, 4x4 translation matrix along the z axis.
pub fn translate_z_4x4(param: f64) -> Array2<f64> {
    array![
        [1.0, 0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, param],
        [0.0, 0.0, 0.0, 1.0],
    ]
}

/// Create a translation from a slice
pub fn translation<T>(param: &[f64]) -> Result<Array2<f64>, MannequinError<T>> {
    let mut result = Array2::<f64>::eye(4);
    result
        .slice_mut(s![..3, 3])
        .assign(&ArrayView1::<f64>::from_shape(3, param)?);
    Ok(result)
}

/// inverts a homogeneous, 4x4 tranformation matrix.
pub fn invert_tranformation_4x4(trafo: &Array2<f64>) -> Array2<f64> {
    let mut result = Array2::<f64>::eye(4);
    let rot = trafo.slice(s![..3, ..3]);
    result.slice_mut(s![..3, ..3]).assign(&rot.t());
    let ipos = &trafo.slice(s![..3, 3]) * -1.0;
    result.slice_mut(s![..3, 3]).assign(&ipos);
    result
}

pub fn cross_3d<T>(
    a: ArrayView1<f64>,
    b: ArrayView1<f64>,
    mut target: ArrayViewMut1<f64>,
) -> Result<(), MannequinError<T>> {
    if a.len() != 3 || b.len() != 3 || target.len() != 3 {
        Err(ShapeError::from_kind(IncompatibleShape).into())
    } else {
        target[0] = a[1] * b[2] - a[2] * b[1];
        target[1] = a[2] * b[0] - a[0] * b[2];
        target[2] = a[0] * b[1] - a[1] * b[0];
        Ok(())
    }
}
