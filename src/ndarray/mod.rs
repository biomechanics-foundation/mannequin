//! Module for the implementations using the ndarray backend. Coontains the basic calculus required
use crate::MannequinError;
use ndarray::{prelude::*, ErrorKind::IncompatibleShape, ShapeError};

pub mod robot;

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

/// inverts a homogeneous, 4x4 transformation matrix.
pub fn invert_transformation_4x4(trafo: &Array2<f64>) -> Array2<f64> {
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

#[allow(unused_variables)]
pub fn solve_linear(matrix: &[f64], rows: usize, cols: usize, vector: &[f64], target_buffer: &mut [f64]) {
    target_buffer.iter_mut().for_each(|el| *el = 0.0);
}
