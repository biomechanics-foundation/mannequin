//! Module for the implementations using the ndarray backend. Coontains the basic calculus required
use crate::MannequinError;
use ndarray::{prelude::*, ErrorKind::IncompatibleShape, ShapeError};
use ndarray_linalg::{Inverse, LeastSquaresSvd, Solve, QR};

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
pub fn solve_linear(matrix: ArrayView2<f64>, vector: ArrayView1<f64>, mut target: ArrayViewMut1<f64>) {
    // dbg!(&matrix);
    // dbg!(matrix.t().dot(&matrix));
    // dbg!(&vector);

    // This works. No idea about performance

    let mut pseudo_inverse = matrix.t().dot(&matrix);
    // regularization
    pseudo_inverse = &pseudo_inverse + 1e-5 * Array2::<f64>::eye(pseudo_inverse.nrows());
    pseudo_inverse = pseudo_inverse.inv().unwrap().dot(&matrix.t());
    target.assign(&pseudo_inverse.dot(&vector));

    // let result = matrix.least_squares(&vector).unwrap();
    // target.assign(&result.solution);
    // if matrix.rank()

    // let (q, r) = matrix.qr().unwrap();
    // let left_inverse = r.inv().unwrap().dot(&q.t());

    // this might works but only if the rank is full (over-determined system)
    // println!("{matrix}");
    // println!("{q}");
    // println!("{r}");
    // println!("{left_inverse}");
    // dbg!(left_inverse.dot(&matrix));
    // // dbg!(left_inverse.shape());
    // let t = left_inverse.dot(&vector);
    // // dbg!(target.shape());
    // target.assign(&t);
    // matrix.solve_t(&vector);
    // dbg!(matrix.solve_t(&vector));
    // dbg!(matrix.solve(&vector));
    // dbg!();
    // target.assign(&matrix.solve_t(&vector).expect("Cannot solve equations"));
    // target.iter_mut().for_each(|x| *x = 0.0);
}

// TODO Move functions into `spatial.rs` module
// Make a struct implementing Rigid that has a generic member `nested` with a trait NestedRigid
// that delegates everything to `nested` but with ndarray types
// then move the robot implementation to example (unless used in benchmarks, then leave it in (as a feature maybe)).
