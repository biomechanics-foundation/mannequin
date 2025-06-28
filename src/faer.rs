use faer::{
    ColMut, ColRef, Mat, MatRef, Scale,
    linalg::solvers::{DenseSolveCore, Solve},
};
use faer_traits::RealField;
use num_traits::Float;

/// Solves the normal equations with the [faer crate](https://docs.rs/faer/latest/faer/index.html)
///
/// Notes: I am not too happy with constructing the normal equations, but it works for now.
/// References to solve the equations with QR decomposition:
/// * https://math.stackexchange.com/questions/3518247/least-squares-using-qr-for-underdetermined-system
/// * https://eigen.tuxfamily.org/dox/group__TutorialLinearAlgebra.html
/// * https://math.stackexchange.com/a/2852117
/// * https://people.csail.mit.edu/bkph/articles/Pseudo_Inverse.pdf
pub fn solve_linear<F>(matrix: &[F], rows: usize, cols: usize, vector: &[F], parameters: &mut [F], limit_radians: F)
where
    F: RealField + Float + Into<f64>,
{
    let matrix = MatRef::from_column_major_slice(matrix, rows, cols);
    let vector = ColRef::from_slice(vector);

    let mut update = ColMut::from_slice_mut(parameters);

    match rows.cmp(&cols) {
        std::cmp::Ordering::Less => {
            // underdetermined
            let square = matrix * matrix.transpose() + Mat::<F>::identity(rows, rows) * 1e-5;
            let lu = square.partial_piv_lu();
            update.copy_from(matrix.transpose() * lu.inverse() * vector);
        }
        std::cmp::Ordering::Equal => {
            // square
            let square = matrix + Mat::<F>::identity(cols, cols) * 1e-5;
            let lu = square.partial_piv_lu();

            // one can use solve_in_place to avoid one allocation
            // avoiding the other allocations requires using the low level api
            // therefore we'd need to initialize copy_from_slice on `Vector`
            update.copy_from(lu.solve(vector));
        }
        std::cmp::Ordering::Greater => {
            // overdetermined
            let vector = matrix.transpose() * vector;
            let mut square = matrix.transpose() * matrix;
            // Regularization in case of singularity. TODO: Convert to constant
            square += Mat::<F>::identity(cols, cols) * 1e-5;
            let lu = square.partial_piv_lu();
            update.copy_from(lu.solve(vector));
        }
    }

    // limit update to 10 degrees (assuming that the function is near linear in that range)

    let norm = update.norm_l2();

    if norm.into() > 1e-5 {
        update /= Scale(norm);

        let limited = limit_radians.min(norm);

        update *= Scale(limited);

        dbg!(limited, update.norm_l2());
    }

    // update.iter().zip(parameters).for_each(|(a, b)| *b = *a);
}

#[cfg(test)]
mod test {
    use super::solve_linear;
    use std::f32::consts::PI;

    // Watch out examples are singular!

    #[test]
    fn test_f32() {
        // let matrix = [0f32; 3 * 6];
        let matrix = [
            0f32, 1f32, 2f32, 3f32, 4f32, 5f32, 1f32, 2f32, 3f32, 4f32, 5f32, 6f32, 2f32, 3f32, 4f32, 5f32, 6f32, 7f32,
        ];
        let mut param = [0f32; 6];

        // let target = [0f32; 3];
        let target = [0f32, 1f32, 2f32];

        let limit = PI / 18.0;

        solve_linear(&matrix, 3, 6, &target, &mut param, limit);

        dbg!(param);
    }

    #[test]
    fn test_minimal() {
        use faer::{ColMut, ColRef, MatRef, linalg::solvers::DenseSolveCore};
        // let matrix = [0f32; 3 * 6];
        let matrix = [
            0f32, 1f32, 2f32, 3f32, 4f32, 5f32, 1f32, 2f32, 3f32, 4f32, 5f32, 6f32, 2f32, 3f32, 4f32, 5f32, 6f32, 7f32,
        ];
        let matrix = MatRef::from_column_major_slice(&matrix, 3, 6);

        let mut result = [0f32; 6];
        let mut update = ColMut::from_slice_mut(&mut result);

        let vector = [0f32, 1f32, 2f32];
        let vector = ColRef::from_slice(&vector);

        let square = matrix * matrix.transpose();
        let lu = square.partial_piv_lu();

        update.copy_from(matrix.transpose() * lu.inverse() * vector);

        dbg!(result);
    }
}
