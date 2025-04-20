use faer::{linalg::solvers::Solve, ColRef, Mat, MatRef, Scale};
use faer_traits::RealField;
use num_traits::Float;

/// Solves the normal equations with the [faer crate](https://docs.rs/faer/latest/faer/index.html)
///
/// Notes: I am not too happy with constructing the normal equations, but it works for now.
/// References to solve the equations with QR decomposition:
/// * https://math.stackexchange.com/questions/3518247/least-squares-using-qr-for-underdetermined-system
/// * https://eigen.tuxfamily.org/dox/group__TutorialLinearAlgebra.html
/// * https://math.stackexchange.com/a/2852117
pub fn solve_linear<F>(matrix: &[F], rows: usize, cols: usize, vector: &[F], parameters: &mut [F], limit_radians: F)
where
    F: RealField + Float,
{
    let matrix = MatRef::from_column_major_slice(matrix, rows, cols);
    let vector = ColRef::from_slice(vector);

    let diff = matrix.transpose() * vector;
    let matrix = matrix.transpose() * matrix + Mat::<F>::identity(cols, cols) * 1e-5;

    let lu = matrix.partial_piv_lu();

    let mut update = lu.solve(diff);

    // limit update to 10 degrees (assuming that the function is near linear in that range)
    let norm = update.norm_l2();

    update /= Scale(norm);

    let limited = limit_radians.min(norm);

    update *= Scale(limited);

    dbg!(limited, update.norm_l2());

    update.iter().zip(parameters).for_each(|(a, b)| *b = *a);
}

#[cfg(test)]
mod test {
    use std::f32::consts::PI;

    use super::solve_linear;

    #[test]
    fn test_f32() {
        let matrix = [0f32; 3 * 6];
        let mut param = [0f32; 6];
        let target = [0f32; 3];
        let limit = PI / 18.0;

        solve_linear(&matrix, 3, 6, &target, &mut param, limit);
    }
}
