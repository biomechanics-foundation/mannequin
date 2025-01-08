use itertools::{izip, Itertools};

use crate::{accumulate, DepthFirst, Rigid, TransformationAccumulation, TreeIterable};
#[cfg(feature = "rayon")]
use rayon::prelude::*;
use std::hash::Hash;

// TODO maybe make precision generic
trait Differentiable<T, R, I>
where
    T: TreeIterable<R, I>,
    R: Rigid,
    I: Eq + Clone + Hash,
{
    type Data;

    /// returns a reference to the internal data type
    fn jacobian(&self) -> &Self::Data;

    /// compute the jacobian matrix
    fn compute(&mut self, tree: &T, params: &[R::Parameter], active_joints: &[bool], active_points: &[bool]) {
        // selected_joints: &HashSet<String>,
        // selected_points: &HashSet<String>,
        // let active_joints = tree
        //     .iter(DepthFirst, None)
        //     .map(|n| selected_joints.get(&n.id()).is_some())
        //     .collect_vec();
        // let active_points = tree
        //     .iter(DepthFirst, None)
        //     .map(|n| selected_points.get(&n.id()).is_some())
        //     .collect_vec();

        // compute transformations only once
        let nodes_trafos = tree
            .iter(DepthFirst, None)
            .accumulate_transformations(params, 42)
            .enumerate()
            .map(|(idx, (node, trafo))| (idx, node, trafo)) // flatten
            .collect_vec();

        // I guess either column layout or the transpose if we want to go with the 3 trick
        // now go with transpose
        let (mut jacobian, m, n) = self.data(); // Array3::<f64>::zeros((m, n, 3));

        // nomenclature (as in https://stats.stackexchange.com/a/588492): row-, column-, tube fibers

        // TODO this does not jet respect multiple axes!
        jacobian
            .iter_mut()
            .chunks(n * 3) // TODO let the 3 be determined by the nodes and do a manual `chunk_by`
            // TODO use rayon: into_par_iter()
            .into_iter()
            .zip(
                nodes_trafos
                    .iter()
                    .zip(active_joints.iter()) // Add the active joint lists
                    .filter_map(|(x, active)| if *active { Some(x) } else { None }), // filter inactive joints and remove flag
                                                                                     // .par_iter()
            )
            /* .Initially I wanted to iterate over the degrees of freedom, however each
            axis should have its own `trafo` including the previous axis's tranformation
            this makes it much more complex than anticipated.
            Rather handle this internally. E.g. NodeID being a tuple data type of <string,usize> and implement hasfunction for this
            map(|(idx, node, trafo)| ()),
            That works! (surprisingly enough)
            However, a 2D spline joint would be difficult to achieve (but doable with tuples of float as parameter type and a bit of duplication)
            */
            .for_each(|(mut row, (idx, joint_node, joint_trafo))| {
                izip!(
                    tree.iter(DepthFirst, Some(joint_node)), // iterating over the child tree
                    // zipping the corresponding trafos (by skipping until the current node)
                    // Using the index here is ok, keeping an iterator is to hard (gets mutated in a different closure)
                    nodes_trafos.iter().skip(*idx).map(|(_, _, trafo)| trafo),
                    row.chunks(3).into_iter(),
                    active_points.iter()
                )
                .filter(|(_, _, _, active)| **active)
                .for_each(|(point_node, point_trafo, mut tube, _)| {
                    // joint_trafo^{-1} * point_trafo is the argument to compute the displacement
                    // Assign to slice without bounday checks
                    tube.into_iter()
                        .zip(vec![0f64; 3].into_iter())
                        .for_each(|(a, b)| *a = b);
                });
            });
    }

    /// provide a reference to the raw data and the dimensions of its axes (so first the major axis)
    fn data(&mut self) -> (&mut [f64], usize, usize);

    // fn set(&mut self, data: &mut [f64], m: usize, n: usize )
}

#[cfg(feature = "ndarray")]
mod ndarray {
    use super::Differentiable;
    use crate::{Rigid, TreeIterable};
    use ndarray::Array2;
    use std::hash::Hash;

    struct NDArrayJacobian {
        data: Array2<f64>,
    }

    impl<T, R, I> Differentiable<T, R, I> for NDArrayJacobian
    where
        T: TreeIterable<R, I>,
        R: Rigid,
        I: Eq + Clone + Hash,
    {
        // Note: could ArrayView too
        type Data = Array2<f64>;

        fn jacobian(&self) -> &Self::Data {
            &self.data
        }

        fn data(&mut self) -> (&mut [f64], usize, usize) {
            let shape = self.data.shape();
            let (m, n) = (shape[0], shape[1]);
            (self.data.as_slice_mut().unwrap(), m, n)
        }
    }
}

#[cfg(feature = "nalgebra")]
mod nalgebra {
    use super::Differentiable;
    use crate::{Rigid, TreeIterable};
    use nalgebra::DMatrix;
    use std::hash::Hash;

    struct NAlgebraJacobian {
        data: DMatrix<f64>,
    }

    impl<T, R, I> Differentiable<T, R, I> for NAlgebraJacobian
    where
        T: TreeIterable<R, I>,
        R: Rigid,
        I: Eq + Clone + Hash,
    {
        // Note: could ArrayView too
        type Data = DMatrix<f64>;

        fn jacobian(&self) -> &Self::Data {
            &self.data
        }

        fn data(&mut self) -> (&mut [f64], usize, usize) {
            let (m, n) = (self.data.ncols(), self.data.nrows());
            (self.data.as_mut_slice(), m, n)
        }
    }
}
