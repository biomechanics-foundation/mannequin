//! Module for the implementations using the ndarray backend.
use crate::{Differentiable, Rigid, TreeIterable, VecJacobian};
use ndarray::prelude::*;
use std::collections::HashSet;
use std::hash::Hash;

pub mod basic;

pub struct NDArrayJacobian {
    base: VecJacobian,
}

impl NDArrayJacobian {
    pub fn new() -> Self {
        Self {
            base: VecJacobian::new(),
        }
    }
}

impl Differentiable for NDArrayJacobian {
    // Note: could ArrayView too
    type Data = Array2<f64>;

    fn jacobian(&self) -> &Self::Data {
        todo!()
    }

    fn data(&mut self) -> &mut [f64] {
        // (self.base as dyn Differentiable<T, R, I, Data = Vec<f64>>).data()
        // <VecJacobian as Differentiable<T, R, I>>::data(&mut self.base)
        self.base.data()
    }

    fn setup<T, R, I>(&mut self, tree: &T, active_joints: &HashSet<I>, active_points: &HashSet<I>)
    where
        T: TreeIterable<R, I>,
        R: Rigid,
        I: Eq + Clone + Hash,
    {
        // <VecJacobian as Differentiable<T, R, I>>::setup(&mut self.base, tree, active_joints, active_points)
        self.base.setup(tree, active_joints, active_points)
    }

    fn rows(&self) -> usize {
        todo!()
    }

    fn cols(&self) -> usize {
        todo!()
    }

    fn compute<T, R, I>(&mut self, tree: &T, params: &[<R as Rigid>::Parameter])
    where
        T: TreeIterable<R, I>,
        R: Rigid,
        I: Eq + Clone + Hash,
    {
        self.base.compute(tree, params)
    }
}
