//! An iterator that traverses a tree depth-first.

use std::marker::PhantomData;

use crate::{Visitable, Visiting};

/// An iterator that traverses a tree depth-first.
/// Required for parallel processing.
pub(crate) struct TreeIterator<'a, V, T, Z>
where
    V: Visiting<'a, T, T::Parameter, T::Accumulator>,
    T: 'a + Visitable,
    Z: Iterator<Item = &'a T::Parameter>,
{
    visitor: V,
    zipped: Z,
    t: PhantomData<T>,
}

impl<'a, V, T, Z> TreeIterator<'a, V, T, Z>
where
    V: Visiting<'a, T, T::Parameter, T::Accumulator>,
    T: 'a + Visitable,
    Z: Iterator<Item = &'a T::Parameter>,
{
    /// Constructor that takes a visitor and an iterator.
    /// ...
    pub fn new(visitor: V, zipped: Z) -> Self {
        Self {
            visitor,
            zipped,
            t: PhantomData,
        }
    }
}

impl<'a, V, T, Z> Iterator for TreeIterator<'a, V, T, Z>
where
    V: Visiting<'a, T, T::Parameter, T::Accumulator>,
    T: Visitable,
    Z: Iterator<Item = &'a T::Parameter>,
    T::Accumulator: Clone,
{
    type Item = T::Accumulator;

    fn next(&mut self) -> Option<Self::Item> {
        match self.visitor.next(self.zipped.next()) {
            // TODO Do I really want to copy? What would be the lifetime of element?
            // Copying would be necessary, for parallel computation with rayon!
            // Instead of copying, it might be possible to hold the accumulates in a vector instead of a stack in the Visitor
            Some(stack) => Some(stack.last().unwrap().1.clone()),
            _ => None,
        }
    }
}

pub trait DepthFirstIteration<'a, V, Z>: Visitable
where
    V: Visiting<'a, Self, Self::Parameter, Self::Accumulator>,
    Z: Iterator<Item = &'a Self::Parameter>,
    Self::Accumulator: Clone,
    Self: 'a,
{
    /// Get a depth-first iterator from the visitor
    /// Another iterator is expected.
    fn zip(&self, zipped: Z, max_depth: usize) -> TreeIterator<'a, V, Self, Z> {
        // TreeIterator::new(self.visitor(max_depth), zipped)
        // TreeIterator::new(
        //     Visitor::new(self, max_depth, |s| s.children(), |s, a, z| s.accumulate(a, z)),
        //     zipped,
        // )
        todo!()
    }

    // TODO create a non-zipped version of the iterator
    // fn iter(&self) ->
}

#[cfg(test)]
mod tests {

    use crate::{DepthFirstIteration, Visitable, Visiting};

    // example how to use the iterator for a kinematics
    struct Kinematics<T> {
        index: usize,
        points: T,
        // influence: float, // Type is wrong
    }

    //
    // struct Rigging<S, T> {
    //     memory: Vec<f64>,
    //     reference_pose: Vec<f64>,
    //     surface_reference: T, // That should probably be a parameter for the constructor
    //                           // surface_local // Surface points in the local cooardinate systems they are relevant to .. or limit it to two nly (which might make sense, or be too restrictive)
    // }

    // impl Rigging {
    //     pub fn new<V>(n_joints: usize, n_points: usize, kinematics: V, max_depth: usize) -> Self
    //     where
    //         V: Visitable,
    //     {
    //         let memory = vec![0.0; n_joints * n_points * 3];
    //         let adjacencies: Vec<&[f64]> = memory.chunks(n_points).collect(); // Cannot move that into the struct--rust structs mue be memcopyable

    //         let visitor = kinematics.visitor(max_depth);
    //         let reference_pose = vec![0; 42];
    //         let reference_pose_iterator = reference_pose.iter();

    //         while let Some(stack) = visitor.next(reference_pose_iterator.next()) {}

    //         Self { memory }
    //     }
    // }

    fn test_kinematics() {}
}
