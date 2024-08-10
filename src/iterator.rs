//! TODO Implement an iterator that maps
//! a tree to a list of accumulated values.
//! Will be required for parallel processing later.

use std::marker::PhantomData;

use crate::{Visitable, Visiting};

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
    // TODO Do I really want to copy? What would be the lifetime of element?
    // Copying would be necessary, for parallel computation with rayon!
    // Instead of copying, it might be possible to hold the accumulates in a vector instead of a stack in the Visitor
    type Item = T::Accumulator;

    fn next(&mut self) -> Option<Self::Item> {
        match self.visitor.next(self.zipped.next()) {
            Some(stack) => Some(stack.last().unwrap().1.clone()),
            _ => None,
        }
    }
}

pub trait TreeIteration<'a, V, Z>: Visitable
where
    V: Visiting<'a, Self, Self::Parameter, Self::Accumulator>,
    Z: Iterator<Item = &'a Self::Parameter>,
    Self::Accumulator: Clone,
    Self: 'a,
{
    fn iter_accumulated(&self, zipped: Z, max_depth: usize) -> TreeIterator<'a, V, Self, Z> {
        // TreeIterator::new(self.visitor(max_depth), zipped)
        // TreeIterator::new(
        //     Visitor::new(self, max_depth, |s| s.children(), |s, a, z| s.accumulate(a, z)),
        //     zipped,
        // )
        todo!()
    }
}

#[cfg(test)]
mod tests {

    fn test_iterator() {}
}
