/*! Defines the payload carried by [Nodelike] in the context of kinematics/character animation */

use crate::Nodelike;

/// A Rigid Body represents a single, rigid link connected to other links via a joint.
pub trait Rigid {
    /// E.g., 4x4 matrix, (3x1, 3x3), quaternions ...
    type Transformation: Clone;
    /// Vec, \[f64;4\], ...
    type Point;
    /// typically joint positions (angles/extension), f64, \[f64,3\]
    type Parameter;

    /// Get the Transformation from the parent taking the connecting joint into account
    fn transformation(&self, param: &Self::Parameter) -> Self::Transformation;

    /// Transform a point into the world coordinate system
    fn globalize(&self, other: &Self::Point) -> Self::Point;

    /// Transform a point into the local coordinate system
    fn localize(&self, other: &Self::Point) -> Self::Point;

    /// ...
    fn derivative(&self, param: Self::Parameter) -> Self::Transformation;

    /// ...
    fn n_dof(&self) -> i32;

    /// Returns the eutral element wrt. the transoformation convention used
    fn neutral_element() -> Self::Transformation;

    /// Concat two transformations
    fn concat(first: &Self::Transformation, second: &Self::Transformation) -> Self::Transformation;
}

/// Helper function to be used in [std::iter::Scan] for accumulating transformations from direct path from a root to a node
pub fn accumulate<'a, S, T>(
    stack: &mut Vec<T::Transformation>,
    arg: (&'a S, &T::Parameter),
) -> Option<(&'a S, T::Transformation)>
where
    S: Nodelike<T>,
    T: Rigid,
{
    // TODO not expected to work (pop is not even called). Will need the depth in order to know how much to pop
    let (node, param) = arg;
    let current = T::concat(
        stack.last().unwrap_or(&T::neutral_element()),
        &node.get().transformation(param),
    );
    stack.push(current.clone());
    Some((node, current))
}

///  Trait that adds an `accumulate` functions for accumulating transformations from direct path from a root to a node.
/// Wraps a zip and scan operation
///
/// **Warning** currently borken! See unit tests in this file
pub trait Accumulator<'a, N, T>
where
    T: Rigid,
    N: Nodelike<T> + 'a,
{
    fn accumulate(self, param: &[T::Parameter], max_depth: usize) -> impl Iterator<Item = (&'a N, T::Transformation)>;
}

impl<'a, 'b, S, T> Accumulator<'a, S, T> for Box<dyn Iterator<Item = &'a S> + 'b>
where
    S: Nodelike<T>,
    T: Rigid,
{
    fn accumulate(
        self,
        param: &[T::Parameter],
        max_depth: usize,
    ) -> impl Iterator<Item = (&'a S, <T as Rigid>::Transformation)> {
        self.into_iter()
            .zip(param.iter())
            // .scan(Vec::<T::Transformation>::with_capacity(max_depth), |a, (b, c)| None)
            .scan(Vec::<T::Transformation>::with_capacity(max_depth), accumulate)
    }
}

#[cfg(test)]
mod tests {
    use itertools::Itertools;

    use crate::dummy::DummyBody;
    use crate::*;

    #[test]
    fn smoke_test() {
        let root = [0usize];
        {
            let a = ArenaTree::<i32>::new(DepthFirst);
            let mut i = a.iter(DepthFirst, &root);
            i.next();

            let neutral = 0;
            i.scan(Vec::<i32>::with_capacity(42), |stack, node| {
                let current = *stack.last().unwrap_or(&neutral) + node.get();
                stack.push(current);
                Some(current)
            })
            .enumerate()
            .for_each(|(idx, el)| println!("Accumulated {el} for node {idx}"));
        }
        #[cfg(not(feature = "accumulate"))]
        {
            let tree = ArenaTree::<DummyBody>::new(DepthFirst);
            let param = &[1.0, 2.0, 3.0];
            let zip = tree.iter(DepthFirst, &[]).zip(param.iter());
            zip.scan(
                Vec::<<DummyBody as Rigid>::Transformation>::with_capacity(42),
                accumulate,
            )
            .collect_vec();
        }
        #[cfg(feature = "accumulate")]
        {
            let tree = ArenaTree::<DummyBody>::new(DepthFirst);
            let param = &[1.0, 2.0, 3.0];
            let iter = tree.iter(DepthFirst, &[]);
            iter.accumulate(param, 32).collect_vec();
        }
    }
}
