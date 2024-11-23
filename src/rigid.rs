use crate::Node;
pub trait Body {
    // E.g., 4x4 matrix, (3x1, 3x3), quaternions ...
    type Transformation: Clone;
    // Vec, [f64;4], ...
    type Point;
    // typically joint positions (angles/extension), f64, [f64,3]
    type Parameter;

    fn transformation(&self, param: &Self::Parameter) -> Self::Transformation;
    fn derivative(&self, param: Self::Parameter) -> Self::Transformation;
    fn neutral_element() -> Self::Transformation;
    fn n_dof(&self) -> i32;

    fn concat(first: &Self::Transformation, second: &Self::Transformation) -> Self::Transformation;
    fn globalize(&self, other: &Self::Point) -> Self::Point;
    fn localize(&self, other: &Self::Point) -> Self::Point;
}

pub fn acc<S, T>(stack: &mut Vec<T::Transformation>, arg: (&S, &T::Parameter)) -> Option<T::Transformation>
where
    S: Node<T>,
    T: Body,
{
    let (node, param) = arg;
    let current = T::concat(
        stack.last().unwrap_or(&T::neutral_element()),
        &node.get().transformation(param),
    );
    stack.push(current.clone());
    Some(current)
}

// make a trait that has a fn `accumulate` that returns a Scan iterator (or an impl Iter<item = T::Transform>)
// implement the trait for Box<dyn Iterator<Item = S>>

pub trait Accumulator<T>
where
    T: Body,
{
    fn accumulate(self, param: &[T::Parameter], max_depth: usize) -> impl Iterator<Item = T::Transformation>;
}

impl<'a, S, T> Accumulator<T> for Box<dyn Iterator<Item = &'a S>>
where
    S: Node<T>,
    T: Body,
{
    fn accumulate(self, param: &[T::Parameter], max_depth: usize) -> impl Iterator<Item = <T as Body>::Transformation> {
        self.into_iter()
            .zip(param.iter())
            // .scan(Vec::<T::Transformation>::with_capacity(max_depth), |a, (b, c)| None)
            .scan(Vec::<T::Transformation>::with_capacity(max_depth), acc)
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
            zip.scan(Vec::<<DummyBody as Body>::Transformation>::with_capacity(42), acc)
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
