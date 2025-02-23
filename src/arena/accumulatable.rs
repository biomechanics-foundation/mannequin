// TODO REMOVE
// scan is not so bad, we could save `depth` on the `NodeLike` interface though with this file
// Not worth it, keep it with the old implementation of `iter` for a blog post
// The interface in rigid is good enough!

use super::Nodelike;

// TODO restrict for DepthFrist, but is requires more trait conditions

/// Accumulation over a *Depth First* iteration (this is not checked).
pub trait DepthFirstAccumulatable<'a, Node, Load, F, Parameter, Accumulated, NodeId>
where
    Node: Nodelike<Load, NodeId> + 'a,
    F: Fn(&Node, &Parameter, &Accumulated) -> Accumulated + 'static,
    Accumulated: Clone,
{
    /// Method on an iterator. Accepts a slice of parameters used to
    /// compute a value that can be accumulated considering the depth of the tree.
    /// The latter means that each node receives the accumulated values from its parent.
    /// The calculation of the value and the accumulation is specified by the `accumulator`
    /// parameter (usually, a closure). Accumulation requires a `first` element (neutral element)
    /// and a max depth (for performance)
    fn accumulate(
        self,
        param: &[Parameter],
        accumulator: F,
        first: Accumulated,
        max_depth: usize,
    ) -> impl Iterator<Item = (&'a Node, Accumulated)>;
}

impl<'a, 'b, Node, Load, F, Parameter, Accumulated, NodeId>
    DepthFirstAccumulatable<'a, Node, Load, F, Parameter, Accumulated, NodeId>
    for Box<dyn Iterator<Item = &'a Node> + 'b>
where
    Node: Nodelike<Load, NodeId> + 'a,
    F: Fn(&Node, &Parameter, &Accumulated) -> Accumulated + 'static,
    Accumulated: Clone,
{
    fn accumulate(
        self,
        param: &[Parameter],
        acc: F,
        first: Accumulated,
        max_depth: usize,
    ) -> impl Iterator<Item = (&'a Node, Accumulated)> {
        self.into_iter()
            .zip(param.iter())
            // TODO look up whether the move here (required for the acc parameter) slows down execution
            .scan(
                Vec::<Accumulated>::with_capacity(max_depth),
                move |stack, (node, param)| {
                    let depth = stack.len();
                    while node.depth() < depth {
                        stack.pop();
                    }
                    let result = acc(node, param, stack.last().unwrap_or(&first));
                    stack.push(result.clone());
                    Some((node, result))
                },
            )
    }
}
