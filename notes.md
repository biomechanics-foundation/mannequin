# Notes

<!-- todo move to lib.rs -->

## Conventions

* Where applicable, use an adjective describing an ability for traits in addition to the common
  convention of using CamelCase notation. Example: `Tree Iterable`
* Structs are regular substantives describing objects or entities in CamelCase notation
* Functions and method names use imperative phrases in snake_case notation. Example: `derive`, `transform_child`
* Getters and setters methods are exceptions to the rule and omit the `get_*` and are not imperatives phrases
  as regular methods and functions. Examples: `axes`/`axes_mut` instead of `get_axes`/`get_axes_mut`.
* If there are multiple generic parameters, we use capitalized names similar to struct names, even
  if it becomes more difficult to distinguish them. For one or two generics, single capital Letters may be used.

## Dead ends

* Define a bone as a rigid transformation and multiple axes.
  * This would offer a more concise interface to joints with multiple DoF
  * However, at the price of a more convoluted iteration and overall complexity
  * Nodes would have to reserve space for up to 6DoF
  * A simpler solution is to have 2D joint identifiers of type `(String,usize)` which can be hashed and
    offers the same flexibility
  * A joint that uses 2D spline interpolation would be more difficult and hacky to implement
    but could be realized as a tuple of nodes with similar data (duplication) and changing the parameter type
    from `&[f64]` to `&[(f64,f64])` such that both nodes know each other's parameter. Regular 1D axes would
    ignore the second parameter, or one would use an `enum` (e.g., `Independent(f64)` and `Coupled((f64,f64))`).
    As this is an unlikely requirement in the future, this workaround seems acceptable.

## Glossary and synonyms

* **Rigid Body**, *Rigid*, *Bone*: Often called `Bone` in CG and game development. The trait is called `Rigid`
  and implementing structs can use `bone` for instance.
  In the `mannequin` crate, it consists of a single transformation from a parent to the ~~axis/axes~~ *single* axis.
* **Articulation**, *Articulated*, *Joint*: A ...
* **Transformation**, *frame of reference*, *(local) coordinate system*: A combination of a rotation
  and subsequent translation. Describes the transformation
  from one coordinate system or frame of reference to another. For themselves, they describe a frame of
  reference or local coordinate system in relation to the world coordinate system. Often expressed as
  homogeneous 4x4 matrices or dual quaternions.
