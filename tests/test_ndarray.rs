#![cfg(feature = "ndarray")]

use mannequin::ndarray::{
    basic::{Axis, Bone, LinkNodeId},
    Jacobian,
};
use mannequin::{ArenaTree, DepthFirst, Differentiable, Rigid, TreeIterable};
use ndarray::prelude::*;

#[test]
fn test_jacobian() {
    // let ik = DifferentialIK { max_depth: 10 };
    let mut tree = ArenaTree::<Bone, LinkNodeId>::new();

    let mut trafo = Bone::neutral_element();
    trafo.slice_mut(s![..3, 3]).assign(&array![10.0, 0.0, 0.0]);

    let link1 = Bone::new(&trafo, Axis::RotationZ);
    let link2 = Bone::new(&trafo, Axis::RotationZ);
    let link3 = Bone::new(&trafo, Axis::RotationZ);
    let link4 = Bone::new(&trafo, Axis::RotationZ);

    // TODO .. can we make the refs fix in a way they don't get optimized away?
    // Then these could be strings even!

    let ref1 = tree.set_root(link1, "link1".to_string());
    let _ref2 = tree.add(link2, "link2".to_string(), &ref1).unwrap();
    let ref3 = tree.add(link3, "link3".to_string(), &ref1).unwrap();
    let _ref4 = tree.add(link4, "link4".to_string(), &ref3).unwrap();

    tree.optimize(DepthFirst);

    let mut jacobian = Jacobian::new();
    jacobian.setup(
        &tree,
        &[
            "link1".to_string(),
            "link2".to_string(),
            "link3".to_string(),
            "link4".to_string(),
        ]
        .into(),
        &["link2".to_string(), "link4".to_string()].into(),
    );

    jacobian.compute(&tree, &[0.0, 0.0, std::f64::consts::FRAC_PI_2, 0.0]);

    // let jacobian = ik.jacobian(&tree, array![0.0, 0.0, std::f64::consts::FRAC_PI_2, 0.0]);

    println!("jacobian: {}", jacobian.jacobian());
}
