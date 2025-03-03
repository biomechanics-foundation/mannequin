//! "Default" imnplementation of a kinematics as encountered in robotics.
#![allow(unused_variables)]

use super::{
    cross_3d, invert_tranformation_4x4, rotate_x_4x4, rotate_y_4x4, rotate_z_4x4, translate_x_4x4, translate_y_4x4,
    translate_z_4x4,
};
use crate::Rigid;
use core::fmt;
use ndarray::prelude::*;
use ndarray::{Array1, Array2};

#[derive(Debug, PartialEq, Default)]
pub enum Axis {
    RotationX,
    RotationY,
    #[default]
    RotationZ,
    Rotation(Array1<f64>),
    TranslationX,
    TranslationY,
    TranslationZ,
    Translation(Array1<f64>),
}

#[derive(Debug, Default, PartialEq)]
pub enum Mode {
    #[default]
    Position,
    Pose,
}

#[derive(Debug, Default, PartialEq)]
pub struct Bone {
    link: Array2<f64>,
    axis: Axis,
    mode: Mode,
    effector: Option<Array2<f64>>,
}

impl Bone {
    pub fn new(from_parent: &Array2<f64>, axis: Axis, effector: Option<Array2<f64>>) -> Self {
        Self {
            link: from_parent.clone(),
            axis,
            mode: Mode::Position,
            effector,
        }
    }
}

impl fmt::Display for Bone {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "Bone, link: {}, Axis: {:?}", self.link, self.axis)
    }
}

impl Rigid for Bone {
    type Transformation = Array2<f64>;

    type Point = Array1<f64>;

    type Parameter = f64;

    type NodeId = String;

    fn transform(&self, param: &Self::Parameter) -> Self::Transformation {
        let joint = match self.axis {
            Axis::RotationX => rotate_x_4x4(*param),
            Axis::RotationY => rotate_y_4x4(*param),
            Axis::RotationZ => rotate_z_4x4(*param),
            // TODO implement arbitratry axis and translations
            Axis::Rotation(_) => todo!(),
            Axis::TranslationX => translate_x_4x4(*param),
            Axis::TranslationY => translate_y_4x4(*param),
            Axis::TranslationZ => translate_z_4x4(*param),
            Axis::Translation(_) => todo!(),
        };
        self.link.dot(&joint)
    }

    fn globalize(&self, other: &Self::Point) -> Self::Point {
        self.link.dot(other)
    }

    fn localize(&self, other: &Self::Point) -> Self::Point {
        invert_tranformation_4x4(&self.link).dot(other)
    }

    fn neutral_element() -> Self::Transformation {
        Array2::<f64>::eye(4)
    }

    fn concat(first: &Self::Transformation, second: &Self::Transformation) -> Self::Transformation {
        first.dot(second)
    }

    fn invert(trafo: &Self::Transformation) -> Self::Transformation {
        invert_tranformation_4x4(trafo)
    }

    fn dim(&self) -> usize {
        match self.mode {
            Mode::Position => 3,
            Mode::Pose => unimplemented!(), //6,
        }
    }

    fn effector_count(&self) -> usize {
        if self.effector.is_some() {
            1
        } else {
            0
        }
    }

    fn partial_derivative(
        &self,
        pose: &Self::Transformation,
        joint: &Self,
        joint_pose: &Self::Transformation,
        target_buffer: &mut [f64],
    ) {
        // Formula: axis_in_world x (end_effector_world - pivod_in_world)
        let local_axis = match &joint.axis {
            Axis::RotationX => &array![1.0, 0.0, 0.0, 0.0],
            Axis::RotationY => &array![0.0, 1.0, 0.0, 0.0],
            Axis::RotationZ => &array![0.0, 0.0, 1.0, 0.0],
            Axis::Rotation(array_base) => array_base,
            Axis::TranslationX => unimplemented!(),
            Axis::TranslationY => unimplemented!(),
            Axis::TranslationZ => unimplemented!(),
            Axis::Translation(array_base) => unimplemented!(),
        };
        let axis_global = joint_pose.dot(local_axis);

        // TODO can we avoid this clone?
        let mut pose = pose.clone();
        if let Some(effector) = &self.effector {
            pose = pose.dot(effector);
        }
        let lever = &pose.slice(s![0..3, 3]) - &joint_pose.slice(s![0..3, 3]);

        cross_3d::<Self::NodeId>(
            axis_global.slice(s![0..3]),
            lever.view(),
            ArrayViewMut1::from(target_buffer),
        )
        .unwrap();
    }
}

// TODO move solvers to dedicated module

pub type LinkNodeId = <Bone as Rigid>::NodeId;

pub struct DifferentialIK {
    #[allow(dead_code)]
    max_depth: usize,
}

// // TODO: Keep generic
// pub type DifferentialIKParameter =
//     <DifferentialIK as Inverse<DirectedArenaTree<Bone, LinkNodeId>, Bone, ForwardsKinematics>>::Parameter;
// pub type DifferentialIKArray =
//     <DifferentialIK as Inverse<DirectedArenaTree<Bone, LinkNodeId>, Bone, ForwardsKinematics>>::Array;

// impl Inverse<DepthFirstArenaTree<Bone, LinkNodeId>, Bone, ForwardsKinematics> for DifferentialIK {
//     type Parameter = Array1<f64>;

//     type Array = Array2<f64>;

//     fn solve(
//         &mut self,
//         tree: &DepthFirstArenaTree<Bone, LinkNodeId>,
//         fk: &ForwardsKinematics,
//         param: Self::Parameter,
//         target_refs: &[LinkNodeId],
//         target_val: &[Self::Array],
//     ) -> Self::Parameter {
//         todo!()
//     }
// }

// pub type BasicMannequin = Mannequin<DirectedArenaTree<Bone, LinkNodeId>, Bone, ForwardsKinematics, DifferentialIK>;

#[cfg(test)]
mod tests {
    use super::super::Jacobian;
    use super::{Axis, Bone, DifferentialIK, LinkNodeId};
    use crate::{DepthFirstArenaTree, DirectedArenaTree, DirectionIterable, Forward, Rigid};
    use crate::{Differentiable, ForwardsKinematics};
    use approx::assert_abs_diff_eq;
    use itertools::Itertools;
    use ndarray::prelude::*;

    #[test]
    fn test_fk() {
        let mut fk = ForwardsKinematics::<Bone>::new(10);
        let mut tree = DirectedArenaTree::<Bone, LinkNodeId>::new();

        let mut trafo = Bone::neutral_element();
        trafo.slice_mut(s![..3, 3]).assign(&array![10.0, 0.0, 0.0]);

        let link1 = Bone::new(&trafo, Axis::RotationZ, None);
        let link2 = Bone::new(&trafo, Axis::RotationZ, None);
        let link3 = Bone::new(&trafo, Axis::RotationZ, None);
        let link4 = Bone::new(&trafo, Axis::RotationZ, None);

        // TODO .. can we make the refs fix in a way they don't get optimized away?
        // Then these could be strings even!
        let ref1 = tree.set_root(link1, "link1".to_string());
        let ref2 = tree.add(link2, "link2".to_string(), &ref1).unwrap();
        let ref3 = tree.add(link3, "link3".to_string(), &ref1).unwrap();
        let ref4 = tree.add(link4, "link4".to_string(), &ref3).unwrap();

        let tree: DepthFirstArenaTree<_, _> = tree.into();

        fk.initialize(&tree, &[ref2, ref3, ref4]);
        let res = fk.solve(&tree, &[0.0, 0.0, std::f64::consts::FRAC_PI_2, 0.0]);
        let res = res.iter().map(|el| el.slice(s![..3, 3]).to_vec()).collect_vec();

        assert_eq!(
            res,
            vec![vec![20.0, 0.0, 0.0], vec![20.0, 0.0, 0.0], vec![20.0, 10.0, 0.0]]
        );
    }

    #[test]
    fn test_jacobian() {
        let ik = DifferentialIK { max_depth: 10 };
        let mut tree = DirectedArenaTree::<Bone, LinkNodeId>::new();

        let mut trafo = Bone::neutral_element();
        trafo.slice_mut(s![..3, 3]).assign(&array![10.0, 0.0, 0.0]);

        let link1 = Bone::new(&trafo, Axis::RotationZ, None);
        let link2 = Bone::new(&trafo, Axis::RotationZ, Some(trafo.clone()));
        let link3 = Bone::new(&trafo, Axis::RotationZ, None);
        let link4 = Bone::new(&trafo, Axis::RotationZ, Some(trafo.clone()));

        // TODO .. can we make the refs fix in a way they don't get optimized away?
        // Then these could be strings even!

        let ref1 = tree.set_root(link1, "link1".to_string());
        let ref2 = tree.add(link2, "link2".to_string(), &ref1).unwrap();
        let ref3 = tree.add(link3, "link3".to_string(), &ref1).unwrap();
        let ref4 = tree.add(link4, "link4".to_string(), &ref3).unwrap();

        let tree: DepthFirstArenaTree<_, _> = tree.into();

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
            // TODO automatically those that have an effector defined
            &["link2".to_string(), "link4".to_string()].into(),
        );

        jacobian.compute(
            &tree,
            &[0.0, 0.0, std::f64::consts::FRAC_PI_2, std::f64::consts::FRAC_PI_2],
        );

        let target = array![
            [0.0, 0.0, 0.0, 0.0],
            [20.0, 10.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0,],
            [-10.0, 0.0, -10.0, 0.0],
            [0.0, 0.0, -10.0, -10.0],
            [0.0, 0.0, 0.0, 0.0,]
        ];

        // println!("{}", &jacobian.jacobian() - &target);

        assert_abs_diff_eq!(jacobian.jacobian(), target, epsilon = 1e-6);
    }
}
