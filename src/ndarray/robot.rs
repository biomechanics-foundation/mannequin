//! "Default" imnplementation of a kinematics as encountered in robotics.
#![allow(unused_variables)]

use super::{
    cross_3d, invert_transformation_4x4, rotate_x_4x4, rotate_y_4x4, rotate_z_4x4, solve_linear, translate_x_4x4,
    translate_y_4x4, translate_z_4x4,
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
pub struct Segment {
    link: Array2<f64>,
    axis: Axis,
    mode: Mode,
    effector_local: Option<Array2<f64>>,
}

impl Segment {
    pub fn new(from_parent: &Array2<f64>, axis: Axis, effector: Option<Array2<f64>>) -> Self {
        Self {
            link: from_parent.clone(),
            axis,
            mode: Mode::Position,
            effector_local: effector,
        }
    }
}

impl fmt::Display for Segment {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "Bone, link: {}, Axis: {:?}", self.link, self.axis)
    }
}

impl Rigid for Segment {
    type Transformation = Array2<f64>;

    type Point = Array1<f64>;

    type FloatType = f64;

    type NodeId = String;

    fn transform(&self, params: &[f64], index: usize) -> Self::Transformation {
        let joint = match self.axis {
            Axis::RotationX => rotate_x_4x4(params[index]),
            Axis::RotationY => rotate_y_4x4(params[index]),
            Axis::RotationZ => rotate_z_4x4(params[index]),
            // TODO implement arbitrary axis and translations
            Axis::Rotation(_) => todo!(),
            Axis::TranslationX => translate_x_4x4(params[index]),
            Axis::TranslationY => translate_y_4x4(params[index]),
            Axis::TranslationZ => translate_z_4x4(params[index]),
            Axis::Translation(_) => todo!(),
        };
        self.link.dot(&joint)
    }

    fn globalize(&self, other: &Self::Point) -> Self::Point {
        self.link.dot(other)
    }

    fn localize(&self, other: &Self::Point) -> Self::Point {
        invert_transformation_4x4(&self.link).dot(other)
    }

    fn neutral_element() -> Self::Transformation {
        Array2::<f64>::eye(4)
    }

    fn concat(first: &Self::Transformation, second: &Self::Transformation) -> Self::Transformation {
        first.dot(second)
    }

    fn invert(trafo: &Self::Transformation) -> Self::Transformation {
        invert_transformation_4x4(trafo)
    }

    fn dim(&self) -> usize {
        match self.mode {
            Mode::Position => 3,
            Mode::Pose => unimplemented!(), //6,
        }
    }

    fn effector_count(&self) -> usize {
        if self.effector_local.is_some() {
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
        buffer: &mut [f64],
        offset: usize,
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
        if let Some(effector) = &self.effector_local {
            pose = pose.dot(effector);
        }
        let lever = &pose.slice(s![0..3, 3]) - &joint_pose.slice(s![0..3, 3]);

        let target_buffer = &mut buffer[offset..offset + self.effector_size()];
        cross_3d::<Self::NodeId>(
            axis_global.slice(s![0..3]),
            lever.view(),
            ArrayViewMut1::from(target_buffer),
        )
        .unwrap();
    }

    /// Get the coordinates of the effenctor in the global (or an arbitatry) system.
    fn effector(&self, pose: &Self::Transformation, buffer: &mut [f64], offset: usize) {
        dbg!(&buffer, offset, self.effector_size());
        let target_buffer = &mut buffer[offset..offset + self.effector_size()];
        let mut target = ArrayViewMut1::from(target_buffer);

        if let Some(effector) = &self.effector_local {
            target.assign(&(pose.dot(effector)).slice(s![0..3, 3]));
        } else {
            panic!("Should not call this method if no effector is defined")
        }
    }

    fn solve_linear(matrix: &[f64], rows: usize, cols: usize, vector: &[f64], target_buffer: &mut [f64]) {
        solve_linear(matrix, rows, cols, vector, target_buffer);
    }
}

// TODO move solvers to dedicated module

pub type LinkNodeId = <Segment as Rigid>::NodeId;

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
mod tests {}
