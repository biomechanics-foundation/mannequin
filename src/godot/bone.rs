//! A representation of a Bone of [godot::classes::Skeleton3D] in Godot as a [Rigid] body.

use crate::Rigid;
use godot::prelude::*;

#[derive(Debug, Default, PartialEq)]
pub struct GodotBone {
    from_parent: Transform3D,
    axis: Vector3,
    pub orientation: bool,
    pub effector: bool,
}

impl GodotBone {
    pub fn new(from_parent: Transform3D, axis: Vector3, orientation: bool) -> Self {
        Self {
            from_parent,
            effector: false,
            axis,
            orientation,
        }
    }
}

impl Rigid for GodotBone {
    type Transformation = Transform3D;

    type Point = Vector3;

    type FloatType = f32;

    type NodeId = i32;

    fn transform(&self, params: &[Self::FloatType], index: usize) -> Self::Transformation {
        self.from_parent.rotated_local(Vector3::BACK, params[index])
    }

    fn globalize(&self, other: &Self::Point) -> Self::Point {
        self.from_parent.origin + (self.from_parent.basis * *other)
    }

    fn localize(&self, other: &Self::Point) -> Self::Point {
        let _ = other;
        todo!()
    }

    fn dim(&self) -> usize {
        if self.orientation { 6 } else { 3 }
    }

    fn partial_derivative(
        &self,
        pose: &Self::Transformation,
        joint: &Self,
        joint_pose: &Self::Transformation,
        target_buffer: &mut [Self::FloatType],
        offset: usize,
    ) {
        let _ = joint;
        // Assume only z-axis for robots
        let axis_global = joint_pose.translated_local(self.axis).origin;
        let lever = pose.origin - joint_pose.origin;
        let target_buffer = &mut target_buffer[offset..offset + self.effector_size()];
        let result = axis_global.cross(lever);
        target_buffer[0] = result.x;
        target_buffer[1] = result.y;
        target_buffer[2] = result.z;

        if self.orientation {
            target_buffer[3] = axis_global.x;
            target_buffer[4] = axis_global.y;
            target_buffer[5] = axis_global.z;
        }
    }

    fn effector(&self, pose: &Self::Transformation, target_buffer: &mut [Self::FloatType], offset: usize) {
        let target_buffer = &mut target_buffer[offset..offset + self.effector_size()];
        target_buffer[0] = pose.origin.x;
        target_buffer[1] = pose.origin.y;
        target_buffer[2] = pose.origin.z;

        // we get the orientation from godot
    }

    fn effector_count(&self) -> usize {
        if self.effector { 1 } else { 0 }
    }

    fn neutral_element() -> Self::Transformation {
        Transform3D::IDENTITY
    }

    fn invert(trafo: &Self::Transformation) -> Self::Transformation {
        trafo.affine_inverse()
    }

    fn concat(first: &Self::Transformation, second: &Self::Transformation) -> Self::Transformation {
        *first * *second
    }

    fn solve_linear(
        matrix: &[Self::FloatType],
        rows: usize,
        cols: usize,
        vector: &[Self::FloatType],
        parameters: &mut [Self::FloatType],
    ) {
        let _ = parameters;
        let _ = vector;
        let _ = cols;
        let _ = rows;
        let _ = matrix;
        unimplemented!()
    }
}
