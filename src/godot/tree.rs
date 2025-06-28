//! Generate a kinematic tree from a [godot::classes::Skeleton3D]

use super::BoneIteratable;
use crate::{DepthFirstArenaTree, DirectedArenaTree, DirectionIterable, godot::GodotBone};
use godot::classes::Skeleton3D;
use godot::prelude::*;

pub type GodotTree = DepthFirstArenaTree<GodotBone, i32>;

impl From<Gd<Skeleton3D>> for GodotTree {
    fn from(skeleton: Gd<Skeleton3D>) -> Self {
        let mut tree = DirectedArenaTree::new();
        let root_bones = skeleton.get_parentless_bones();
        if root_bones.len() != 1 {
            godot_error!(
                "Currently we only support a single root / parentless bone ({} found)",
                root_bones.len()
            );
        } else {
            skeleton.iter_bones().for_each(|(trafo, bone, parent)| {
                let axis = skeleton
                    .get_bone_meta(bone, "axis")
                    .try_to::<Vector3>()
                    .unwrap_or(Vector3::BACK);

                godot_print!("Processing bone `{}`, axis: {}", skeleton.get_bone_name(bone), axis);
                if let Some(parent) = parent {
                    tree.add(GodotBone::new(trafo, axis, false), bone, &parent).unwrap();
                } else {
                    tree.set_root(GodotBone::new(trafo, axis, false), bone);
                }
            });
        }

        tree.into()
    }
}
