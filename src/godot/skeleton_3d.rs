//! Adds depth-first iteration to [godot::classes::Skeleton3D]

use godot::classes::Skeleton3D;
use godot::prelude::*;

/// Iterate over the bones of a [Skeleton3D] depth-first.
pub trait BoneIteratable {
    /// Depth-first iteration over bones of a skeleton
    fn iter_bones(&self) -> BoneIterator;
}

/// Iterator over the bones of a [Skeleton3D] depth-first.
pub struct BoneIterator<'a> {
    stack: Vec<std::vec::IntoIter<i32>>,
    skeleton: &'a Skeleton3D,
}

impl Iterator for BoneIterator<'_> {
    /// The bone's transformation, its index and its optional parent index
    type Item = (Transform3D, i32, Option<i32>);

    fn next(&mut self) -> Option<Self::Item> {
        if let Some(last) = self.stack.last_mut() {
            let bone = last.next();

            if let Some(bone) = bone {
                let children = self.skeleton.get_bone_children(bone).to_vec();
                self.stack.push(children.into_iter());
                let parent = self.skeleton.get_bone_parent(bone);
                let parent = if parent != -1 { Some(parent) } else { None };
                Some((self.skeleton.get_bone_pose(bone), bone, parent))
            } else {
                self.stack.pop();
                self.next()
            }
        } else {
            None
        }
    }
}

impl BoneIteratable for Skeleton3D {
    fn iter_bones(&self) -> BoneIterator {
        let roots = self.get_parentless_bones().to_vec();
        BoneIterator {
            skeleton: self,
            stack: vec![roots.into_iter()],
        }
    }
}
