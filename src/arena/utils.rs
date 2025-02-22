//! Utility function(s).

use super::directed::ArenaIndex;

/// Sorts a vector/slice by a vector of indices in O(n)
/// Found on [stackoverflow](https://stackoverflow.com/a/69774341)
pub fn sort_by_indices<T>(data: &mut [T], mut indices: Vec<ArenaIndex>) {
    for idx in 0..data.len() {
        if indices[idx].0 != idx {
            let mut current_idx = idx;
            loop {
                let target_idx = indices[current_idx];
                indices[current_idx] = ArenaIndex(current_idx);
                if indices[target_idx.0] == target_idx {
                    break;
                }
                data.swap(current_idx, target_idx.0);
                current_idx = target_idx.0;
            }
        }
    }
}
