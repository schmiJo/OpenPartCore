//! Hash tables with a fixed seed, so that the same input always gives the same bits.
//!
//! The standard tables pick a random seed per process, and any loop over one then runs in
//! a different order each time. Here that changed volumes in the last digit and triangle
//! counts by a few, which makes a failure hard to reproduce. These tables have no defence
//! against keys chosen to collide, which doesn't matter for geometry.

use std::hash::{BuildHasherDefault, Hasher};

pub type HashMap<K, V> = std::collections::HashMap<K, V, BuildHasherDefault<FixedHasher>>;
pub type HashSet<K> = std::collections::HashSet<K, BuildHasherDefault<FixedHasher>>;

const K: u64 = 0x9E37_79B9_7F4A_7C15;

#[derive(Clone, Copy, Default)]
pub struct FixedHasher(u64);

impl FixedHasher {
    #[inline]
    fn add(&mut self, word: u64) {
        self.0 = (self.0.rotate_left(5) ^ word).wrapping_mul(K);
    }
}

impl Hasher for FixedHasher {
    fn write(&mut self, bytes: &[u8]) {
        for chunk in bytes.chunks(8) {
            let mut word = [0u8; 8];
            word[..chunk.len()].copy_from_slice(chunk);
            self.add(u64::from_le_bytes(word));
        }
    }

    #[inline]
    fn write_u8(&mut self, v: u8) {
        self.add(v as u64);
    }

    #[inline]
    fn write_u32(&mut self, v: u32) {
        self.add(v as u64);
    }

    #[inline]
    fn write_u64(&mut self, v: u64) {
        self.add(v);
    }

    #[inline]
    fn write_i64(&mut self, v: i64) {
        self.add(v as u64);
    }

    #[inline]
    fn write_usize(&mut self, v: usize) {
        self.add(v as u64);
    }

    /// Floating-point bit patterns often end in zeros, and a table indexes by low bits, so
    /// the result is mixed once more.
    #[inline]
    fn finish(&self) -> u64 {
        let h = (self.0 ^ (self.0 >> 32)).wrapping_mul(K);
        h ^ (h >> 29)
    }
}
