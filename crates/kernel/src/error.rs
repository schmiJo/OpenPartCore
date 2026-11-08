//! Kernel errors. Each one says what went wrong in terms a caller can show.

use std::fmt;

#[derive(Clone, Debug, PartialEq)]
pub enum Error {
    /// A 2D profile is open, self-intersecting or otherwise unusable.
    InvalidProfile(String),
    /// An argument is out of range.
    InvalidInput(String),
    /// A body failed the validity checks. Holds one line per defect.
    InvalidBody(Vec<String>),
    /// The input is too close to a degenerate case to decide reliably.
    NearDegenerate(String),
    /// A valid request that this kernel does not implement.
    Unsupported(String),
    /// An internal consistency check failed. This is a kernel bug or limit.
    Internal(String),
}

pub type Result<T> = std::result::Result<T, Error>;

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Error::InvalidProfile(s) => write!(f, "invalid profile: {s}"),
            Error::InvalidInput(s) => write!(f, "invalid input: {s}"),
            Error::InvalidBody(v) => write!(f, "invalid body: {}", v.join("; ")),
            Error::NearDegenerate(s) => write!(f, "near-degenerate input: {s}"),
            Error::Unsupported(s) => write!(f, "unsupported: {s}"),
            Error::Internal(s) => write!(f, "internal error: {s}"),
        }
    }
}

impl std::error::Error for Error {}
