use crate::{
    nalgebra::RealField,
    nphysics::math::{Force, Point},
};

use super::{AngularInertiaType, CombinedInertia, Motion};

use shrinkwraprs::Shrinkwrap;
use specs::{Component, DenseVecStorage};

#[cfg(feature = "serialization")]
use serde::{Deserialize, Serialize};

pub use position::{Position, SimplePosition};
pub use status::{ActivationStatus, BodyStatus, BodyUpdateStatus};

pub mod internal;
mod position;
mod status;

/// Velocity component.
///
/// # Size
/// `dim3`: 6 x `RealField`
/// `dim2`: 3 x `RealField`
#[cfg_attr(feature = "serialization", derive(Serialize, Deserialize))]
#[derive(Component, Shrinkwrap, Clone, Debug, Default, PartialEq)]
#[shrinkwrap(mutable)]
pub struct Velocity<N: RealField>(pub Motion<N>);

/// Acceleration component.
///
/// # Size
/// `dim3`: 6 x `RealField`
/// `dim2`: 3 x `RealField`
#[cfg_attr(feature = "serialization", derive(Serialize, Deserialize))]
#[derive(Component, Shrinkwrap, Clone, Debug, Default, PartialEq)]
#[shrinkwrap(mutable)]
pub struct Acceleration<N: RealField>(pub Motion<N>);

/// Force component.
///
/// # Size
/// `dim3`: 6 x `RealField`
/// `dim2`: 3 x `RealField`
#[cfg_attr(feature = "serialization", derive(Serialize, Deserialize))]
#[derive(Component, Shrinkwrap, Clone, Debug, Default, PartialEq)]
#[shrinkwrap(mutable)]
pub struct ExternalForces<N: RealField>(pub Motion<N>);

impl<N: RealField> Into<Force<N>> for ExternalForces<N> {
    fn into(self) -> Force<N> {
        Force::new(self.linear, self.angular)
    }
}

/// Component for a body's linear inertia in that body's local space
///
/// # Size
/// 1 x `RealField`
#[cfg_attr(feature = "serialization", derive(Serialize, Deserialize))]
#[derive(Component, Shrinkwrap, Clone, Debug, Default, PartialEq)]
#[shrinkwrap(mutable)]
pub struct Mass<N: RealField>(pub N);

/// Component for a body's center of inertia expressed in that body's local
/// space
///
/// # Size
/// `dim3`: 3 x `RealField`
/// `dim2`: 2 x `RealField`
#[cfg_attr(feature = "serialization", derive(Serialize, Deserialize))]
#[derive(Component, Shrinkwrap, Clone, Debug, PartialEq)]
#[shrinkwrap(mutable)]
pub struct CenterOfMass<N: RealField>(pub Point<N>);

impl<N: RealField> Default for CenterOfMass<N> {
    #[inline]
    fn default() -> Self {
        CenterOfMass(Point::origin())
    }
}

/// Component for a body's angular mass in that body's local space
///
/// # Size
/// `dim3`: 9 x `RealField`
/// `dim2`: 1 x `RealField`
#[cfg_attr(feature = "serialization", derive(Serialize, Deserialize))]
#[derive(Component, Shrinkwrap, Clone, Debug, PartialEq)]
#[shrinkwrap(mutable)]
pub struct AngularInertia<N: RealField>(pub AngularInertiaType<N>);

#[cfg(feature = "dim3")]
impl<N: RealField> Default for AngularInertia<N> {
    #[inline]
    fn default() -> Self {
        AngularInertia(AngularInertiaType::identity())
    }
}

#[cfg(feature = "dim2")]
impl<N: RealField> Default for AngularInertia<N> {
    #[inline]
    fn default() -> Self {
        AngularInertia(N::one())
    }
}
