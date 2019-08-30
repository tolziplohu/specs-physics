use crate::{
    nalgebra::RealField,
    nphysics::math::{Force, Point},
    position::Position,
};

use shrinkwraprs::Shrinkwrap;
use specs::{prelude::*, storage::UnprotectedStorage, Component};
use std::{any::Any, marker::PhantomData};

#[cfg(feature = "serialization")]
use serde::{Deserialize, Serialize};

pub use base::{AngularInertiaType, CombinedInertia, Motion};
pub use rigid::RigidBodyProperties;
pub use status::{ActivationStatus, BodyStatus, BodyUpdateStatus};

mod base;
mod cross;
mod rigid;
mod status;

/// Position used in stepping process.
///
/// # Size
/// Dynamic with `P`
#[cfg_attr(feature = "serialization", derive(Serialize, Deserialize))]
#[derive(Component, Shrinkwrap, Clone, Debug, Default, PartialEq)]
#[shrinkwrap(mutable)]
pub struct AdvancePosition<
    N: RealField,
    P: Position<N, Storage = PS>,
    PS: UnprotectedStorage<P> + Any + Send + Sync,
>(#[shrinkwrap(main_field)] pub P, PhantomData<N>);

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

/// Component for a body's center of inertia expressed in global space.
///
/// # Size
/// `dim3`: 3 x `RealField`
/// `dim2`: 2 x `RealField`
#[cfg_attr(feature = "serialization", derive(Serialize, Deserialize))]
#[derive(Component, Shrinkwrap, Clone, Debug, PartialEq)]
#[shrinkwrap(mutable)]
pub struct GlobalCenterOfMass<N: RealField>(pub Point<N>);

impl<N: RealField> Default for GlobalCenterOfMass<N> {
    #[inline]
    fn default() -> Self {
        GlobalCenterOfMass(Point::origin())
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
        AngularInertia(AngularInertiaType::zeros())
    }
}

#[cfg(feature = "dim2")]
impl<N: RealField> Default for AngularInertia<N> {
    #[inline]
    fn default() -> Self {
        AngularInertia(AngularInertiaType::zero())
    }
}

/// Component for a body's combined linear and angular inertia in global space
///
/// # Size
/// `dim3`: 10 x `RealField`
/// `dim2`: 2 x `RealField`
#[cfg_attr(feature = "serialization", derive(Serialize, Deserialize))]
#[derive(Component, Shrinkwrap, Clone, Debug, Default, PartialEq)]
#[shrinkwrap(mutable)]
pub struct GlobalInertia<N: RealField>(pub CombinedInertia<N>);

/// Component for a body's combined linear and angular inertia augmented in
/// global space
///
/// # Size
/// `dim3`: 10 x `RealField`
/// `dim2`: 2 x `RealField`
#[cfg_attr(feature = "serialization", derive(Serialize, Deserialize))]
#[derive(Component, Shrinkwrap, Clone, Debug, Default, PartialEq)]
#[shrinkwrap(mutable)]
pub struct AugmentedInertia<N: RealField>(pub CombinedInertia<N>);

/// Component for a body's combined linear and angular inertia augmented and
/// inverted in global space
///
/// # Size
/// `dim3`: 10 x `RealField`
/// `dim2`: 2 x `RealField`
#[cfg_attr(feature = "serialization", derive(Serialize, Deserialize))]
#[derive(Component, Shrinkwrap, Clone, Debug, Default, PartialEq)]
#[shrinkwrap(mutable)]
pub struct InvertedAugmentedInertia<N: RealField>(pub CombinedInertia<N>);
