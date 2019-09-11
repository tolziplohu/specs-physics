use crate::{nalgebra::RealField, nphysics::math::Point};

use super::{CombinedInertia, Position};

use shrinkwraprs::Shrinkwrap;
use specs::{storage::UnprotectedStorage, Component, DenseVecStorage};
use std::{any::Any, marker::PhantomData};

#[cfg(feature = "serialization")]
use serde::{Deserialize, Serialize};

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
>(#[shrinkwrap(main_field)] pub P, pub PhantomData<N>);

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
