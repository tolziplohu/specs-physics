use crate::{nalgebra::RealField, nphysics::math::Isometry};

use shrinkwraprs::Shrinkwrap;
use specs::{Component, DenseVecStorage};

#[cfg(feature = "serialization")]
use serde::{Deserialize, Serialize};

/// An implementation of the `Position` trait is required for the
/// synchronisation of the position of Specs and nphysics objects.
///
/// Initially, it is used to position bodies in the nphysics `World`. Then after
/// progressing the `World` it is used to synchronise the updated positions back
/// towards Specs.
pub trait Position<N: RealField>: Component + Send + Sync {
    fn isometry(&self) -> &Isometry<N>;
    fn isometry_mut(&mut self) -> &mut Isometry<N>;
    fn set_isometry(&mut self, isometry: &Isometry<N>) -> &mut Self;
}

#[cfg(feature = "amethyst")]
impl Position<f32> for amethyst_core::Transform {
    fn isometry(&self) -> &Isometry<f32> {
        self.isometry()
    }

    fn isometry_mut(&mut self) -> &mut Isometry<f32> {
        self.isometry_mut()
    }

    fn set_isometry(&mut self, isometry: &Isometry<f32>) -> &mut Self {
        self.set_isometry(*isometry)
    }
}

#[cfg_attr(feature = "serialization", derive(Serialize, Deserialize))]
#[derive(Component, Shrinkwrap, Clone, Debug, PartialEq)]
#[shrinkwrap(mutable)]
pub struct SimplePosition<N: RealField>(pub Isometry<N>);

impl<N: RealField> Position<N> for SimplePosition<N> {
    fn isometry(&self) -> &Isometry<N> {
        &self.0
    }

    fn isometry_mut(&mut self) -> &mut Isometry<N> {
        &mut self.0
    }

    fn set_isometry(&mut self, isometry: &Isometry<N>) -> &mut Self {
        self.0.rotation = isometry.rotation;
        self.0.translation = isometry.translation;
        self
    }
}
