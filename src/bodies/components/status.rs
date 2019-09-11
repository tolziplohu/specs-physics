use crate::{
    nalgebra::{convert as na_convert, RealField},
    nphysics::object::{
        ActivationStatus as NActivationStatus,
        BodyStatus as NBodyStatus,
        BodyUpdateStatus as NBodyUpdateStatus,
    },
};

use bitflags::bitflags;
use specs::{Component, DenseVecStorage};

#[cfg(feature = "serialization")]
use serde::{Deserialize, Serialize};

/// Component with energy used to determine if a Body is awake.
///
/// # Size
/// 1 x `RealField` + optional `RealField`
#[cfg_attr(feature = "serialization", derive(Serialize, Deserialize))]
#[derive(Component, Copy, Clone, Debug, PartialEq)]
pub struct ActivationStatus<N: RealField> {
    pub threshold: Option<N>,
    pub energy: N,
}

impl<N: RealField> ActivationStatus<N> {
    #[inline]
    pub fn default_threshold() -> N {
        na_convert(0.01f64)
    }

    #[inline]
    pub fn new_active() -> Self {
        ActivationStatus {
            threshold: Some(Self::default_threshold()),
            energy: Self::default_threshold() * na_convert(4.0),
        }
    }

    /// Create a new activation status initialised with the default activation
    /// threshold and is inactive.
    #[inline]
    pub fn new_inactive() -> Self {
        ActivationStatus {
            threshold: Some(Self::default_threshold()),
            energy: N::zero(),
        }
    }

    /// Retuns `true` if the body is not asleep.
    #[inline]
    pub fn is_active(&self) -> bool {
        !self.energy.is_zero()
    }
}

impl<N: RealField> Default for ActivationStatus<N> {
    #[inline]
    fn default() -> Self {
        Self::new_active()
    }
}

impl<N: RealField> Into<NActivationStatus<N>> for ActivationStatus<N> {
    #[inline]
    fn into(self) -> NActivationStatus<N> {
        let mut status = NActivationStatus::new_inactive();
        status.set_deactivation_threshold(self.threshold);
        status.set_energy(self.energy);
        status
    }
}

/// Component designating the simulation status for a Body.
/// TODO: This could perhaps be a marker component?
///
/// # Size
/// Probably a byte but maybe four :)
#[cfg_attr(feature = "serialization", derive(Serialize, Deserialize))]
#[derive(Component, Copy, Clone, PartialEq, Eq, Hash, Debug)]
pub enum BodyStatus {
    /// The body is disabled and ignored by the physics engine.
    Disabled,
    /// The body is static and thus cannot move.
    Static,
    /// The body is dynamic and thus can move and is subject to forces.
    Dynamic,
    /// The body is kinematic so its velocity is controlled by the user and it
    /// is not affected by forces and constraints.
    Kinematic,
}

impl Default for BodyStatus {
    #[inline]
    fn default() -> Self {
        BodyStatus::Dynamic
    }
}

impl Into<NBodyStatus> for BodyStatus {
    #[inline]
    fn into(self) -> NBodyStatus {
        match self {
            BodyStatus::Disabled => NBodyStatus::Disabled,
            BodyStatus::Static => NBodyStatus::Static,
            BodyStatus::Dynamic => NBodyStatus::Dynamic,
            BodyStatus::Kinematic => NBodyStatus::Kinematic,
        }
    }
}

bitflags! {
    /// Component tracking changes for a body. This will be sticky business...
    ///
    /// # Size
    /// 1 Byte
    #[cfg_attr(feature = "serialization", derive(Serialize, Deserialize))]
    #[derive(Component)]
    pub struct BodyUpdateStatus: u8 {
        const POSITION_CHANGED = 0b00000001;
        const VELOCITY_CHANGED = 0b00000010;
        const INERTIA_CHANGED = 0b000100;
        const CENTER_OF_MASS_CHANGED = 0b001000;
        const DAMPING_CHANGED = 0b010000;
        const STATUS_CHANGED = 0b100000;
    }
}

impl BodyUpdateStatus {
    #[inline]
    pub fn inertia_needs_update(&self) -> bool {
        self.contains(BodyUpdateStatus::POSITION_CHANGED)
            || self.contains(BodyUpdateStatus::VELOCITY_CHANGED)
            || self.contains(BodyUpdateStatus::INERTIA_CHANGED)
            || self.contains(BodyUpdateStatus::CENTER_OF_MASS_CHANGED)
            || self.contains(BodyUpdateStatus::DAMPING_CHANGED)
            || self.contains(BodyUpdateStatus::STATUS_CHANGED)
    }

    #[inline]
    pub fn colliders_need_update(&self) -> bool {
        self.contains(BodyUpdateStatus::POSITION_CHANGED)
    }

    #[inline]
    pub fn clear(&mut self) {
        self.remove(BodyUpdateStatus::all());
    }
}

impl Into<NBodyUpdateStatus> for BodyUpdateStatus {
    #[inline]
    fn into(self) -> NBodyUpdateStatus {
        let mut status = NBodyUpdateStatus::empty();
        status.set_position_changed(self.contains(BodyUpdateStatus::POSITION_CHANGED));
        status.set_velocity_changed(self.contains(BodyUpdateStatus::VELOCITY_CHANGED));
        status.set_local_inertia_changed(self.contains(BodyUpdateStatus::INERTIA_CHANGED));
        status.set_local_com_changed(self.contains(BodyUpdateStatus::CENTER_OF_MASS_CHANGED));
        status.set_damping_changed(self.contains(BodyUpdateStatus::DAMPING_CHANGED));
        status.set_status_changed(self.contains(BodyUpdateStatus::STATUS_CHANGED));
        status
    }
}
