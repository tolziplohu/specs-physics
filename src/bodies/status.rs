use crate::{
    nalgebra::{convert as na_convert, RealField},
    nphysics::object::{
        ActivationStatus as NActivationStatus,
        BodyStatus as NBodyStatus,
        BodyUpdateStatus as NBodyUpdateStatus,
    },
};

#[derive(Copy, Clone, Debug, PartialEq)]
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

#[derive(Copy, Clone, PartialEq, Eq, Hash, Debug)]
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

#[derive(Copy, Clone, PartialEq, Eq, Hash, Debug)]
pub struct BodyUpdateStatus {
    pub position_changed: bool,
    pub velocity_changed: bool,
    pub inertia_changed: bool,
    pub center_of_mass_changed: bool,
    pub damping_changed: bool,
    pub status_changed: bool,
}

impl BodyUpdateStatus {
    #[inline]
    pub fn inertia_needs_update(&self) -> bool {
        self.position_changed
            || self.velocity_changed
            || self.inertia_changed
            || self.center_of_mass_changed
            || self.damping_changed
            || self.status_changed
    }

    #[inline]
    pub fn colliders_need_update(&self) -> bool {
        self.position_changed
    }

    #[inline]
    pub fn clear(&mut self) {
        self.position_changed = false;
        self.velocity_changed = false;
        self.inertia_changed = false;
        self.center_of_mass_changed = false;
        self.damping_changed = false;
        self.status_changed = false;
    }
}

impl Into<NBodyUpdateStatus> for BodyUpdateStatus {
    #[inline]
    fn into(self) -> NBodyUpdateStatus {
        let mut status = NBodyUpdateStatus::empty();
        status.set_position_changed(self.position_changed);
        status.set_velocity_changed(self.velocity_changed);
        status.set_local_inertia_changed(self.inertia_changed);
        status.set_local_com_changed(self.center_of_mass_changed);
        status.set_damping_changed(self.damping_changed);
        status.set_status_changed(self.status_changed);
        status
    }
}
