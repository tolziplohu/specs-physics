use crate::{nalgebra::RealField, nphysics::math::SpatialVector};

use bitflags::bitflags;
use specs::{Component, DenseVecStorage};

#[cfg(feature = "serialization")]
use serde::{Deserialize, Serialize};

/// Component that describes a rigid body's characteristics.
///
/// # Size
/// 4 x `RealField` + `usize` + 1 byte
#[cfg_attr(feature = "serialization", derive(Serialize, Deserialize))]
#[derive(Component, Debug, Clone, PartialEq)]
pub struct RigidBodyProperties<N: RealField> {
    /// Associated flags for this body
    pub flags: RigidBodyFlags,

    /// Linear damping/drag coefficient
    pub damping: N,
    /// Angular damping/drag coefficient
    pub damping_angular: N,

    /// Maximum allowed linear velocity
    pub max_velocity: N,
    /// Maximum allowed angular velocity
    pub max_velocity_angular: N,

    // Used for things.
    pub(crate) companion_id: usize,
}

impl<N: RealField> Default for RigidBodyProperties<N> {
    #[inline]
    fn default() -> Self {
        RigidBodyProperties {
            flags: RigidBodyFlags::default(),
            damping: N::zero(),
            damping_angular: N::zero(),
            max_velocity: N::max_value(),
            max_velocity_angular: N::max_value(),
            companion_id: 0,
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
    pub struct RigidBodyFlags: u8 {
        /// Whether the linear X axis is a kinematic degree of freedom
        const KINEMATIC_TRANSLATION_X      = 0b10000000;

        /// Whether the linear Y axis is a kinematic degree of freedom
        const KINEMATIC_TRANSLATION_Y      = 0b01000000;

        /// Whether the linear Z axis is a kinematic degree of freedom
        /// Ignored when the `dim2` feature is set.
        const KINEMATIC_TRANSLATION_Z      = 0b00100000;

        /// Whether the angular X axis is a kinematic degree of freedom
        const KINEMATIC_ROTATION_X         = 0b00010000;

        /// Whether the angular Y axis is a kinematic degree of freedom
        /// Ignored when the `dim2` feature is set.
        const KINEMATIC_ROTATION_Y         = 0b00001000;

        /// Whether the angular Z axis is a kinematic degree of freedom
        /// Ignored when the `dim2` feature is set.
        const KINEMATIC_ROTATION_Z         = 0b00000100;

        /// Enables the universal gravitational force for this Body.
        const GRAVITY_ENABLED              = 0b00000010;

        /// Enables linear motion interpolation for CCD
        const LINEAR_INTERPOLATION_ENABLED = 0b00000001;
    }
}

impl Default for RigidBodyFlags {
    #[inline]
    fn default() -> Self {
        RigidBodyFlags::GRAVITY_ENABLED
    }
}

#[cfg(feature = "dim3")]
impl<N: RealField> Into<SpatialVector<N>> for RigidBodyFlags {
    fn into(self) -> SpatialVector<N> {
        SpatialVector::new(
            if self.contains(RigidBodyFlags::KINEMATIC_TRANSLATION_X) {
                N::one()
            } else {
                N::zero()
            },
            if self.contains(RigidBodyFlags::KINEMATIC_TRANSLATION_Y) {
                N::one()
            } else {
                N::zero()
            },
            if self.contains(RigidBodyFlags::KINEMATIC_TRANSLATION_Z) {
                N::one()
            } else {
                N::zero()
            },
            if self.contains(RigidBodyFlags::KINEMATIC_ROTATION_X) {
                N::one()
            } else {
                N::zero()
            },
            if self.contains(RigidBodyFlags::KINEMATIC_ROTATION_Y) {
                N::one()
            } else {
                N::zero()
            },
            if self.contains(RigidBodyFlags::KINEMATIC_ROTATION_Z) {
                N::one()
            } else {
                N::zero()
            },
        )
    }
}

#[cfg(feature = "dim2")]
impl<N: RealField> Into<SpatialVector<N>> for RigidBodyFlags {
    fn into(self) -> SpatialVector<N> {
        SpatialVector::new(
            if self.contains(RigidBodyFlags::KINEMATIC_TRANSLATION_X) {
                N::one()
            } else {
                N::zero()
            },
            if self.contains(RigidBodyFlags::KINEMATIC_TRANSLATION_Y) {
                N::one()
            } else {
                N::zero()
            },
            if self.contains(RigidBodyFlags::KINEMATIC_ROTATION_X) {
                N::one()
            } else {
                N::zero()
            },
        )
    }
}
