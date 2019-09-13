use crate::{
    nalgebra::{
        Isometry as NalgebraIsometry,
        RealField,
        UnitComplex,
        UnitQuaternion,
        Vector2,
        Vector3,
        U2,
        U3,
    },
    nphysics::math::{Inertia, Isometry as NphysicsIsometry, Vector},
};

use super::components::{AngularInertia, Mass};

use std::{mem::transmute as mem_transmute, ops::Mul};

#[cfg(feature = "serialization")]
use serde::{Deserialize, Serialize};

#[cfg(feature = "dim3")]
pub type AngularMotion<N> = AngularMotion3<N>;
#[cfg(feature = "dim2")]
pub type AngularMotion<N> = AngularMotion2<N>;

/// Describes a motion, or difference, in space.
/// Such as a Velocity, an Acceleration, or a Force.
///
/// # Size
/// `dim3`: 6 x `RealField`
/// `dim2`: 3 x `RealField`
#[cfg_attr(feature = "serialization", derive(Serialize, Deserialize))]
#[derive(Clone, Debug, PartialEq)]
pub struct Motion<N: RealField> {
    pub linear: Vector<N>,
    pub angular: AngularMotion<N>,
}

#[cfg(feature = "dim3")]
impl<N: RealField> Default for Motion<N> {
    #[inline]
    fn default() -> Self {
        Motion {
            linear: Vector::zeros(),
            angular: Vector::zeros(),
        }
    }
}

#[cfg(feature = "dim2")]
impl<N: RealField> Default for Motion<N> {
    #[inline]
    fn default() -> Self {
        Motion {
            linear: Vector::zeros(),
            angular: N::zero(),
        }
    }
}

impl<N: RealField> Mul<N> for Motion<N> {
    type Output = Self;

    #[inline]
    fn mul(self, rhs: N) -> Self::Output {
        Motion {
            linear: self.linear * rhs,
            angular: self.angular * rhs,
        }
    }
}

#[cfg(feature = "dim3")]
pub(crate) type MotionAsVector<N> = crate::nalgebra::Vector6<N>;

#[cfg(feature = "dim2")]
pub(crate) type MotionAsVector<N> = crate::nalgebra::Vector3<N>;

impl<N: RealField> Motion<N> {
    #[inline]
    #[cfg(feature = "dim3")]
    pub fn zero(&mut self) {
        self.linear = Vector::zeros();
        self.angular = Vector::zeros();
    }

    #[inline]
    #[cfg(feature = "dim2")]
    pub fn zero(&mut self) {
        self.linear = Vector::zeros();
        self.angular = N::zero();
    }

    #[inline]
    pub(crate) fn as_slice(&self) -> &[N] {
        self.as_vector().as_slice()
    }

    #[inline]
    pub(crate) fn as_vector(&self) -> &MotionAsVector<N> {
        unsafe { mem_transmute(self) }
    }

    #[inline]
    pub(crate) fn as_mut_slice(&mut self) -> &mut [N] {
        self.as_vector_mut().as_mut_slice()
    }

    #[inline]
    pub(crate) fn as_vector_mut(&mut self) -> &mut MotionAsVector<N> {
        unsafe { mem_transmute(self) }
    }

    #[inline]
    #[cfg(feature = "dim3")]
    pub(crate) fn from_slice(data: &[N]) -> Self {
        Motion {
            linear: Vector::new(data[0], data[1], data[2]),
            angular: Vector::new(data[3], data[4], data[5]),
        }
    }

    #[inline]
    #[cfg(feature = "dim2")]
    pub(crate) fn from_slice(data: &[N]) -> Self {
        Motion {
            linear: Vector::new(data[0], data[1]),
            angular: data[2],
        }
    }
}

#[cfg(feature = "dim3")]
pub type AngularInertiaType<N> = crate::nalgebra::Matrix3<N>;

#[cfg(feature = "dim2")]
pub type AngularInertiaType<N> = N;

/// Struct combining linear and angular inertia
///
/// # Size
/// `dim3`: 10 x `RealField`
/// `dim2`: 2 x `RealField`
#[cfg_attr(feature = "serialization", derive(Serialize, Deserialize))]
#[derive(Clone, Debug, PartialEq)]
pub struct CombinedInertia<N: RealField> {
    pub linear: N,
    pub angular: AngularInertiaType<N>,
}

#[cfg(feature = "dim3")]
impl<N: RealField> Default for CombinedInertia<N> {
    #[inline]
    fn default() -> Self {
        CombinedInertia {
            linear: N::one(),
            angular: AngularInertiaType::identity(),
        }
    }
}

#[cfg(feature = "dim2")]
impl<N: RealField> Default for CombinedInertia<N> {
    #[inline]
    fn default() -> Self {
        CombinedInertia {
            linear: N::one(),
            angular: N::one(),
        }
    }
}

impl<N: RealField> From<(Box<Mass<N>>, Box<AngularInertia<N>>)> for CombinedInertia<N> {
    fn from(separate: (Box<Mass<N>>, Box<AngularInertia<N>>)) -> CombinedInertia<N> {
        CombinedInertia {
            linear: **separate.0,
            angular: **separate.1,
        }
    }
}

impl<N: RealField> Into<Inertia<N>> for CombinedInertia<N> {
    fn into(self) -> Inertia<N> {
        Inertia::new(self.linear, self.angular)
    }
}

impl<N: RealField> CombinedInertia<N> {
    #[inline]
    #[cfg(feature = "dim3")]
    pub(crate) fn transformed(&self, isometry: NphysicsIsometry<N>) -> Inertia<N> {
        let rot = isometry.rotation.to_rotation_matrix();
        Inertia::new(self.linear, rot * self.angular * rot.inverse())
    }

    #[inline]
    #[cfg(feature = "dim2")]
    pub(crate) fn transformed(&self, _isometry: NphysicsIsometry<N>) -> Inertia<N> {
        Inertia::new(self.linear, self.angular)
    }

    #[inline]
    #[cfg(feature = "dim3")]
    pub(crate) fn inverse(&self) -> Inertia<N> {
        let inv_mass = if self.linear.is_zero() {
            N::zero()
        } else {
            N::one() / self.linear
        };
        let inv_angular = self
            .angular
            .try_inverse()
            .unwrap_or_else(|| AngularInertiaType::zeros());
        Inertia::new(inv_mass, inv_angular)
    }

    #[inline]
    #[cfg(feature = "dim2")]
    pub(crate) fn inverse(&self) -> Inertia<N> {
        let inv_mass = if self.linear.is_zero() {
            N::zero()
        } else {
            N::one() / self.linear
        };
        let inv_angular = if self.angular.is_zero() {
            N::zero()
        } else {
            N::one() / self.angular
        };
        Inertia::new(inv_mass, inv_angular)
    }
}
