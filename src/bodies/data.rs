use crate::{
    math::PhysicalDimension,
    nalgebra::{
        Isometry as NalgebraIsometry,
        RealField,
        UnitComplex,
        UnitQuaternion,
        Vector2,
        Vector3,
        Vector6,
        VectorN,
        U2,
        U3,
    },
    nphysics::math::{Inertia, Isometry as NphysicsIsometry, Vector},
};

//use super::components::{AngularInertia, Mass};

use std::{mem::transmute as mem_transmute, ops::Mul};

#[cfg(feature = "serialization")]
use serde::{Deserialize, Serialize};

//#[cfg(feature = "dim3")]
//pub type AngularMotion<N> = AngularMotion3<N>;
//#[cfg(feature = "dim2")]
//pub type AngularMotion<N> = AngularMotion2<N>;

/// Describes a motion, or difference, in space.
/// Such as a Velocity, an Acceleration, or a Force.
///
/// # Size
/// `dim3`: 6 x `RealField`
/// `dim2`: 3 x `RealField`
#[cfg_attr(feature = "serialization", derive(Serialize, Deserialize))]
#[derive(Clone, Debug, PartialEq)]
pub struct Motion<D: PhysicalDimension> {
    pub linear: D::Vector,
    pub angular: D::AngularMotion,
}
/*

#[cfg(feature = "dim3")]
impl<D: PhysicalDimension> Default for Motion<N> {
    #[inline]
    fn default() -> Self {
        Motion {
            linear: Vector::zeros(),
            angular: Vector::zeros(),
        }
    }
}

#[cfg(feature = "dim2")]
impl<D: PhysicalDimension> Default for Motion<N> {
    #[inline]
    fn default() -> Self {
        Motion {
            linear: Vector::zeros(),
            angular: N::zero(),
        }
    }
}
*/
impl<D: PhysicalDimension> Mul<D::Float> for Motion<D>
where
    D::Float: RealField,
{
    type Output = Self;

    #[inline]
    fn mul(self, rhs: D::Float) -> Self::Output {
        Motion {
            linear: self.linear * rhs,
            angular: self.angular * rhs,
        }
    }
}

impl<D: PhysicalDimension<Dimension = U2>> Motion<D>
where
    D::Float: RealField,
{
    #[inline]
    pub fn zero(&mut self) {
        self.linear = D::Vector::zeros();
        self.angular = D::AngularMotion::zero();
    }

    #[inline]
    pub(crate) fn as_vector(&self) -> &Vector3<D::Float> {
        unsafe { mem_transmute(self) }
    }

    #[inline]
    pub(crate) fn as_vector_mut(&mut self) -> &mut Vector3<D::Float> {
        unsafe { mem_transmute(self) }
    }

    #[inline]
    pub(crate) fn from_slice(data: &[D::Float]) -> Self {
        Motion {
            linear: D::Vector::new(data[0], data[1]),
            angular: data[2],
        }
    }
}
/*
impl<D: PhysicalDimension<Dimension = U3>> Motion<D> {
    #[inline]
    pub fn zero(&mut self) {
        self.linear = D::Vector::zeros();
        self.angular = D::AngularMotion::zeros();
    }

    #[inline]
    pub(crate) fn as_vector(&self) -> &Vector6<D::Float> {
        unsafe { mem_transmute(self) }
    }

    #[inline]
    pub(crate) fn as_vector_mut(&mut self) -> &mut Vector6<D::Float> {
        unsafe { mem_transmute(self) }
    }

    #[inline]
    pub(crate) fn from_slice(data: &[D::Float]) -> Self {
        Motion {
            linear: D::Vector::new(data[0], data[1], data[2]),
            angular: Vector::new(data[3], data[4], data[5]),
        }
    }
}*/

impl<D: PhysicalDimension> Motion<D> {
    #[inline]
    pub(crate) fn as_slice(&self) -> &[D::Float] {
        self.as_vector().as_slice()
    }

    #[inline]
    pub(crate) fn as_mut_slice(&mut self) -> &mut [D::Float] {
        self.as_vector_mut().as_mut_slice()
    }
}
/*

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
*/
