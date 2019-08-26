use crate::{
    nalgebra::{RealField, Vector2, Matrix2, Vector1, RowVector2, Matrix3, Vector3, },
    nphysics::math::{Inertia, Isometry, Point, Vector},
};

use std::{mem::transmute as mem_transmute, ops::Mul};

#[cfg(feature = "serialization")]
use serde::{Deserialize, Serialize};

pub use rigid::RigidBodyProperties;
pub use status::{ActivationStatus, BodyStatus, BodyUpdateStatus};

mod rigid;
mod status;

#[cfg(feature = "dim3")]
type AsVector<N> = crate::nalgebra::Vector6<N>;
#[cfg(feature = "dim2")]
type AsVector<N> = crate::nalgebra::Vector3<N>;

/// Angular inertia for selected physics dimension.
/// This will be a 3x3 Matrix in 3d, and just a single Float in 2d.
#[cfg(feature = "dim3")]
pub type AngularInertia<N> = crate::nalgebra::Matrix3<N>;

/// Angular inertia for selected physics dimension.
/// This will be a 3x3 Matrix in 3d, and just a single Float in 2d.
#[cfg(feature = "dim2")]
pub type AngularInertia = N;

#[cfg(feature = "dim3")]
pub type AngularForce<N> = crate::nalgebra::Vector3<N>;

#[cfg(feature = "dim2")]
pub type AngularForce<N> = N;

/// Velocity component. When coupled with a Position, a Mass, and
/// RigidBodyProperties, the linked entity is then designated a rigid body.
#[cfg_attr(feature = "serialization", derive(Serialize, Deserialize))]
#[derive(Clone, Debug, PartialEq)]
pub struct Velocity<N: RealField> {
    pub linear: Vector<N>,
    pub angular: Vector<N>,
}

impl<N: RealField> Default for Velocity<N> {
    #[inline]
    fn default() -> Self {
        Velocity {
            linear: Vector::zeros(),
            angular: Vector::zeros(),
        }
    }
}

impl<N: RealField> Mul<N> for Velocity<N> {
    type Output = Self;

    #[inline]
    fn mul(self, rhs: N) -> Self {
        Velocity {
            linear: self.linear * rhs,
            angular: self.angular * rhs,
        }
    }
}

impl<N: RealField> Velocity<N> {
    #[inline]
    pub fn zero(&mut self) {
        self.linear = Vector::repeat(N::zero());
        self.angular = Vector::repeat(N::zero());
    }

    #[inline]
    pub(crate) fn as_slice(&self) -> &[N] {
        self.as_vector().as_slice()
    }

    #[inline]
    pub(crate) fn as_vector(&self) -> &AsVector<N> {
        unsafe { mem_transmute(self) }
    }

    #[inline]
    pub(crate) fn as_mut_slice(&mut self) -> &mut [N] {
        self.as_vector_mut().as_mut_slice()
    }

    #[inline]
    pub(crate) fn as_vector_mut(&mut self) -> &mut AsVector<N> {
        unsafe { mem_transmute(self) }
    }
}

/// Velocity component. When coupled with a Position, a Mass, and
/// RigidBodyProperties, the linked entity is then designated a rigid body.
#[cfg_attr(feature = "serialization", derive(Serialize, Deserialize))]
#[derive(Clone, Debug, PartialEq)]
pub struct Acceleration<N: RealField> {
    pub linear: Vector<N>,
    pub angular: Vector<N>,
}

impl<N: RealField> Default for Acceleration<N> {
    #[inline]
    fn default() -> Self {
        Acceleration {
            linear: Vector::zeros(),
            angular: Vector::zeros(),
        }
    }
}

impl<N: RealField> Acceleration<N> {
    #[inline]
    pub fn zero(&mut self) {
        self.linear = Vector::repeat(N::zero());
        self.angular = Vector::repeat(N::zero());
    }

    #[inline]
    pub(crate) fn as_slice(&self) -> &[N] {
        self.as_vector().as_slice()
    }

    #[inline]
    pub(crate) fn as_vector(&self) -> &AsVector<N> {
        unsafe { mem_transmute(self) }
    }

    #[inline]
    pub(crate) fn as_mut_slice(&mut self) -> &mut [N] {
        self.as_vector_mut().as_mut_slice()
    }

    #[inline]
    pub(crate) fn as_vector_mut(&mut self) -> &mut AsVector<N> {
        unsafe { mem_transmute(self) }
    }
}

#[cfg_attr(feature = "serialization", derive(Serialize, Deserialize))]
#[derive(Clone, Debug, PartialEq)]
pub struct ExternalForces<N: RealField> {
    pub linear: Vector<N>,
    pub angular: AngularForce<N>,
}

#[cfg(feature = "dim3")]
impl<N: RealField> ExternalForces<N> {
    pub fn zero(&mut self) {
        self.linear = Vector::zeros();
        self.angular = AngularForce::zeros();
    }
}

#[cfg(feature = "dim2")]
impl<N: RealField> ExternalForces<N> {
    pub fn zero(&mut self) {
        self.linear = Vector::zeros();
        self.angular = AngularForce::zero();
    }
}

#[cfg(feature = "dim3")]
impl<N: RealField> Default for ExternalForces<N> {
    fn default() -> Self {
        ExternalForces {
            linear: Vector::zeros(),
            angular: AngularForce::zeros(),
        }
    }
}

#[cfg(feature = "dim2")]
impl<N: RealField> Default for ExternalForces<N> {
    fn default() -> Self {
        ExternalForces {
            linear: Vector::zeros(),
            angular: AngularForce::zero(),
        }
    }
}

#[cfg_attr(feature = "serialization", derive(Serialize, Deserialize))]
#[derive(Clone, Debug, PartialEq)]
pub struct Mass<N: RealField> {
    /// Center of mass expressed in local space
    pub center: Point<N>,
    /// Linear inertia (mass) in local space
    pub linear: N,
    /// Angular inertia in local space
    pub angular: AngularInertia<N>,
}

#[cfg(feature = "dim3")]
impl<N: RealField> Mass<N> {
    fn transformed(&self, isometry: Isometry<N>) -> Inertia<N> {
        let rot = isometry.rotation.to_rotation_matrix();
        Inertia::new(self.linear, rot * self.angular * rot.inverse())
    }
}

#[cfg(feature = "dim2")]
impl<N: RealField> Mass<N> {
    fn transformed(&self, isometry: Isometry<N>) -> Inertia<N> {
        Inertia::new(self.linear, self.angular)
    }
}

#[cfg(feature = "dim3")]
impl<N: RealField> Default for Mass<N> {
    #[inline]
    fn default() -> Self {
        Mass {
            center: Point::origin(),
            linear: N::zero(),
            angular: AngularInertia::zeros(),
        }
    }
}

#[cfg(feature = "dim2")]
impl<N: RealField> Default for Mass<N> {
    #[inline]
    fn default() -> Self {
        Mass {
            center: Point::origin(),
            linear: N::zero(),
            angular: AngularInertia::zero(),
        }
    }
}

#[cfg_attr(feature = "serialization", derive(Serialize, Deserialize))]
#[derive(Clone, Debug, PartialEq)]
pub struct GlobalInertia<N: RealField> {
    pub linear: N,
    pub angular: AngularInertia<N>,
}

#[cfg_attr(feature = "serialization", derive(Serialize, Deserialize))]
#[derive(Clone, Debug, PartialEq)]
pub struct AugmentedInertia<N: RealField> {
    pub linear: N,
    pub angular: AngularInertia<N>,
}

#[cfg_attr(feature = "serialization", derive(Serialize, Deserialize))]
#[derive(Clone, Debug, PartialEq)]
pub struct AugmentedInertiaInv<N: RealField> {
    pub linear: N,
    pub angular: AngularInertia<N>,
}

/// This is a non-standard generalization of the cross product design exclusively to group the
/// 3D cross product and the 2D perpendicular product behind the same interface.
pub(crate) trait GeneralizedCross {
    /// The right-hand-side of this cross product.
    type Rhs;
    /// The result type of the this (non-standard) generalized cross product.
    type CrossVector;
    /// The matrix representation of this (non-standard) generalized cross product.
    type CrossMatrix;
    /// The transposed matrix representation of this (non-standard) generalized cross product.
    type CrossMatrixTr;

    /// Computes this (non-standard) generalized cross product.
    fn gcross(&self, rhs: &Self::Rhs) -> Self::CrossVector;

    /// Computes the matrix represenattion of this (non-standard) generalized cross product.
    fn gcross_matrix(&self) -> Self::CrossMatrix;

    /// Computes the transposed matrix represenattion of this (non-standard) generalized cross product.
    fn gcross_matrix_tr(&self) -> Self::CrossMatrixTr;
}

impl<N: RealField> GeneralizedCross for Vector1<N> {
    type Rhs = Vector2<N>;
    type CrossVector = Vector2<N>;
    type CrossMatrix = Matrix2<N>;
    type CrossMatrixTr = Matrix2<N>;

    #[inline]
    fn gcross(&self, rhs: &Vector2<N>) -> Vector2<N> {
        Vector2::new(-rhs.y * self.x, rhs.x * self.x)
    }

    #[inline]
    fn gcross_matrix(&self) -> Matrix2<N> {
        Matrix2::new(N::zero(), -self.x, self.x, N::zero())
    }

    #[inline]
    fn gcross_matrix_tr(&self) -> Matrix2<N> {
        Matrix2::new(N::zero(), self.x, -self.x, N::zero())
    }
}

impl<N: RealField> GeneralizedCross for Vector2<N> {
    type Rhs = Vector2<N>;
    type CrossVector = Vector1<N>;
    type CrossMatrix = RowVector2<N>;
    type CrossMatrixTr = Vector2<N>;

    #[inline]
    fn gcross(&self, rhs: &Vector2<N>) -> Vector1<N> {
        Vector1::new(self.x * rhs.y - self.y * rhs.x)
    }

    #[inline]
    fn gcross_matrix(&self) -> RowVector2<N> {
        RowVector2::new(-self.y, self.x)
    }

    #[inline]
    fn gcross_matrix_tr(&self) -> Vector2<N> {
        Vector2::new(-self.y, self.x)
    }
}

impl<N: RealField> GeneralizedCross for Vector3<N> {
    type Rhs = Vector3<N>;
    type CrossVector = Vector3<N>;
    type CrossMatrix = Matrix3<N>;
    type CrossMatrixTr = Matrix3<N>;

    #[inline]
    fn gcross(&self, rhs: &Vector3<N>) -> Vector3<N> {
        self.cross(rhs)
    }

    #[inline]
    fn gcross_matrix(&self) -> Matrix3<N> {
        self.cross_matrix()
    }

    #[inline]
    fn gcross_matrix_tr(&self) -> Matrix3<N> {
        Matrix3::new(
            N::zero(),
            self.z,
            -self.y,
            -self.z,
            N::zero(),
            self.x,
            self.y,
            -self.x,
            N::zero(),
        )
    }
}