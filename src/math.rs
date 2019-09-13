use crate::nalgebra::{
    Dim,
    DimName,
    IsNotStaticOne,
    Isometry as NalgebraIsometry,
    NamedDim,
    Point,
    RealField,
    UnitComplex,
    UnitQuaternion,
    Vector3,
    U1,
    U2,
    U3,
};

use alga::linear::Rotation;
use generic_array::ArrayLength;

use std::{marker::PhantomData, ops::Mul};

use shamanic_tome::DimensionalSealSpell;

mod shamanic_tome {
    pub trait DimensionalSealSpell {}
}

pub trait PhysicalDimension: DimensionalSealSpell {
    type Float: RealField;
    type Dimension: Dim + DimName<Value = Self::DimensionValue> + IsNotStaticOne;
    type DimensionValue: NamedDim<Name = Self::Dimension>
        + Mul<<U1 as DimName>::Value, Output = Self::InternalDimProd>;
    type InternalDimProd: ArrayLength<Self::Float>;
    type Rotation: Rotation<Point<Self::Float, Self::Dimension>>;
}

#[derive(Copy, Clone, Debug, Default, Eq, Hash, PartialEq)]
pub struct F32D2;

impl DimensionalSealSpell for F32D2 {}

impl PhysicalDimension for F32D2 {
    type Dimension = U2;
    type DimensionValue = <U2 as DimName>::Value;
    type Float = f32;
    type InternalDimProd = <Self::DimensionValue as Mul<<U1 as DimName>::Value>>::Output;
    type Rotation = UnitComplex<f32>;
}

pub type Isometry3<N: RealField> = NalgebraIsometry<N, U3, UnitQuaternion<N>>;
pub type Isometry2<N: RealField> = NalgebraIsometry<N, U2, UnitComplex<N>>;

pub type AngularMotion3<N: RealField> = Vector3<N>;
pub type AngularMotion2<N: RealField> = N;

#[cfg(feature = "dim3")]
pub type Isometry<N: RealField> = Isometry3<N>;
#[cfg(feature = "dim2")]
pub type Isometry<N: RealField> = Isometry2<N>;

#[cfg(feature = "dim3")]
pub type Vector<N: RealField> = Vector3<N>;
#[cfg(feature = "dim2")]
pub type Vector<N: RealField> = Vector2<N>;
