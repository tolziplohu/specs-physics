use specs::{Component, DenseVecStorage, FlaggedStorage};

use crate::{
    nalgebra::{Isometry3, Matrix3, Point3, RealField},
    nphysics::{
        algebra::{Force3, ForceType, Velocity3},
        object::{Body, BodyHandle, BodyPart, BodyStatus, RigidBody, RigidBodyDesc},
    },
};

/// The `PhysicsBodyBuilder` implements the builder pattern for `PhysicsBody`s
/// and is the recommended way of instantiating and customising new
/// `PhysicsBody` instances.
///
/// # Example
///
/// ```rust
/// use specs_physics::{
///     nalgebra::{Matrix3, Point3},
///     nphysics::{algebra::Velocity3, object::BodyStatus},
///     PhysicsBodyBuilder,
/// };
///
/// let physics_body = PhysicsBodyBuilder::from(BodyStatus::Dynamic)
///     .gravity_enabled(true)
///     .velocity(Velocity3::linear(1.0, 1.0, 1.0))
///     .angular_inertia(Matrix3::from_diagonal_element(3.0))
///     .mass(1.3)
///     .local_center_of_mass(Point3::new(0.0, 0.0, 0.0))
///     .build();
/// ```
// TODO: This could be expanded through BodyData to build not only rigidbodies,
// TODO: but FEMVolumes, MassConstraintSystems, and MassSpringSystems.
pub struct PhysicsBodyBuilder<'a, N: RealField, P: Position<N>> {
    world: &'a World,
    entity: Option<Entity>,
    body_type: Option<BodyData<N>>,
    status: Option<BodyStatus>,

    position: Option<P>,
    velocity: Option<Velocity<N>>,
    acceleration: Option<Acceleration<N>>,
    external_forces: Option<ExternalForces<N>>,

    mass: Option<Mass<N>>,
    angular_inertia: Option<AngularInertia<N>>,
    center_of_mass: Option<CenterOfMass<N>>,
}

impl<'a, N: RealField, P: Position<N>> PhysicsBodyBuilder<'a, N, P> {
    pub fn create_body(world: &mut World) -> Self {
        Self::create_body_internal(world, None, None, None)
    }

    pub fn create_body_on_entity(world: &mut World, entity: Entity) -> Self {
        Self::create_body_internal(world, Some(entity), None, None)
    }

    pub fn create_rigid_body(world: &mut World) -> Self {
        Self::create_body_internal(world, None, Some(BodyData::new_rigid()), None)
    }

    pub fn create_rigid_body_on_entity(world: &mut World, entity: Entity) -> Self {
        Self::create_body_internal(world, Some(entity), Some(BodyData::new_rigid()), None)
    }

    pub fn create_kinematic_rigid_body(world: &mut World) -> Self {
        Self::create_body_internal(
            world,
            None,
            Some(BodyData::new_rigid()),
            Some(BodyStatus::Kinematic),
        )
    }

    pub fn create_kinematic_rigid_body_on_entity(world: &mut World, entity: Entity) -> Self {
        Self::create_body_internal(
            world,
            Some(entity),
            Some(BodyData::new_rigid()),
            Some(BodyStatus::Kinematic),
        )
    }

    pub fn create_static_rigid_body(world: &mut World) -> Self {
        Self::create_body_internal(
            world,
            None,
            Some(BodyData::new_rigid()),
            Some(BodyStatus::Static),
        )
    }

    pub fn create_static_rigid_body_on_entity(world: &mut World, entity: Entity) -> Self {
        Self::create_body_internal(
            world,
            Some(entity),
            Some(BodyData::new_rigid()),
            Some(BodyStatus::Static),
        )
    }

    fn create_body_internal(
        world: &World,
        entity: Option<Entity>,
        body_type: Option<BodyData<N>>,
        status: Option<BodyStatus>,
    ) -> Self {
        PhysicsBodyBuilder {
            world,
            entity,
            body_type,
            status,

            None,
            None,
            None,
            None,

            None,
            None,
            None,
        }
    }

    pub fn position(mut self, position: P) -> Self {
        self.position = Some(position);
        self
    }

    // Sets the `velocity` value of the `PhysicsBodyBuilder`.
    pub fn velocity(mut self, velocity: Motion<N>) -> Self {
        self.velocity = Some(velocity);
        self
    }

    //pub fn velocity_linear(mut self, velocity: Vector<N>)

    //pub fn acceleration(mut self, acceleration: )

    /// Sets the `gravity_enabled` value of the `PhysicsBodyBuilder`.
    pub fn gravity_enabled(mut self, gravity_enabled: bool) -> Self {
        self.gravity_enabled = gravity_enabled;
        self
    }

    /// Sets the `angular_inertia` value of the `PhysicsBodyBuilder`.
    pub fn angular_inertia(mut self, angular_inertia: Matrix3<N>) -> Self {
        self.angular_inertia = angular_inertia;
        self
    }

    /// Sets the `mass` value of the `PhysicsBodyBuilder`.
    pub fn mass(mut self, mass: N) -> Self {
        self.mass = mass;
        self
    }

    /// Sets the local center of mass of the Body.
    pub fn local_center_of_mass(mut self, local_center_of_mass: Point3<N>) -> Self {
        self.local_center_of_mass = local_center_of_mass;
        self
    }

    /// Builds the `PhysicsBody` from the values set in the `PhysicsBodyBuilder`
    /// instance.
    pub fn build(self) -> Entity {
        let entity = self.unwrap_or_else(|| self.world.entities_mut().alloc.allocate());

        entity
    }
}

enum BodyData<N: RealField> {
    Rigid {
        properties: RigidBodyProperties<N>,
        flags: RigidBodyFlags<N>,
    },
}

impl<N: RealField> BodyData<N> {
    fn new_rigid() -> Self {
        Self::Rigid {
            properties: RigidBodyProperties::default(),
            flags: RigidBodyFlags::default(),
        }
    }

    fn with_rigid_properties(mut self, properties: RigidBodyProperties<N>) -> Self {
        match self {
            Self::Rigid(data) => {
                data.properties = properties;
                data
            }
            _ => Self::new_rigid().with_rigid_properties(properties),
        }
    }

    fn with_rigid_flags(mut self, flags: RigidBodyFlags<N>) -> Self {
        match self {
            Self::Rigid(data) => {
                data.flags = flags;
                data
            }
            _ => Self::new_rigid().with_rigid_flags(flags),
        }
    }
}

pub trait PhysicsWorldExt {
    fn create_body<N: RealField, P: Position<N>>(&mut self) -> PhysicsBodyBuilder;

    fn create_body_on_entity<N: RealField, P: Position<N>>(
        &mut self,
        entity: Entity,
    ) -> PhysicsBodyBuilder;

    fn create_rigid_body<N: RealField, P: Position<N>>(&mut self) -> PhysicsBodyBuilder;

    fn create_rigid_body_on_entity<N: RealField, P: Position<N>>(
        &mut self,
        entity: Entity,
    ) -> PhysicsBodyBuilder;

    fn create_kinematic_rigid_body<N: RealField, P: Position<N>>(&mut self) -> PhysicsBodyBuilder;

    fn create_kinematic_rigid_body_on_entity<N: RealField, P: Position<N>>(
        &mut self,
        entity: Entity,
    ) -> PhysicsBodyBuilder;

    fn create_static_rigid_body<N: RealField, P: Position<N>>(&mut self) -> PhysicsBodyBuilder;

    fn create_static_rigid_body_on_entity<N: RealField, P: Position<N>>(
        &mut self,
        entity: Entity,
    ) -> PhysicsBodyBuilder;
}

impl PhysicsWorldExt for World {
    fn create_body<N: RealField, P: Position<N>>(&mut self) -> PhysicsBodyBuilder {
        PhysicsBodyBuilder::<N, P>::create_body(&mut self)
    }

    fn create_body_on_entity<N: RealField, P: Position<N>>(
        &mut self,
        entity: Entity,
    ) -> PhysicsBodyBuilder {
        PhysicsBodyBuilder::<N, P>::create_body_on_entity(&mut self, entity)
    }

    fn create_rigid_body<N: RealField, P: Position<N>>(&mut self) -> PhysicsBodyBuilder {
        PhysicsBodyBuilder::<N, P>::create_rigid_body(&mut self)
    }

    fn create_rigid_body_on_entity<N: RealField, P: Position<N>>(
        &mut self,
        entity: Entity,
    ) -> PhysicsBodyBuilder {
        PhysicsBodyBuilder::<N, P>::create_rigid_body_on_entity(&mut self, entity)
    }

    fn create_kinematic_rigid_body<N: RealField, P: Position<N>>(&mut self) -> PhysicsBodyBuilder {
        PhysicsBodyBuilder::<N, P>::create_kinematic_rigid_body(&mut self)
    }

    fn create_kinematic_rigid_body_on_entity<N: RealField, P: Position<N>>(
        &mut self,
        entity: Entity,
    ) -> PhysicsBodyBuilder {
        PhysicsBodyBuilder::<N, P>::create_kinematic_rigid_body_on_entity(&mut self, entity)
    }

    fn create_static_rigid_body<N: RealField, P: Position<N>>(&mut self) -> PhysicsBodyBuilder {
        PhysicsBodyBuilder::<N, P>::create_static_rigid_body(&mut self)
    }

    fn create_static_rigid_body_on_entity<N: RealField, P: Position<N>>(
        &mut self,
        entity: Entity,
    ) -> PhysicsBodyBuilder {
        PhysicsBodyBuilder::<N, P>::create_static_rigid_body_on_entity(&mut self, entity)
    }
}
