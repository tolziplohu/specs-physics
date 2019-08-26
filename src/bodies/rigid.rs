#[cfg(feature = "serialization")]
use serde::{Deserialize, Serialize};

use super::{
    status::{ActivationStatus, BodyStatus, BodyUpdateStatus},
    GeneralizedCross,
    Acceleration,
    ExternalForces,
    GlobalInertia,
    AugmentedInertia,
    AugmentedInertiaInv,
    Mass,
    Velocity,
};

use crate::{
    nalgebra::{geometry::Translation, zero as na_zero, DVectorSlice, DVectorSliceMut, RealField},
    ncollide::{
        interpolation::{ConstantLinearVelocityRigidMotion, ConstantVelocityRigidMotion},
        shape::DeformationsType,
    },
    nphysics::{
        math::{Force, ForceType, Inertia, Isometry, Point, Vector, SPATIAL_DIM},
        object::{
            ActivationStatus as NActivationStatus,
            Body,
            BodyPart,
            BodyPartMotion,
            BodyStatus as NBodyStatus,
            BodyUpdateStatus as NBodyUpdateStatus,
        },
        solver::{ForceDirection, IntegrationParameters},
    },
    position::Position,
};

/// When paired with a Position, Velocity, and Mass, designates an entity as a
/// rigid body, and thus describes that rigid body's characteristics.
#[cfg_attr(feature = "serialization", derive(Serialize, Deserialize))]
#[derive(Debug, Clone, PartialEq)]
pub struct RigidBodyProperties<N: RealField> {
    pub gravity_enabled: bool,

    /// Which linear degrees of freedom are kinematic
    pub kinematic_translations: Vector<bool>,
    /// Which angular degrees of freedom are kinematic
    pub kinematic_rotations: Vector<bool>,

    /// Linear damping/drag coefficient
    pub damping: N,
    /// Angular damping/drag coefficient
    pub damping_angular: N,

    /// Maximum allowed linear velocity
    pub max_velocity: N,
    /// Maximum allowed angular velocity
    pub max_velocity_angular: N,

    /// Enables linear motion interpolation for CCD
    pub linear_motion_interpolation_enabled: bool,

    // LOL idk.
    pub companion_id: usize,
}

impl<N: RealField> Default for RigidBodyProperties<N> {
    #[inline]
    fn default() -> Self {
        RigidBodyProperties {
            gravity_enabled: true,
            kinematic_translations: Vector::repeat(false),
            kinematic_rotations: Vector::repeat(false),
            damping: N::zero(),
            damping_angular: N::zero(),
            max_velocity: N::max_value(),
            max_velocity_angular: N::max_value(),
            linear_motion_interpolation_enabled: false,
            companion_id: 0,
        }
    }
}

struct RigidBody<N: RealField, P: Position<N>> {
    next_position: Box<P>,
    position: Box<P>,
    velocity: Box<Velocity<N>>,
    acceleration: Box<Acceleration<N>>,
    external_forces: Box<ExternalForces<N>>,
    mass: Box<Mass<N>>,
    global_inertia: Box<GlobalInertia<N>>,
    augmented_inertia: Box<AugmentedInertia<N>>,
    augmented_inertia_inv: Box<AugmentedInertiaInv<N>>,
    properties: Box<RigidBodyProperties<N>>,
    activation: Box<ActivationStatus<N>>,
    status: Box<BodyStatus>,
    update: Box<BodyUpdateStatus>,
}

impl<P: Position<N>, N: RealField> Body<N> for RigidBody<N, P> {
    #[inline]
    fn activation_status(&self) -> &NActivationStatus<N> {
        &(*self.activation).into()
    }

    #[inline]
    fn activate_with_energy(&mut self, energy: N) {
        self.activation.energy = energy;
    }

    #[inline]
    fn deactivate(&mut self) {
        self.update.clear();
        self.activation.energy = N::zero();
        self.velocity.zero();
    }

    #[inline]
    fn set_deactivation_threshold(&mut self, threshold: Option<N>) {
        self.activation.threshold = threshold
    }

    #[inline]
    fn update_status(&self) -> NBodyUpdateStatus {
        (*self.update).into()
    }

    #[inline]
    fn status(&self) -> NBodyStatus {
        (*self.status).into()
    }

    #[inline]
    fn set_status(&mut self, status: NBodyStatus) {
        if status != (*self.status).into() {
            self.update.status_changed = true;
        }
        *self.status = match status {
            NBodyStatus::Disabled => BodyStatus::Disabled,
            NBodyStatus::Static => BodyStatus::Static,
            NBodyStatus::Dynamic => BodyStatus::Dynamic,
            NBodyStatus::Kinematic => BodyStatus::Kinematic,
        };
    }

    #[inline]
    fn deformed_positions(&self) -> Option<(DeformationsType, &[N])> {
        None
    }

    #[inline]
    fn deformed_positions_mut(&mut self) -> Option<(DeformationsType, &mut [N])> {
        None
    }

    #[inline]
    fn companion_id(&self) -> usize {
        self.properties.companion_id
    }

    #[inline]
    fn set_companion_id(&mut self, id: usize) {
        self.properties.companion_id = id
    }

    #[inline]
    fn ndofs(&self) -> usize {
        SPATIAL_DIM
    }

    #[inline]
    fn generalized_velocity(&self) -> DVectorSlice<N> {
        DVectorSlice::from_slice(self.velocity.as_slice(), SPATIAL_DIM)
    }

    #[inline]
    fn generalized_velocity_mut(&mut self) -> DVectorSliceMut<N> {
        self.update.velocity_changed = true;
        DVectorSliceMut::from_slice(self.velocity.as_mut_slice(), SPATIAL_DIM)
    }

    #[inline]
    fn generalized_acceleration(&self) -> DVectorSlice<N> {
        DVectorSlice::from_slice(self.acceleration.as_slice(), SPATIAL_DIM)
    }

    #[inline]
    fn integrate(&mut self, parameters: &IntegrationParameters<N>) {
        self.velocity.linear *= N::one() / (N::one() + parameters.dt() * self.properties.damping);
        self.velocity.angular *=
            N::one() / (N::one() + parameters.dt() * self.properties.damping_angular);

        let linvel_norm = self.velocity.linear.norm();

        if linvel_norm > self.properties.max_velocity {
            if self.properties.max_velocity.is_zero() {
                self.velocity.linear = na_zero();
            } else {
                self.velocity.linear /= self.properties.max_velocity * linvel_norm;
            }
        }

        #[cfg(feature = "dim2")]
        {
            if self.velocity.angular > self.properties.max_velocity_angular {
                self.velocity.angular = self.max_velocity_angular;
            } else if self.velocity.angular < -self.max_velocity_angular {
                self.velocity.angular = -self.max_velocity_angular;
            }
        }

        #[cfg(feature = "dim3")]
        {
            let angvel_norm = self.velocity.angular.norm();

            if angvel_norm > self.properties.max_velocity_angular {
                if self.properties.max_velocity_angular.is_zero() {
                    self.velocity.angular = na_zero()
                } else {
                    self.velocity.angular *= self.properties.max_velocity_angular / angvel_norm;
                }
            }
        }

        let disp = *self.velocity * parameters.dt();
        self.apply_displacement(&disp.as_slice());
    }

    fn clear_forces(&mut self) {
        self.external_forces.zero();
    }

    fn clear_update_flags(&mut self) {
        self.update.clear();
    }

    fn update_kinematics(&mut self) {}

    fn step_started(&mut self) {
        self.next_position.set_isometry(self.position.isometry());
    }

    fn advance(&mut self, time_ratio: N) {
        let motion = self.part_motion(0, N::zero()).unwrap();
        self.next_position.set_isometry(match motion {
            BodyPartMotion::RigidLinear(m) => &Isometry::from_parts(
                (m.start.translation.vector + m.velocity * (time_ratio - m.t0)).into(),
                m.start.rotation,
            ),
            BodyPartMotion::RigidNonlinear(m) => &{
                let scaled_linvel = m.linvel * (time_ratio - m.t0);
                let scaled_angvel = m.angvel * (time_ratio - m.t0);

                let center = m.start.rotation * m.local_center.coords;
                let lhs = m.start.translation * Translation::from(center);
                let rhs = Translation::from(-center) * m.start.rotation;

                lhs * Isometry::new(scaled_linvel, scaled_angvel) * rhs
            },
            BodyPartMotion::Static(m) => &m,
        });
    }

    fn validate_advancement(&mut self) {
        self.next_position.set_isometry(self.position.isometry());
    }

    fn clamp_advancement(&mut self) {
        if self.properties.linear_motion_interpolation_enabled {
            let p0 = Isometry::from_parts(
                self.next_position.isometry().translation,
                self.position.isometry().rotation,
            );
            self.position.set_isometry(&p0);
        } else {
            self.position.set_isometry(self.next_position.isometry());
        }
    }

    fn part_motion(&self, _: usize, time_origin: N) -> Option<BodyPartMotion<N>> {
        if self.properties.linear_motion_interpolation_enabled {
            let p0 = Isometry::from_parts(
                self.next_position.isometry().translation,
                self.position.isometry().rotation,
            );
            let motion =
                ConstantLinearVelocityRigidMotion::new(time_origin, p0, self.velocity.linear);
            Some(BodyPartMotion::RigidLinear(motion))
        } else {
            let motion = ConstantVelocityRigidMotion::new(
                time_origin,
                *self.next_position.isometry(),
                self.mass.center,
                self.velocity.linear,
                self.velocity.angular,
            );
            Some(BodyPartMotion::RigidNonlinear(motion))
        }
    }

    #[allow(unused_variables)] // for parameters used only in 3D.
    fn update_dynamics(&mut self, dt: N) {
        if !self.update.inertia_needs_update() || *self.status != BodyStatus::Dynamic {
            return;
        }

        if !self.activation.is_active() {
            self.activate();
        }

        match *self.status {
            #[cfg(feature = "dim3")]
            BodyStatus::Dynamic => {
                // The inverse inertia matrix is constant in 2D.
                let transformed = self.mass.transformed(*self.position.isometry());
                self.global_inertia.linear = transformed.linear;
                self.global_inertia.angular = transformed.angular;
                self.augmented_inertia.linear = transformed.linear;
                self.augmented_inertia.angular = transformed.angular;

                let i = &self.global_inertia.angular;
                let w = &self.velocity.angular;
                let iw = i * w;
                let w_dt = w * dt;
                let w_dt_cross = w_dt.gcross_matrix();
                let iw_dt_cross = (iw * dt).gcross_matrix();
                self.augmented_inertia.angular += w_dt_cross * i - iw_dt_cross;

                // NOTE: if we did not have the gyroscopic forces, we would not have to invert
                // the inertia matrix at each time-step => add a flag to disable
                // gyroscopic forces?
                let inv = self.augmented_inertia.inverse();
                self.augmented_inertia_inv.linear = inv.linear;
                self.augmented_inertia_inv.angular = inv.angular;
            }
            _ => {}
        }
    }

    fn update_acceleration(&mut self, gravity: &Vector<N>, _: &IntegrationParameters<N>) {
        self.acceleration.zero();

        match *self.status {
            BodyStatus::Dynamic => {
                // The inverse inertia matrix is constant in 2D.
                #[cfg(feature = "dim3")]
                {
                    /*
                     * Compute acceleration due to gyroscopic forces.
                     */
                    let i = &self.mass.angular;
                    let w = &self.velocity.angular;
                    let iw = i * w;
                    let gyroscopic = -w.cross(&iw);
                    self.acceleration.angular = self.augmented_inertia_inv.angular * gyroscopic;
                }

                if self.augmented_inertia_inv.linear != N::zero() && self.properties.gravity_enabled {
                    self.acceleration.linear = *gravity;
                }

                self.acceleration += self.augmented_inertia_inv * self.external_forces;
                self.acceleration
                    .as_vector_mut()
                    .component_mul_assign(&self.jacobian_mask);
            }
            _ => {}
        }
    }

    #[inline]
    fn part(&self, _: usize) -> Option<&dyn BodyPart<N>> {
        Some(self)
    }

    #[inline]
    fn apply_displacement(&mut self, displacement: &[N]) {
        self.apply_displacement(&Velocity::from_slice(displacement));
    }

    #[inline]
    fn world_point_at_material_point(&self, _: &dyn BodyPart<N>, point: &Point<N>) -> Point<N> {
        self.position * point
    }

    #[inline]
    fn position_at_material_point(&self, _: &dyn BodyPart<N>, point: &Point<N>) -> Isometry<N> {
        self.position * Translation::from(point.coords)
    }

    #[inline]
    fn material_point_at_world_point(&self, _: &dyn BodyPart<N>, point: &Point<N>) -> Point<N> {
        self.position.isometry().inverse_transform_point(point)
    }

    #[inline]
    fn gravity_enabled(&self) -> bool {
        self.gravity_enabled
    }

    #[inline]
    fn enable_gravity(&mut self, enabled: bool) {
        self.gravity_enabled = enabled
    }

    #[inline]
    fn fill_constraint_geometry(
        &self,
        _: &dyn BodyPart<N>,
        _: usize,
        point: &Point<N>,
        force_dir: &ForceDirection<N>,
        j_id: usize,
        wj_id: usize,
        jacobians: &mut [N],
        inv_r: &mut N,
        ext_vels: Option<&DVectorSlice<N>>,
        out_vel: Option<&mut N>,
    ) {
        let pos = point - self.com.coords;
        let force = force_dir.at_point(&pos);
        let mut masked_force = force.clone();
        masked_force
            .as_vector_mut()
            .component_mul_assign(&self.jacobian_mask);

        match *self.status {
            BodyStatus::Kinematic => {
                if let Some(out_vel) = out_vel {
                    // Don't use the masked force here so the locked
                    // DOF remain controllable at the velocity level.
                    *out_vel += force.as_vector().dot(&self.velocity.as_vector());
                }
            }
            BodyStatus::Dynamic => {
                jacobians[j_id..j_id + SPATIAL_DIM].copy_from_slice(masked_force.as_slice());

                let inv_mass = self.inv_augmented_mass();
                let imf = *inv_mass * masked_force;
                jacobians[wj_id..wj_id + SPATIAL_DIM].copy_from_slice(imf.as_slice());

                *inv_r +=
                    inv_mass.mass() + masked_force.angular_vector().dot(&imf.angular_vector());

                if let Some(out_vel) = out_vel {
                    // Don't use the masked force here so the locked
                    // DOF remain controllable at the velocity level.
                    *out_vel += force.as_vector().dot(&self.velocity.as_vector());

                    if let Some(ext_vels) = ext_vels {
                        *out_vel += masked_force.as_vector().dot(ext_vels)
                    }
                }
            }
            BodyStatus::Static | BodyStatus::Disabled => {}
        }
    }

    #[inline]
    fn has_active_internal_constraints(&mut self) -> bool {
        false
    }

    #[inline]
    fn setup_internal_velocity_constraints(
        &mut self,
        _: &DVectorSlice<N>,
        _: &IntegrationParameters<N>,
    ) {
    }

    #[inline]
    fn warmstart_internal_velocity_constraints(&mut self, _: &mut DVectorSliceMut<N>) {}

    #[inline]
    fn step_solve_internal_velocity_constraints(&mut self, _: &mut DVectorSliceMut<N>) {}

    #[inline]
    fn step_solve_internal_position_constraints(&mut self, _: &IntegrationParameters<N>) {}

    #[inline]
    fn add_local_inertia_and_com(&mut self, _: usize, com: Point<N>, inertia: Inertia<N>) {
        self.update.center_of_mass_changed = true;
        self.update.inertia_changed = true;

        let mass_sum = self.inertia.linear + inertia.linear;

        // Update center of mass.
        if !mass_sum.is_zero() {
            self.local_com =
                (self.local_com * self.inertia.linear + com.coords * inertia.linear) / mass_sum;
            self.com = self.position * self.local_com;
        } else {
            self.local_com = Point::origin();
            self.com = self.position.translation.vector.into();
        }

        // Update local inertia.
        self.mass.linear += inertia.linear;
        self.mass.angular += inertia.angular;
        self.update_inertia_from_local_inertia();
    }

    /*
     * Application of forces/impulses.
     */
    fn apply_force(
        &mut self,
        _: usize,
        force: &Force<N>,
        force_type: ForceType,
        auto_wake_up: bool,
    ) {
        if *self.status != BodyStatus::Dynamic {
            return;
        }

        if auto_wake_up {
            self.activate();
        }

        match force_type {
            ForceType::Force => self.external_forces.as_vector_mut().cmpy(
                N::one(),
                force.as_vector(),
                &self.jacobian_mask,
                N::one(),
            ),
            ForceType::Impulse => {
                self.update.velocity_changed = true;
                let dvel = self.inv_augmented_mass * *force;
                self.velocity.as_vector_mut().cmpy(
                    N::one(),
                    dvel.as_vector(),
                    &self.jacobian_mask,
                    N::one(),
                )
            }
            ForceType::AccelerationChange => {
                let change = self.augmented_inertia * *force;
                self.external_forces.as_vector_mut().cmpy(
                    N::one(),
                    change.as_vector(),
                    &self.jacobian_mask,
                    N::one(),
                )
            }
            ForceType::VelocityChange => {
                self.update.velocity_changed = true;
                self.velocity.as_vector_mut().cmpy(
                    N::one(),
                    force.as_vector(),
                    &self.jacobian_mask,
                    N::one(),
                )
            }
        }
    }

    fn apply_local_force(
        &mut self,
        _: usize,
        force: &Force<N>,
        force_type: ForceType,
        auto_wake_up: bool,
    ) {
        let world_force = force.transform_by(&self.position.isometry());
        self.apply_force(0, &world_force, force_type, auto_wake_up)
    }

    fn apply_force_at_point(
        &mut self,
        _: usize,
        force: &Vector<N>,
        point: &Point<N>,
        force_type: ForceType,
        auto_wake_up: bool,
    ) {
        let force = Force::linear_at_point(*force, &(point - self.mass.center.coords));
        self.apply_force(0, &force, force_type, auto_wake_up)
    }

    fn apply_local_force_at_point(
        &mut self,
        _: usize,
        force: &Vector<N>,
        point: &Point<N>,
        force_type: ForceType,
        auto_wake_up: bool,
    ) {
        self.apply_force_at_point(
            0,
            &(self.position.isometry() * force),
            point,
            force_type,
            auto_wake_up,
        )
    }

    fn apply_force_at_local_point(
        &mut self,
        _: usize,
        force: &Vector<N>,
        point: &Point<N>,
        force_type: ForceType,
        auto_wake_up: bool,
    ) {
        self.apply_force_at_point(
            0,
            force,
            &(self.position.isometry() * point),
            force_type,
            auto_wake_up,
        )
    }

    fn apply_local_force_at_local_point(
        &mut self,
        _: usize,
        force: &Vector<N>,
        point: &Point<N>,
        force_type: ForceType,
        auto_wake_up: bool,
    ) {
        self.apply_force_at_point(
            0,
            &(self.position.isometry() * force),
            &(self.position.isometry() * point),
            force_type,
            auto_wake_up,
        )
    }
}
