use super::{cross::GeneralizedCross, RigidBodyFlags, RigidBodyProperties};

use crate::{
    bodies::{
        components::{
            internal::{
                AdvancePosition,
                AugmentedInertia,
                GlobalCenterOfMass,
                GlobalInertia,
                InvertedAugmentedInertia,
            },
            Acceleration,
            ActivationStatus,
            AngularInertia,
            BodyStatus,
            BodyUpdateStatus,
            CenterOfMass,
            ExternalForces,
            Mass,
            Position,
            Velocity,
        },
        CombinedInertia,
        Motion,
    },
    nalgebra::{geometry::Translation, zero as na_zero, DVectorSlice, DVectorSliceMut, RealField},
    ncollide::{
        interpolation::{ConstantLinearVelocityRigidMotion, ConstantVelocityRigidMotion},
        shape::DeformationsType,
    },
    nphysics::{
        math::{
            Force,
            ForceType,
            Inertia,
            Isometry,
            Point,
            Vector,
            Velocity as NVelocity,
            SPATIAL_DIM,
        },
        object::{
            ActivationStatus as NActivationStatus,
            Body,
            BodyPart,
            BodyPartMotion,
            BodySet,
            BodyStatus as NBodyStatus,
            BodyUpdateStatus as NBodyUpdateStatus,
        },
        solver::{ForceDirection, IntegrationParameters},
    },
};

use shrinkwraprs::Shrinkwrap;
use specs::{Component, DenseVecStorage, Entity, Join, WriteStorage};

#[derive(Shrinkwrap)]
#[shrinkwrap(mutable)]
pub(crate) struct RigidBodySet<'a, N: RealField, P: Position<N>>(
    pub WriteStorage<'a, RigidBody<N, P>>,
);

impl<'a, N: RealField, P: Position<N>> BodySet<N> for RigidBodySet<'a, N, P> {
    type Body = RigidBody<N, P>;
    type Handle = Entity;

    fn get(&self, handle: Self::Handle) -> Option<&Self::Body> {
        self.0.get(handle)
    }

    fn get_mut(&mut self, handle: Self::Handle) -> Option<&mut Self::Body> {
        self.0.get_mut(handle)
    }

    fn get_pair_mut(
        &mut self,
        _handle1: Self::Handle,
        _handle2: Self::Handle,
    ) -> (Option<&mut Self::Body>, Option<&mut Self::Body>) {
        unimplemented!()
    }

    fn contains(&self, handle: Self::Handle) -> bool {
        self.0.contains(handle)
    }

    fn foreach(&self, mut f: impl FnMut(Self::Handle, &Self::Body)) {
        for (handle, body) in (self.fetched_entities(), &self.0).join() {
            f(handle, body)
        }
    }

    fn foreach_mut(&mut self, _f: impl FnMut(Self::Handle, &mut Self::Body)) {
        unimplemented!()
    }

    fn pop_removal_event(&mut self) -> Option<Self::Handle> {
        unimplemented!()
    }
}

// TODO: problems (such as the 'static lifetime bound on Body's)
// TODO: prevent us from directly implementing onto storage pointers.
// TODO: We must synchronize to our buddy here instead.
#[derive(Component)]
pub struct RigidBody<N: RealField, P: Position<N>> {
    properties: RigidBodyProperties<N>,
    activation: ActivationStatus<N>,
    status: BodyStatus,
    update: BodyUpdateStatus,
    position: P,
    velocity: Velocity<N>,
    acceleration: Acceleration<N>,
    mass: Mass<N>,
    angular_inertia: AngularInertia<N>,
    center_of_mass: CenterOfMass<N>,
    external_forces: ExternalForces<N>,
    advance_position: AdvancePosition<N, P, <P as Component>::Storage>,
    global_center_of_mass: GlobalCenterOfMass<N>,
    global_inertia: GlobalInertia<N>,
    augmented_inertia: AugmentedInertia<N>,
    augmented_inertia_inv: InvertedAugmentedInertia<N>,
    converted_activation_status: NActivationStatus<N>,
}

impl<P: Position<N>, N: RealField> Body<N> for RigidBody<N, P> {
    fn advance(&mut self, time_ratio: N) {
        let motion = self.part_motion(0, N::zero()).unwrap();
        let mut i = Isometry::identity();
        self.advance_position.set_isometry(match motion {
            BodyPartMotion::RigidLinear(m) => {
                i.translation =
                    (m.start.translation.vector + m.velocity * (time_ratio - m.t0)).into();
                i.rotation = m.start.rotation;

                &i
            }
            BodyPartMotion::RigidNonlinear(m) => {
                let scaled_linvel = m.linvel * (time_ratio - m.t0);
                let scaled_angvel = m.angvel * (time_ratio - m.t0);

                let center = m.start.rotation * m.local_center.coords;
                let lhs = m.start.translation * Translation::from(center);
                let rhs = Translation::from(-center) * m.start.rotation;

                let ret = lhs * Isometry::new(scaled_linvel, scaled_angvel) * rhs;
                i.translation = ret.translation;
                i.rotation = ret.rotation;
                &i
            }
            BodyPartMotion::Static(m) => {
                i.translation = m.translation;
                i.rotation = m.rotation;
                &i
            }
        });
    }

    fn validate_advancement(&mut self) {
        self.advance_position.set_isometry(self.position.isometry());
    }

    fn clamp_advancement(&mut self) {
        if self
            .properties
            .flags
            .contains(RigidBodyFlags::LINEAR_INTERPOLATION_ENABLED)
        {
            let p0 = Isometry::from_parts(
                self.advance_position.isometry().translation,
                self.position.isometry().rotation,
            );
            self.position.set_isometry(&p0);
        } else {
            self.position.set_isometry(self.advance_position.isometry());
        }
    }

    fn part_motion(&self, _: usize, time_origin: N) -> Option<BodyPartMotion<N>> {
        if self
            .properties
            .flags
            .contains(RigidBodyFlags::LINEAR_INTERPOLATION_ENABLED)
        {
            let p0 = Isometry::from_parts(
                self.advance_position.isometry().translation,
                self.position.isometry().rotation,
            );
            let motion =
                ConstantLinearVelocityRigidMotion::new(time_origin, p0, self.velocity.linear);
            Some(BodyPartMotion::RigidLinear(motion))
        } else {
            let motion = ConstantVelocityRigidMotion::new(
                time_origin,
                *self.advance_position.isometry(),
                *self.center_of_mass,
                self.velocity.linear,
                self.velocity.angular,
            );
            Some(BodyPartMotion::RigidNonlinear(motion))
        }
    }

    fn step_started(&mut self) {
        self.advance_position.set_isometry(self.position.isometry());
    }

    fn update_kinematics(&mut self) {}

    #[allow(unused_variables)] // for parameters used only in 3D.
    fn update_dynamics(&mut self, dt: N) {
        if !self.update.inertia_needs_update() || self.status != BodyStatus::Dynamic {
            return;
        }

        if !self.activation.is_active() {
            self.activate();
        }

        match self.status {
            #[cfg(feature = "dim3")]
            BodyStatus::Dynamic => {
                // The inverse inertia matrix is constant in 2D.
                let transformed = CombinedInertia {
                    linear: self.mass.0,
                    angular: self.angular_inertia.0,
                }
                .transformed(*self.position.isometry());
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

        match self.status {
            BodyStatus::Dynamic => {
                // The inverse inertia matrix is constant in 2D.
                #[cfg(feature = "dim3")]
                {
                    /*
                     * Compute acceleration due to gyroscopic forces.
                     */
                    let i = self.global_inertia.angular;
                    let w = &self.velocity.angular;
                    let iw = i * w;
                    let gyroscopic = -w.cross(&iw);
                    self.acceleration.angular = self.augmented_inertia_inv.angular * gyroscopic;
                }

                if self.augmented_inertia_inv.linear != N::zero()
                    && self
                        .properties
                        .flags
                        .contains(RigidBodyFlags::GRAVITY_ENABLED)
                {
                    self.acceleration.linear = *gravity;
                }

                let accel: NVelocity<N> =
                    Inertia::new(
                        self.augmented_inertia_inv.linear,
                        self.augmented_inertia_inv.angular,
                    ) * Force::new(self.external_forces.linear, self.external_forces.angular);
                self.acceleration.linear += accel.linear;
                self.acceleration.angular += accel.angular;
                self.acceleration
                    .as_vector_mut()
                    .component_mul_assign(&self.properties.flags.into());
            }
            _ => {}
        }
    }

    fn clear_forces(&mut self) {
        self.external_forces.zero();
    }

    fn clear_update_flags(&mut self) {
        self.update.clear();
    }

    #[inline]
    fn update_status(&self) -> NBodyUpdateStatus {
        self.update.into()
    }

    #[inline]
    fn apply_displacement(&mut self, displacement: &[N]) {
        let displacement = Motion::from_slice(displacement);
        let displacement = {
            let shift = Translation::from(self.global_center_of_mass.coords);
            shift * Isometry::new(displacement.linear, displacement.angular) * shift.inverse()
        };
        let new_pos = displacement * self.position.isometry();
        self.update.insert(BodyUpdateStatus::POSITION_CHANGED);
        self.position.set_isometry(&new_pos);
        self.global_center_of_mass.0 = new_pos * self.center_of_mass.0;
    }

    #[inline]
    fn status(&self) -> NBodyStatus {
        self.status.into()
    }

    #[inline]
    fn set_status(&mut self, status: NBodyStatus) {
        if status != self.status.into() {
            self.update.insert(BodyUpdateStatus::STATUS_CHANGED);
        }
        self.status = match status {
            NBodyStatus::Disabled => BodyStatus::Disabled,
            NBodyStatus::Static => BodyStatus::Static,
            NBodyStatus::Dynamic => BodyStatus::Dynamic,
            NBodyStatus::Kinematic => BodyStatus::Kinematic,
        };
    }

    #[inline]
    fn activation_status(&self) -> &NActivationStatus<N> {
        &self.converted_activation_status
    }

    #[inline]
    fn set_deactivation_threshold(&mut self, threshold: Option<N>) {
        self.activation.threshold = threshold;
        self.converted_activation_status
            .set_deactivation_threshold(threshold);
    }

    #[inline]
    fn ndofs(&self) -> usize {
        SPATIAL_DIM
    }

    #[inline]
    fn generalized_acceleration(&self) -> DVectorSlice<N> {
        DVectorSlice::from_slice(self.acceleration.as_slice(), SPATIAL_DIM)
    }

    #[inline]
    fn generalized_velocity(&self) -> DVectorSlice<N> {
        DVectorSlice::from_slice((*self.velocity).as_slice(), SPATIAL_DIM)
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
    fn generalized_velocity_mut(&mut self) -> DVectorSliceMut<N> {
        self.update.insert(BodyUpdateStatus::VELOCITY_CHANGED);
        DVectorSliceMut::from_slice(self.velocity.as_mut_slice(), SPATIAL_DIM)
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
                self.velocity.angular = self.properties.max_velocity_angular;
            } else if self.velocity.angular < -self.properties.max_velocity_angular {
                self.velocity.angular = -self.properties.max_velocity_angular;
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

        let disp = Motion {
            linear: self.velocity.linear * parameters.dt(),
            angular: self.velocity.angular * parameters.dt(),
        };
        self.apply_displacement(&disp.as_slice());
    }

    #[inline]
    fn activate_with_energy(&mut self, energy: N) {
        self.activation.energy = energy;
        self.converted_activation_status.set_energy(energy);
    }

    #[inline]
    fn deactivate(&mut self) {
        self.update.clear();
        self.activation.energy = N::zero();
        self.converted_activation_status.set_energy(N::zero());
        self.velocity.zero();
    }

    #[inline]
    fn part(&self, _: usize) -> Option<&dyn BodyPart<N>> {
        Some(self)
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
        let pos = point - self.global_center_of_mass.coords;
        let force = force_dir.at_point(&pos);
        let mut masked_force = force.clone();
        masked_force
            .as_vector_mut()
            .component_mul_assign(&self.properties.flags.into());

        match self.status {
            BodyStatus::Kinematic => {
                if let Some(out_vel) = out_vel {
                    // Don't use the masked force here so the locked
                    // DOF remain controllable at the velocity level.
                    *out_vel += force.as_vector().dot(&self.velocity.as_vector());
                }
            }
            BodyStatus::Dynamic => {
                jacobians[j_id..j_id + SPATIAL_DIM].copy_from_slice(masked_force.as_slice());

                let inv_mass = Inertia::new(
                    self.augmented_inertia_inv.linear,
                    self.augmented_inertia_inv.angular,
                );
                let imf = inv_mass * masked_force;
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
    fn world_point_at_material_point(&self, _: &dyn BodyPart<N>, point: &Point<N>) -> Point<N> {
        self.position.isometry() * point
    }

    #[inline]
    fn position_at_material_point(&self, _: &dyn BodyPart<N>, point: &Point<N>) -> Isometry<N> {
        self.position.isometry() * Translation::from(point.coords)
    }

    #[inline]
    fn material_point_at_world_point(&self, _: &dyn BodyPart<N>, point: &Point<N>) -> Point<N> {
        self.position.isometry().inverse_transform_point(point)
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
        self.update
            .insert(BodyUpdateStatus::CENTER_OF_MASS_CHANGED | BodyUpdateStatus::INERTIA_CHANGED);

        let mass_sum = self.global_inertia.linear + inertia.linear;

        // Update center of mass.
        if !mass_sum.is_zero() {
            self.center_of_mass.0 = (self.center_of_mass.0 * self.global_inertia.linear
                + com.coords * inertia.linear)
                / mass_sum;
            self.global_center_of_mass.0 = self.position.isometry() * self.center_of_mass.0;
        } else {
            self.center_of_mass.0 = Point::origin();
            self.global_center_of_mass.0 = self.position.isometry().translation.vector.into();
        }

        // Update local inertia.
        self.mass.0 += inertia.linear;
        self.angular_inertia.0 += inertia.angular;
        let global = CombinedInertia {
            linear: self.mass.0,
            angular: self.angular_inertia.0,
        }
        .transformed(*self.position.isometry());
        self.global_inertia.linear = global.linear;
        self.global_inertia.angular = global.angular;
        self.augmented_inertia.linear = global.linear;
        self.augmented_inertia.angular = global.angular;
        let inverted = global.inverse();
        self.augmented_inertia_inv.linear = inverted.linear;
        self.augmented_inertia_inv.angular = inverted.angular;
    }

    #[inline]
    fn gravity_enabled(&self) -> bool {
        self.properties
            .flags
            .contains(RigidBodyFlags::GRAVITY_ENABLED)
    }

    #[inline]
    fn enable_gravity(&mut self, enabled: bool) {
        self.properties
            .flags
            .set(RigidBodyFlags::GRAVITY_ENABLED, enabled);
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
        if self.status != BodyStatus::Dynamic {
            return;
        }

        if auto_wake_up {
            self.activate();
        }

        match force_type {
            ForceType::Force => self.external_forces.as_vector_mut().cmpy(
                N::one(),
                force.as_vector(),
                &self.properties.flags.into(),
                N::one(),
            ),
            ForceType::Impulse => {
                self.update.insert(BodyUpdateStatus::VELOCITY_CHANGED);
                let dvel = Inertia::new(
                    self.augmented_inertia_inv.linear,
                    self.augmented_inertia_inv.angular,
                ) * *force;
                self.velocity.as_vector_mut().cmpy(
                    N::one(),
                    dvel.as_vector(),
                    &self.properties.flags.into(),
                    N::one(),
                )
            }
            ForceType::AccelerationChange => {
                let change = Inertia::new(
                    self.augmented_inertia.0.linear,
                    self.augmented_inertia.0.angular,
                ) * *force;
                self.external_forces.as_vector_mut().cmpy(
                    N::one(),
                    change.as_vector(),
                    &self.properties.flags.into(),
                    N::one(),
                )
            }
            ForceType::VelocityChange => {
                self.update.insert(BodyUpdateStatus::VELOCITY_CHANGED);
                self.velocity.as_vector_mut().cmpy(
                    N::one(),
                    force.as_vector(),
                    &self.properties.flags.into(),
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
        let force = Force::linear_at_point(*force, &(point - self.global_center_of_mass.coords));
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

impl<P: Position<N>, N: RealField> BodyPart<N> for RigidBody<N, P> {
    #[inline]
    fn is_ground(&self) -> bool {
        false
    }

    #[inline]
    fn center_of_mass(&self) -> Point<N> {
        self.global_center_of_mass.0
    }

    #[inline]
    fn local_center_of_mass(&self) -> Point<N> {
        self.center_of_mass.0
    }

    #[inline]
    fn position(&self) -> Isometry<N> {
        *self.position.isometry()
    }

    #[inline]
    fn safe_position(&self) -> Isometry<N> {
        if self
            .properties
            .flags
            .contains(RigidBodyFlags::LINEAR_INTERPOLATION_ENABLED)
        {
            Isometry::from_parts(
                self.advance_position.isometry().translation,
                self.position.isometry().rotation,
            )
        } else {
            *self.advance_position.isometry()
        }
    }

    #[inline]
    fn velocity(&self) -> NVelocity<N> {
        NVelocity::new(self.velocity.linear, self.velocity.angular)
    }

    #[inline]
    fn inertia(&self) -> Inertia<N> {
        Inertia::new(self.global_inertia.linear, self.global_inertia.angular)
    }

    #[inline]
    fn local_inertia(&self) -> Inertia<N> {
        Inertia::new(self.mass.0, self.angular_inertia.0)
    }
}
