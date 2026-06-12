use nalgebra as na;

pub enum MotorMixingMode {
    ThrottleAuthority,
    NoTorqueScalling,
    ThrottleAuthorityReasonable { min_scale: f32, max_torque: f32 },
}
impl Default for MotorMixingMode {
    fn default() -> Self {
        Self::ThrottleAuthorityReasonable {
            min_scale: 0.10,
            max_torque: 1.0,
        }
    }
}

pub struct MotorMixer {
    pub motor_map: [[f32; 3]; 4],
    pub min_throttle: f32,
    pub max_throttle: f32,
    pub mixing_mode: MotorMixingMode,
}

impl MotorMixer {
    pub fn mix(&self, throttle: f32, torque: na::Vector3<f32>) -> [f32; 4] {
        use MotorMixingMode::*;
        match self.mixing_mode {
            ThrottleAuthority => {
                return self.mix_throttle_authority(throttle, torque).0;
            }
            NoTorqueScalling => {
                return self.mix_no_torque_scalling(throttle, torque).0;
            }
            ThrottleAuthorityReasonable {
                min_scale,
                max_torque,
            } => {
                return self
                    .mix_throttle_authority_reasonable(throttle, torque, min_scale, max_torque)
                    .0;
            }
        }
    }

    fn compute_delta(&self, torque: na::Vector3<f32>) -> [f32; 4] {
        let mut delta = [0.0f32; 4];

        for i in 0..4 {
            delta[i] = self.motor_map[i][0] * torque.x
                + self.motor_map[i][1] * torque.y
                + self.motor_map[i][2] * torque.z;
        }

        return delta;
    }

    /// Unlike ThrottleAuthority, this implementation has a minimum scale.
    /// That makes sure our torque is never below min_scale * torque.
    ///
    /// Throttle still has authority, this is, torque is limited by the throttle, but within reason, which makes it "reasonable"
    ///
    /// Avoids the downside of the previous implementation: At 100% Throttle, Torque would always be 0. This lowers the actual throttle.
    pub fn mix_throttle_authority_reasonable(
        &self,
        throttle: f32,
        mut torque: na::Vector3<f32>,
        min_scale: f32,
        max_torque: f32,
    ) -> ([f32; 4], bool) {
        if torque.iter().any(|v| v.is_nan()) {
            return ([0.0; 4], true);
        }
        if torque.norm() > max_torque {
            torque = torque.normalize() * max_torque;
        }

        let delta = self.compute_delta(torque);

        let throttle_range_len = self.max_throttle - self.min_throttle;

        let mut scale = 1.0f32;

        for i in 0..4 {
            if delta[i] > 0.05 {
                scale = scale.min((self.max_throttle - throttle) / delta[i]);
            } else if delta[i] < -0.05 {
                scale = scale.min((self.min_throttle - throttle) / delta[i]);
            }
        }

        scale = scale.clamp(min_scale, 1.0);

        let mut max_delta = scale * delta.into_iter().reduce(f32::max).unwrap_or(0.0);
        let mut min_delta = scale * delta.into_iter().reduce(f32::min).unwrap_or(0.0);
        let delta_dif = (max_delta - min_delta).max(0.1);

        if delta_dif > throttle_range_len {
            scale *= throttle_range_len / delta_dif * 0.99;
            max_delta *= throttle_range_len / delta_dif * 0.99;
            min_delta *= throttle_range_len / delta_dif * 0.99;
        }

        let lim_throttle = throttle.clamp(
            self.min_throttle + min_delta.abs(),
            self.max_throttle - max_delta.abs(),
        );

        let mut motors = [0.0f32; 4];
        for i in 0..4 {
            motors[i] = lim_throttle + delta[i] * scale;
        }

        let saturated = scale < 1.0;

        (motors, saturated)
    }

    ///
    /// Made by Chatgpt, unreliable as hell. Deltas try to fit within throttle,
    /// so if throttle is the max, will simply not apply deltas. Shouldn't be used.
    ///
    /// Will actually kind of work if throttle_min + delta < throttle < throttle_max - delta,
    /// Where delta is a value of headroom added so that we always have room to create torque
    ///
    pub fn mix_throttle_authority(
        &self,
        throttle: f32,
        torque: na::Vector3<f32>,
    ) -> ([f32; 4], bool) {
        let mut delta = [0.0f32; 4];

        // 1. Torque-only contribution
        for i in 0..4 {
            delta[i] = self.motor_map[i][0] * torque.x
                + self.motor_map[i][1] * torque.y
                + self.motor_map[i][2] * torque.z;
        }

        // 2. Compute allowable scaling
        let mut scale = 1.0f32;

        for i in 0..4 {
            if delta[i] > 0.0 {
                scale = scale.min((self.max_throttle - throttle) / delta[i]);
            } else if delta[i] < 0.0 {
                scale = scale.min((self.min_throttle - throttle) / delta[i]);
            }
        }

        scale = scale.clamp(0.0, 1.0);

        // 3. Apply
        let mut motors = [0.0f32; 4];
        for i in 0..4 {
            motors[i] = throttle + scale * delta[i];
        }

        let saturated = scale < 1.0;
        (motors, saturated)
    }

    /// Bad, not mine, used for testing and comparison
    pub fn mix_no_torque_scalling(
        &self,
        throttle: f32,
        torque: na::Vector3<f32>,
    ) -> ([f32; 4], bool) {
        let mut motors = [0.0f32; 4];

        // --------------------------------------------------
        // 1. Raw mix: throttle + torque deltas
        // --------------------------------------------------
        for i in 0..4 {
            let delta = self.motor_map[i][0] * torque.x
                + self.motor_map[i][1] * torque.y
                + self.motor_map[i][2] * torque.z;

            motors[i] = throttle + delta;
        }

        // --------------------------------------------------
        // 2. Find saturation
        // --------------------------------------------------
        let mut max_motor = f32::MIN;
        let mut min_motor = f32::MAX;

        for &m in motors.iter() {
            max_motor = max_motor.max(m);
            min_motor = min_motor.min(m);
        }

        let mut saturated = false;

        // --------------------------------------------------
        // 3. Thrust-priority correction (shift down)
        //    Preserves all torque differences
        // --------------------------------------------------
        if max_motor > self.max_throttle {
            let excess = max_motor - self.max_throttle;
            for m in motors.iter_mut() {
                *m -= excess;
            }
            saturated = true;
        }

        // --------------------------------------------------
        // 4. Recompute minimum after shift
        // --------------------------------------------------
        min_motor = f32::MAX;
        for &m in motors.iter() {
            min_motor = min_motor.min(m);
        }

        // --------------------------------------------------
        // 5. Bottom correction (shift up if possible)
        //    Still preserves torque
        // --------------------------------------------------
        if min_motor < self.min_throttle {
            let deficit = self.min_throttle - min_motor;
            for m in motors.iter_mut() {
                *m += deficit;
            }
            saturated = true;
        }

        // --------------------------------------------------
        // 6. Final clamp (last resort — torque may be lost)
        // --------------------------------------------------
        for m in motors.iter_mut() {
            if *m > self.max_throttle {
                *m = self.max_throttle;
                saturated = true;
            } else if *m < self.min_throttle {
                *m = self.min_throttle;
                saturated = true;
            }
        }

        (motors, saturated)
    }
}
