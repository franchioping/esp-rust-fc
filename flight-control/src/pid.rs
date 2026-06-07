use nalgebra as na;

pub struct PidProcessor {
    kp: na::Vector3<f32>,
    ki: na::Vector3<f32>,
    kd: na::Vector3<f32>,
    kff: na::Vector3<f32>,
    integral: na::Vector3<f32>,
    last_error: na::Vector3<f32>,
}

impl PidProcessor {
    pub fn update(
        &mut self,
        target: na::Vector3<f32>,
        current: na::Vector3<f32>,
        dt: f32,
    ) -> na::Vector3<f32> {
        let error = target - current;
        self.integral += error * dt;
        let derivative = (error - self.last_error) / dt;
        self.last_error = error;

        return error.component_mul(&self.kp)
            + self.integral.component_mul(&self.ki)
            + derivative.component_mul(&self.kd)
            + target.component_mul(&self.kff);
    }
}
