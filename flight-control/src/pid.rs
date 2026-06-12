use nalgebra as na;

pub struct PidProcessor {
    pub kp: na::Vector3<f32>,
    pub ki: na::Vector3<f32>,
    pub kd: na::Vector3<f32>,
    pub kff: na::Vector3<f32>,
    pub integral: na::Vector3<f32>,
    pub last_error: na::Vector3<f32>,
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
