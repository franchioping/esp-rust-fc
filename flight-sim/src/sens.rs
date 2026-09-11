use nalgebra::{self as na};

use rand_distr::{Distribution, Normal};

#[derive(Debug, Clone)]
pub struct SensorErrorParams {
    pub bias: na::Vector3<f32>,

    // Sensor Unit / s / sqrt(Hz)
    pub bias_drift_rate: na::Vector3<f32>,

    pub scale_factors: na::Vector3<f32>,

    /*   x y z -> axis
     *  x 1
     *  y a 1
     *  z b   1
     *  |
     *  cross talk
     *
     *  a and b would be y's and z's effect on x, respectively
     */
    pub cross_talk: na::Matrix3<f32>,

    pub random_noise_distrib: Normal<f32>,

    // Quantization
    pub resolution: f32,
}

impl Default for SensorErrorParams {
    fn default() -> Self {
        Self {
            bias: na::Vector3::<f32>::zeros(),
            bias_drift_rate: na::Vector3::<f32>::zeros(),
            scale_factors: na::vector![1.0, 1.0, 1.0],
            cross_talk: na::Matrix3::zeros(),
            random_noise_distrib: Normal::new(0.0, 0.0).unwrap(),
            resolution: 0.0001,
        }
    }
}

impl SensorErrorParams {
    fn update_bias_drift(&mut self, dt: f32, rng: &mut impl rand::Rng) {
        // The standard deviation of a random walk scales with the square root of time.
        let mut drift = na::Vector3::zeros();
        for i in 0..3 {
            let std_dev = self.bias_drift_rate[i] * dt.sqrt();
            if std_dev > 0.0 {
                let normal = Normal::new(0.0, std_dev).unwrap();
                drift[i] = normal.sample(rng);
            }
        }

        self.bias += drift;
    }

    pub fn apply_error(&mut self, sensor_read: &mut na::Vector3<f32>, dt: f32) {
        self.update_bias_drift(dt, &mut rand::rng());

        *sensor_read += self.bias;

        *sensor_read = sensor_read.component_mul(&self.scale_factors);

        for i in 0..=2 {
            let mut biases: na::Vector3<f32> = self.cross_talk.column(i).into();
            biases[i] = 1.0;

            sensor_read[i] = biases.component_mul(sensor_read).iter().sum();
        }

        for i in 0..=2 {
            sensor_read[i] += self.random_noise_distrib.sample(&mut rand::rng());
        }

        for i in 0..=2 {
            sensor_read[i] = (sensor_read[i] / self.resolution).round() * self.resolution;
        }
    }
}
