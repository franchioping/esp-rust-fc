use eframe::egui;
use egui_plot::LineStyle;
use flight_sim::logger::SimLogRow;
use nalgebra::zero;
use strum_macros::EnumIter;

#[derive(Clone, Copy, PartialEq, Eq, Hash, Debug, EnumIter)]
pub enum Vec3Field {
    SensAngularVelocity,
    SensRotation,
    TargetTorque,
    TargetAngularAccel,
    TargetAngularVelocity,
    TargetRotation,
    RealTorque,
    RealAngularAccel,
    RealAngularVelocity,
    RealRotation,

    NoisedAngularVelocity,
}

impl Vec3Field {
    pub fn name(&self) -> &'static str {
        match self {
            Self::SensAngularVelocity => "Sens Angular Velocity",
            Self::SensRotation => "Sens Rotation",
            Self::TargetTorque => "Target Torque",
            Self::TargetAngularAccel => "Target Angular Accel",
            Self::TargetAngularVelocity => "Target Angular Velocity",
            Self::TargetRotation => "Target Rotation",
            Self::RealTorque => "Real Torque",
            Self::RealAngularAccel => "Real Angular Accel",
            Self::RealAngularVelocity => "Real Angular Velocity",
            Self::RealRotation => "Real Rotation",

            Self::NoisedAngularVelocity => "Noised Angular Velocity",
        }
    }

    pub fn extract<'a>(&self, row: &'a SimLogRow) -> &'a nalgebra::Vector3<f32> {
        match self {
            Self::SensAngularVelocity => &row.controller_log.sens_angular_velocty,
            Self::SensRotation => &row.controller_log.sens_rotation,
            Self::TargetTorque => &row.controller_log.target_torque,
            Self::TargetAngularVelocity => &row.controller_log.target_angular_velocty,
            Self::TargetRotation => &row.controller_log.target_rotation,
            Self::RealTorque => &row.real_torque,
            Self::RealAngularVelocity => &row.real_angular_velocty,
            Self::RealRotation => &row.real_rotation,
            Self::RealAngularAccel => &row.real_angular_accel,
            Self::TargetAngularAccel => &row.controller_log.target_angular_accel,
            Self::NoisedAngularVelocity => &row.noised_angular_velocty,
        }
    }

    /// Provides a unique base color and line style modifier per field type.
    /// This keeps the field plots visually distinct when layered.
    pub fn style_config(&self) -> (egui::Color32, LineStyle) {
        match self {
            Self::SensAngularVelocity => (
                egui::Color32::from_rgb(30, 30, 70),
                LineStyle::dotted_dense(),
            ),
            Self::SensRotation => (
                egui::Color32::from_rgb(30, 30, 70),
                LineStyle::dotted_dense(),
            ),
            Self::TargetTorque => (
                egui::Color32::from_rgb(70, 70, 30),
                LineStyle::dashed_loose(),
            ),
            Self::TargetAngularAccel => (
                egui::Color32::from_rgb(70, 30, 30),
                LineStyle::dashed_loose(),
            ),
            Self::TargetAngularVelocity => (
                egui::Color32::from_rgb(30, 70, 30),
                LineStyle::dashed_loose(),
            ),
            Self::TargetRotation => (
                egui::Color32::from_rgb(30, 30, 70),
                LineStyle::dashed_loose(),
            ),
            Self::RealTorque => (egui::Color32::from_rgb(70, 70, 30), LineStyle::Solid),
            Self::RealAngularAccel => (egui::Color32::from_rgb(70, 30, 30), LineStyle::Solid),
            Self::RealAngularVelocity => (egui::Color32::from_rgb(30, 70, 30), LineStyle::Solid),
            Self::RealRotation => (egui::Color32::from_rgb(30, 30, 70), LineStyle::Solid),

            Self::NoisedAngularVelocity => (egui::Color32::from_rgb(30, 45, 30), LineStyle::Solid),
        }
    }

    pub fn colors(&self) -> [egui::Color32; 3] {
        let (mut base_color, _) = self.style_config();
        // base_color = base_color.linear_multiply(0.5);

        let x_color = egui::Color32::from_rgba_unmultiplied(
            base_color.r().saturating_mul(3),
            base_color.g().saturating_mul(3),
            base_color.b().saturating_mul(3),
            150,
        );

        let y_color = egui::Color32::from_rgba_unmultiplied(
            base_color.r().saturating_mul(2),
            base_color.g().saturating_mul(2),
            base_color.b().saturating_mul(2),
            150,
        );

        let z_color = egui::Color32::from_rgba_unmultiplied(
            base_color.r(),
            base_color.g(),
            base_color.b(),
            150,
        );

        return [x_color, y_color, z_color];
    }
}
