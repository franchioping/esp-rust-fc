use eframe::egui;
use egui_plot::{Legend, Line, LineStyle, Plot, PlotItem, PlotPoints};
use flight_sim::logger::SimLogRow;
use std::collections::HashSet;
use strum::IntoEnumIterator;
use strum_macros::EnumIter;

#[derive(Clone, Copy, PartialEq, Eq, Hash, Debug, EnumIter)]
enum Vec3Field {
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
}

impl Vec3Field {
    fn name(&self) -> &'static str {
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
        }
    }

    fn extract<'a>(&self, row: &'a SimLogRow) -> &'a nalgebra::Vector3<f32> {
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
        }
    }

    /// Provides a unique base color and line style modifier per field type.
    /// This keeps the field plots visually distinct when layered.
    fn style_config(&self) -> (egui::Color32, LineStyle) {
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
        }
    }
}

pub struct LogPlotterApp {
    log_data: Vec<SimLogRow>,
    // Store multiple active selections instead of a single enum
    selected_fields: HashSet<Vec3Field>,
    plot_x: bool,
    plot_y: bool,
    plot_z: bool,
}

impl LogPlotterApp {
    pub fn new(log_data: Vec<SimLogRow>) -> Self {
        let mut selected_fields = HashSet::new();
        selected_fields.insert(Vec3Field::RealRotation);

        Self {
            log_data,
            selected_fields,
            plot_x: true,
            plot_y: true,
            plot_z: true,
        }
    }
}

impl eframe::App for LogPlotterApp {
    fn ui(&mut self, ui: &mut egui::Ui, _frame: &mut eframe::Frame) {
        let all_fields = Vec3Field::iter();

        egui::Panel::left("plot_controls")
            .resizable(true)
            .default_size(240.0)
            .show_inside(ui, |side_ui| {
                side_ui.heading("Log Plotter Settings");
                side_ui.separator();

                side_ui.label("Select Vector3 Fields:");

                // Use a ScrollArea in case your list of fields grows large
                egui::ScrollArea::vertical()
                    .max_height(400.0)
                    .show(side_ui, |scroll_ui| {
                        for field in all_fields {
                            let mut is_selected = self.selected_fields.contains(&field);
                            if scroll_ui.checkbox(&mut is_selected, field.name()).changed() {
                                if is_selected {
                                    self.selected_fields.insert(field);
                                } else {
                                    self.selected_fields.remove(&field);
                                }
                            }
                        }
                    });

                side_ui.add_space(10.0);
                side_ui.separator();
                side_ui.label("Coordinates to Plot:");

                side_ui.horizontal(|h_ui| {
                    if h_ui.button("All").clicked() {
                        self.plot_x = true;
                        self.plot_y = true;
                        self.plot_z = true;
                    }
                    if h_ui.button("None").clicked() {
                        self.plot_x = false;
                        self.plot_y = false;
                        self.plot_z = false;
                    }
                });

                side_ui.checkbox(&mut self.plot_x, "Plot X Axis");
                side_ui.checkbox(&mut self.plot_y, "Plot Y Axis");
                side_ui.checkbox(&mut self.plot_z, "Plot Z Axis");
            });

        egui::CentralPanel::default().show_inside(ui, |central_ui| {
            central_ui.heading("Telemetry Analytics Plot");

            let mut lines = Vec::new();

            // Iterate over all active fields to compile plot information
            for field in &self.selected_fields {
                let (mut base_color, base_style) = field.style_config();
                base_color = base_color.linear_multiply(2.0);

                let mut points_x = Vec::with_capacity(self.log_data.len());
                let mut points_y = Vec::with_capacity(self.log_data.len());
                let mut points_z = Vec::with_capacity(self.log_data.len());

                for row in &self.log_data {
                    let time = row.time as f64;
                    let vec = field.extract(row);

                    if self.plot_x {
                        points_x.push([time, vec.x as f64]);
                    }
                    if self.plot_y {
                        points_y.push([time, vec.y as f64]);
                    }
                    if self.plot_z {
                        points_z.push([time, vec.z as f64]);
                    }
                }

                // Give each Axis variant a color tint shifts so X, Y, and Z stay differentiable
                if self.plot_x {
                    let x_color = egui::Color32::from_rgba_premultiplied(
                        base_color.r().saturating_mul(3),
                        base_color.g().saturating_mul(3),
                        base_color.b().saturating_mul(3),
                        base_color.a(),
                    );
                    lines.push(
                        Line::new(format!("{}_x", field.name()), PlotPoints::from(points_x))
                            .name(format!("{} - X", field.name()))
                            .color(x_color) // Keep base color tint but distinctly X
                            .style(base_style),
                    );
                }
                if self.plot_y {
                    // Mix in some secondary channel to distinguish axis inside the same group
                    let y_color = egui::Color32::from_rgba_premultiplied(
                        base_color.r().saturating_mul(2),
                        base_color.g().saturating_mul(2),
                        base_color.b().saturating_mul(2),
                        base_color.a(),
                    );
                    lines.push(
                        Line::new(format!("{}_y", field.name()), PlotPoints::from(points_y))
                            .name(format!("{} - Y", field.name()))
                            .color(y_color)
                            .style(base_style),
                    );
                }
                if self.plot_z {
                    let z_color = egui::Color32::from_rgba_premultiplied(
                        base_color.r(),
                        base_color.g(),
                        base_color.b(),
                        base_color.a(),
                    );
                    lines.push(
                        Line::new(format!("{}_z", field.name()), PlotPoints::from(points_z))
                            .name(format!("{} - Z", field.name()))
                            .color(z_color)
                            .style(base_style),
                    );
                }
            }

            Plot::new("flight_sim_plot")
                .legend(Legend::default().position(egui_plot::Corner::LeftTop))
                .x_axis_label("Time (s)")
                .y_axis_label("Value")
                .show(central_ui, |plot_ui| {
                    for line in lines {
                        plot_ui.line(line);
                    }

                    plot_ui.vline(
                        egui_plot::VLine::new("yaxis", 0.0)
                            .color(egui::Color32::GRAY)
                            .width(2.0_f32)
                            .name(""),
                    );

                    plot_ui.hline(
                        egui_plot::HLine::new("xaxis", 0.0)
                            .color(egui::Color32::GRAY)
                            .width(2.0_f32)
                            .name(""),
                    );
                });
        });
    }
}

use eframe::NativeOptions;
use std::fs::File;
use std::io::BufReader;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!(
        "Attempting to load telemetry from: {}",
        "run-data/test.idfk"
    );

    let file = File::open("run-data/test.idfk")?;
    let reader = BufReader::new(file);

    let log_data: Vec<SimLogRow> = rmp_serde::from_read(reader)?;

    println!("Successfully loaded {} log records.", log_data.len());

    if log_data.is_empty() {
        println!("Warning: Log file is empty. Nothing to plot.");
    }

    let options = NativeOptions {
        viewport: eframe::egui::ViewportBuilder::default()
            .with_title("Flight Simulator Telemetry Viewer")
            .with_inner_size([1100.0, 700.0]),
        ..Default::default()
    };

    eframe::run_native(
        "Telemetry Viewer",
        options,
        Box::new(|_cc| Ok(Box::new(LogPlotterApp::new(log_data)))),
    )?;

    Ok(())
}
