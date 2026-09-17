use eframe::egui;
use egui_plot::{Legend, Line, Plot, PlotPoints};
use flight_sim::logger::SimLogRow;
use std::collections::HashSet;
use strum::IntoEnumIterator;

mod fields;
use fields::*;

pub struct LogPlotterApp {
    log_data: Vec<SimLogRow>,
    // Store multiple active selections instead of a single enum
    selected_fields: HashSet<Vec3Field>,
    fft_fields: HashSet<Vec3Field>,
    plot_x: bool,
    plot_y: bool,
    plot_z: bool,
}

impl LogPlotterApp {
    pub fn new(log_data: Vec<SimLogRow>) -> Self {
        let mut selected_fields = HashSet::new();
        let mut fft_fields = HashSet::new();
        selected_fields.insert(Vec3Field::NoisedAngularVelocity);
        fft_fields.insert(Vec3Field::NoisedAngularVelocity);

        Self {
            log_data,
            selected_fields,
            fft_fields,
            plot_x: true,
            plot_y: false,
            plot_z: false,
        }
    }

    pub fn plot_controls(&mut self, ui: &mut egui::Ui) {
        egui::Panel::left("plot_controls")
            .resizable(true)
            .default_size(240.0)
            .show_inside(ui, |side_ui| {
                side_ui.heading("Log Plotter Settings");
                side_ui.separator();

                side_ui.label("Select Vector3 Fields:");
                egui::ScrollArea::vertical()
                    .max_height(400.0)
                    .show(side_ui, |scroll_ui| {
                        for field in Vec3Field::iter() {
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

                side_ui.label("Use FFT?");
                egui::ScrollArea::vertical()
                    .id_salt(11111)
                    .max_height(400.0)
                    .show(side_ui, |scroll_ui| {
                        for field in Vec3Field::iter() {
                            let mut is_selected = self.fft_fields.contains(&field);
                            if scroll_ui
                                .checkbox(&mut is_selected, format!("{} FFT?", field.name()))
                                .changed()
                            {
                                if is_selected {
                                    self.fft_fields.insert(field);
                                } else {
                                    self.fft_fields.remove(&field);
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
    }

    pub fn process_field(&mut self, field: &Vec3Field, lines: &mut Vec<Line>) {
        // Toggle variable at the start of the function (adjust how you access this flag)
        let use_fft = self.fft_fields.contains(field);
        let (_, base_style) = field.style_config();

        if self.log_data.is_empty() {
            return;
        }

        // Determine data length (FFT requires a power of 2)
        let data_len = if use_fft {
            let len = self.log_data.len();
            if len < 2 {
                return;
            }
            // Find the largest power of 2 <= log_data.len()
            1_usize << (31 - (len as u32).leading_zeros())
        } else {
            self.log_data.len()
        };

        let capacity = if use_fft { data_len / 2 + 1 } else { data_len };
        let mut points_x = Vec::with_capacity(capacity);
        let mut points_y = Vec::with_capacity(capacity);
        let mut points_z = Vec::with_capacity(capacity);

        if use_fft {
            let mut values_x = Vec::with_capacity(data_len);
            let mut values_y = Vec::with_capacity(data_len);
            let mut values_z = Vec::with_capacity(data_len);

            for i in 0..data_len {
                let row = &self.log_data[i];
                let vec = field.extract(row);
                values_x.push(vec.x as f64);
                values_y.push(vec.y as f64);
                values_z.push(vec.z as f64);
            }

            let start_time = self.log_data[0].time as f64;
            let end_time = self.log_data[data_len - 1].time as f64;
            let time_per_sample = (end_time - start_time) / ((data_len - 1) as f64);

            let samplerate = if time_per_sample > 0.0 {
                1.0 / time_per_sample
            } else {
                1.0
            };

            let half_n = data_len / 2 + 1;
            let mut re = vec![0.0; half_n];
            let mut im = vec![0.0; half_n];

            // 3. Run FFT and map bins to [frequency, magnitude]
            let process_axis = |sig: &[f64],
                                points: &mut Vec<[f64; 2]>,
                                real: &mut [f64],
                                imaginary: &mut [f64]| {
                phastft::r2c_fft_f64(sig, real, imaginary);
                for k in 0..half_n {
                    let freq = (k as f64) * samplerate / (data_len as f64);

                    if freq <= 1000.0 {
                        // Normalized magnitude spectrum
                        let mag =
                            (real[k].powi(2) + imaginary[k].powi(2)).sqrt() / (data_len as f64);
                        points.push([freq, mag.ln()]);
                    }
                }
            };

            if self.plot_x {
                process_axis(&values_x, &mut points_x, &mut re, &mut im);
            }
            if self.plot_y {
                process_axis(&values_y, &mut points_y, &mut re, &mut im);
            }
            if self.plot_z {
                process_axis(&values_z, &mut points_z, &mut re, &mut im);
            }
        } else {
            for i in 0..data_len {
                let row = &self.log_data[i];
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
        }

        let suffix = if use_fft { " (FFT)" } else { "" };

        if self.plot_x {
            lines.push(
                Line::new(format!("{}_x", field.name()), PlotPoints::from(points_x))
                    .name(format!("{} - X{}", field.name(), suffix))
                    .color(field.colors()[0])
                    .width(2_f32)
                    .style(base_style),
            );
        }
        if self.plot_y {
            lines.push(
                Line::new(format!("{}_y", field.name()), PlotPoints::from(points_y))
                    .name(format!("{} - Y{}", field.name(), suffix))
                    .color(field.colors()[1])
                    .width(2_f32)
                    .style(base_style),
            );
        }
        if self.plot_z {
            lines.push(
                Line::new(format!("{}_z", field.name()), PlotPoints::from(points_z))
                    .name(format!("{} - Z{}", field.name(), suffix))
                    .color(field.colors()[2])
                    .width(2_f32)
                    .style(base_style),
            );
        }
    }
}

impl eframe::App for LogPlotterApp {
    fn ui(&mut self, ui: &mut egui::Ui, _frame: &mut eframe::Frame) {
        let old_selected = self.selected_fields.clone();
        let old_fft = self.fft_fields.clone();
        let old_x = self.plot_x;
        let old_y = self.plot_y;
        let old_z = self.plot_z;

        self.plot_controls(ui);

        let controls_changed = old_selected != self.selected_fields
            || old_fft != self.fft_fields
            || old_x != self.plot_x
            || old_y != self.plot_y
            || old_z != self.plot_z;

        egui::CentralPanel::default().show_inside(ui, |central_ui| {
            central_ui.heading("Telemetry Analytics Plot");

            let mut lines = Vec::new();

            for field in &self.selected_fields.clone() {
                self.process_field(field, &mut lines);
            }

            Plot::new("flight_sim_plot")
                .legend(Legend::default().position(egui_plot::Corner::LeftTop))
                .x_axis_label("Time (s) / Freq (Hz)")
                .y_axis_label("Value")
                .show(central_ui, |plot_ui| {
                    if controls_changed {
                        plot_ui.set_auto_bounds(true);
                    }

                    for line in lines {
                        plot_ui.line(line);
                    }

                    plot_ui.vline(
                        egui_plot::VLine::new("yaxis", 0.0)
                            .color(egui::Color32::from_rgba_unmultiplied(160, 160, 160, 100))
                            .width(2.0_f32)
                            .name(""),
                    );

                    plot_ui.hline(
                        egui_plot::HLine::new("xaxis", 0.0)
                            .color(egui::Color32::from_rgba_unmultiplied(160, 160, 160, 100))
                            .width(2.0_f32)
                            .name(""),
                    );
                });
        });
    }
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    use eframe::NativeOptions;
    use std::fs::File;
    use std::io::BufReader;

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
