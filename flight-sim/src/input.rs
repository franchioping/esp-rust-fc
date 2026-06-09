use flight_control::controller::Input;
use flight_control::controller::InputMode;

use nalgebra as na;
use std::fs::File;
use std::io::Read;
use std::io::Write;

#[derive(serde::Serialize, serde::Deserialize)]
#[serde(remote = "InputMode")]
pub enum InputModeDef {
    ACRO,
    LEVEL,
}

#[derive(serde::Serialize, serde::Deserialize)]
#[serde(remote = "Input")]
pub struct InputDef {
    #[serde(with = "InputModeDef")]
    pub mode: InputMode,

    pub inp: na::Vector4<f32>,
}

#[derive(Clone, Copy, Default, serde::Serialize, serde::Deserialize)]
pub struct InputRecord {
    #[serde(with = "InputDef")]
    input: Input,
    time: f32,
}

#[derive(Default, Clone, serde::Serialize, serde::Deserialize)]
pub struct InputRecording {
    records: Vec<InputRecord>,
}

impl InputRecording {
    pub fn save_to_file(&self, path: &str) -> Result<(), Box<dyn std::error::Error>> {
        let json = serde_json::to_string_pretty(self)?;
        let mut file = File::create(path)?;
        file.write_all(json.as_bytes())?;
        Ok(())
    }

    pub fn load_from_file(path: &str) -> Result<Self, Box<dyn std::error::Error>> {
        let mut file = File::open(path)?;
        let mut contents = String::new();
        file.read_to_string(&mut contents)?;
        let recording: Self = serde_json::from_str(&contents)?;
        Ok(recording)
    }

    pub fn has_ended(&self, time: f32) -> bool {
        return self.records.last().unwrap_or(&InputRecord::default()).time < time;
    }

    pub fn get_input(&self, time: f32) -> Input {
        /*
         * Binary search returns index to element as OK, or where the element could be placed to
         * keep order as Err, so if result is Ok return that input, if its Err, return the previous
         * input if it exists, if it doesn't (because time is before the first action in the
         * recorded sequence, return an empty input)
         */
        let res = self
            .records
            .binary_search_by(|probe| probe.time.total_cmp(&time));
        match res {
            Ok(res) => {
                return self
                    .records
                    .get(res)
                    .unwrap_or(&InputRecord::default())
                    .input;
            }
            Err(mut res) => {
                if res > 0 {
                    res -= 1;
                }
                return self
                    .records
                    .get(0.max(res))
                    .unwrap_or(&InputRecord::default())
                    .input;
            }
        }
    }

    /*
     * Current time should always be larger thant the last input records time.
     * This method is made for recording inputs in real time, not for retroactively adding
     *
     * Returns the addded Joystick Input
     */
    pub fn add_input(&mut self, current_time: f32, inp: Input) -> Input {
        let input = inp;
        let last_input = self.records.last();
        match last_input {
            Some(last_record) => {
                if last_record.input != input {
                    self.records.push(InputRecord {
                        input,
                        time: current_time,
                    });
                }
            }
            None => {
                self.records.push(InputRecord {
                    input,
                    time: current_time,
                });
            }
        }
        return input;
    }
}
