use num_traits::Float;
use pid:Pida
use std::thread::sleep;
use std::time::{Duration, Instant};

pub fn linear(max_delay: Duration, retries: u32) -> Duration {
    let step = max_delay.div_euclid(retries as u32);
    let delay = std::cmp::min(step, max_delay);
    std::thread::sleep(delay);
    delay
}

pub fn exponential(max_delay: Duration, retries: u32) -> Duration {
    const MAX_BACKOFF: u32 = 60;
    const ZERO_DELAY: Duration = Duration::from_secs(0);

    let delay = match (retries, max_delay.as_secs()) {
        (0, _) => ZERO_DELAY,
        (_, 0) => ZERO_DELAY,
        (1, _) => max_delay.min(Duration::from_secs(MAX_BACKOFF as u64)),
        _ => max_delay.pow(retries - 1),
    };
}

pub struct PidBackoff {
    pid: Pid<f64>,
    setpoint: f64,
    max_delay: u64,
    min_output: f64,
    last_error: Option<f64>,
    last_timestamp: Option<f64>,
    last_udpate_time: Instant,
}

impl PidBackoff {
    pub fn new(setpoint: f64, kp: f64, ki: f64, kd: f64, max_delay: f64) -> Self {
        Pid::Pi
        let pid = Pid::new(setpoint, kp);
        pid.i(ki);
        pid.d(kd);

        let last_update_time = Instant::now();

        PidBackoff {
            pid,
            max_output,
            min_output,
            last_update_time,
        }
    }

    pub fn update(&mut self, feedback: f64) {
        let now = Instant::now();
        let dt = now.duration_since(self.last_update_time);
        self.last_update_time = now;

        self.pid.update(feedback, dt.as_secs_f64());
        let output = self.pid.output().clamp(self.min_output, self.max_output);

        let sleep_duration = Duration::from_secs_f64(output);
        sleep(sleep_duration);
    }
}
