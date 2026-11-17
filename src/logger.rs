//! Hourly capture files written in blocks.
//!
//! Each received message is added to an in-memory block. When the block holds
//! `block_size` messages it is appended to the file of the current UTC hour.
//! At hh:00:00 the old block is flushed and a new hour file is started.
//! The directory tree is checked (and created if missing) before every write.
//! A block that cannot be written is kept in memory and written again later.
//!
//! File layout: <dir>/YYYY/MM/DD/<prefix>_YYYYMMDD_HH00.<ext>

use crate::config::Settings;
use chrono::{DateTime, NaiveDateTime, Timelike, Utc};
use std::fs::{self, OpenOptions};
use std::io::Write;
use std::path::{Path, PathBuf};
use std::time::{Duration, Instant};

/// Most data kept in memory while the disk cannot be written; the oldest are dropped above it.
const MAX_WAITING: usize = 16 * 1024 * 1024;

struct HourFile {
    ext: &'static str,
    enabled: bool,
    hour: Option<NaiveDateTime>,
    buf: Vec<u8>,
    count: usize,
}

pub struct Logger {
    dir: String,
    prefix: String,
    block: usize,
    files: [HourFile; 5], // NMEA, NMEA decoded, UBX raw, UBX decoded, quality
    waiting: Vec<(PathBuf, Vec<u8>)>, // blocks that could not be written yet, oldest first
    last_try: Instant,
    lost: usize, // bytes dropped because more than MAX_WAITING were waiting
    /// The current write problem; None when everything is on disk.
    pub error: Option<String>,
}

/// Appends data to a file; the directory tree is created if it does not exist.
fn append(path: &Path, data: &[u8]) -> std::io::Result<()> {
    if let Some(parent) = path.parent() {
        fs::create_dir_all(parent)?;
    }
    OpenOptions::new()
        .create(true)
        .append(true)
        .open(path)?
        .write_all(data)
}

pub fn hour_of(t: DateTime<Utc>) -> NaiveDateTime {
    t.naive_utc()
        .with_minute(0)
        .unwrap()
        .with_second(0)
        .unwrap()
        .with_nanosecond(0)
        .unwrap()
}

impl Logger {
    pub fn new(s: &Settings) -> Logger {
        let f = |ext, enabled| HourFile {
            ext,
            enabled,
            hour: None,
            buf: Vec::new(),
            count: 0,
        };
        Logger {
            dir: s.directory.clone(),
            prefix: s.prefix.clone(),
            block: s.block_size.max(1),
            files: [
                f("nmea", s.save_nmea),
                f("nmea.txt", s.save_nmea_text),
                f("ubx", s.save_ubx),
                f("ubx.txt", s.save_ubx_text),
                f("quality.txt", s.save_quality),
            ],
            waiting: Vec::new(),
            last_try: Instant::now(),
            lost: 0,
            error: None,
        }
    }

    pub fn path(&self, hour: NaiveDateTime, ext: &str) -> PathBuf {
        PathBuf::from(&self.dir)
            .join(hour.format("%Y").to_string())
            .join(hour.format("%m").to_string())
            .join(hour.format("%d").to_string())
            .join(format!(
                "{}_{}.{}",
                self.prefix,
                hour.format("%Y%m%d_%H00"),
                ext
            ))
    }

    pub fn nmea(&mut self, now: DateTime<Utc>, line: &str) {
        self.add(0, now, format!("{line}\r\n").as_bytes());
        if self.files[1].enabled {
            self.add_text(1, now, now, &crate::nmea::decode(line));
        }
    }

    pub fn ubx(&mut self, now: DateTime<Utc>, frame: &[u8], decoded: &str) {
        self.add(2, now, frame);
        self.add_text(3, now, now, decoded);
    }

    /// Signal quality line of the second starting at `second` (receiver time when known).
    /// Like all files, the hour file is chosen by the PC time `now`.
    pub fn quality(&mut self, now: DateTime<Utc>, second: DateTime<Utc>, line: &str) {
        self.add_text(4, now, second, line);
    }

    /// Decoded text: one entry per message, prefixed with the time `stamp`.
    fn add_text(&mut self, i: usize, now: DateTime<Utc>, stamp: DateTime<Utc>, text: &str) {
        let stamp = stamp.format("%Y-%m-%dT%H:%M:%S%.3fZ");
        let line = format!("{stamp} {}\r\n", text.replace('\n', "\r\n"));
        self.add(i, now, line.as_bytes());
    }

    fn add(&mut self, i: usize, now: DateTime<Utc>, data: &[u8]) {
        if !self.files[i].enabled {
            return;
        }
        self.tick_one(i, now);
        let f = &mut self.files[i];
        f.buf.extend_from_slice(data);
        f.count += 1;
        if f.count >= self.block {
            self.flush_one(i);
        }
    }

    /// Call regularly: starts a new file exactly at the start of a new hour.
    pub fn tick(&mut self, now: DateTime<Utc>) {
        for i in 0..self.files.len() {
            self.tick_one(i, now);
        }
        // blocks that could not be written: try again every second
        if !self.waiting.is_empty() && self.last_try.elapsed() >= Duration::from_secs(1) {
            self.write_waiting();
        }
    }

    fn tick_one(&mut self, i: usize, now: DateTime<Utc>) {
        let h = hour_of(now);
        if self.files[i].hour != Some(h) {
            self.flush_one(i); // remaining data goes to the previous hour's file
            self.files[i].hour = Some(h);
        }
    }

    pub fn flush_all(&mut self) {
        for i in 0..self.files.len() {
            self.flush_one(i);
        }
        self.write_waiting();
    }

    fn flush_one(&mut self, i: usize) {
        let (Some(hour), false) = (self.files[i].hour, self.files[i].buf.is_empty()) else {
            return;
        };
        let path = self.path(hour, self.files[i].ext);
        let data = std::mem::take(&mut self.files[i].buf);
        self.files[i].count = 0;
        // the block joins the queue of its file, so the order inside a file is kept
        match self.waiting.iter_mut().find(|(p, _)| *p == path) {
            Some((_, d)) => d.extend(data),
            None => self.waiting.push((path, data)),
        }
        self.write_waiting();
    }

    /// Writes the queued blocks. What cannot be written stays queued for the next try.
    fn write_waiting(&mut self) {
        self.last_try = Instant::now();
        let mut error = None;
        self.waiting
            .retain(|(path, data)| match append(path, data) {
                Ok(()) => false,
                Err(e) => {
                    error.get_or_insert(format!("{}: {e}", path.display()));
                    true
                }
            });
        let mut size: usize = self.waiting.iter().map(|(_, d)| d.len()).sum();
        while size > MAX_WAITING {
            let (_, d) = self.waiting.remove(0);
            size -= d.len();
            self.lost += d.len();
        }
        let lost = match self.lost {
            0 => String::new(),
            n => format!(", {} kB lost", n.div_ceil(1024)),
        };
        self.error = match error {
            Some(e) => Some(format!(
                "{e} ({} kB waiting to be written{lost})",
                size.div_ceil(1024)
            )),
            None if self.lost > 0 => Some(format!("Disk problem is over{lost}")),
            None => None,
        };
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use chrono::TimeZone;

    #[test]
    fn blocks_and_hours() {
        let dir = std::env::temp_dir().join("gnssCaptureR_test");
        let _ = fs::remove_dir_all(&dir);
        let s = Settings {
            directory: dir.to_string_lossy().into(),
            block_size: 3,
            ..Settings::default()
        };
        let mut lg = Logger::new(&s);
        let t1 = Utc.with_ymd_and_hms(2026, 11, 12, 13, 59, 58).unwrap();
        let p13 = lg.path(hour_of(t1), "nmea");
        lg.nmea(t1, "$A");
        lg.nmea(t1, "$B");
        assert!(!p13.exists()); // block not full yet
        lg.nmea(t1, "$C");
        assert_eq!(fs::read_to_string(&p13).unwrap(), "$A\r\n$B\r\n$C\r\n");
        lg.nmea(t1, "$D");
        let t2 = Utc.with_ymd_and_hms(2026, 11, 12, 14, 0, 0).unwrap();
        lg.tick(t2); // new hour: $D is flushed to the 13:00 file
        assert!(fs::read_to_string(&p13).unwrap().ends_with("$D\r\n"));
        lg.nmea(t2, "$E");
        lg.flush_all();
        let p14 = lg.path(hour_of(t2), "nmea");
        assert!(p14.ends_with("2026/11/12/gps_20261112_1400.nmea"));
        assert_eq!(fs::read_to_string(&p14).unwrap(), "$E\r\n");
    }

    #[test]
    fn failed_write_is_retried() {
        // the capture directory cannot be created while a file has its name (B4)
        let base = std::env::temp_dir().join("gnssCaptureR_test_retry");
        let _ = fs::remove_dir_all(&base);
        let _ = fs::remove_file(&base);
        fs::write(&base, "in the way").unwrap();
        let s = Settings {
            directory: base.join("data").to_string_lossy().into(),
            block_size: 2,
            ..Settings::default()
        };
        let mut lg = Logger::new(&s);
        let t = Utc.with_ymd_and_hms(2026, 11, 15, 10, 59, 59).unwrap();
        lg.nmea(t, "$A");
        lg.nmea(t, "$B"); // block full: the write fails
        assert!(lg.error.as_deref().unwrap().contains("1 kB waiting"));
        lg.nmea(t + chrono::Duration::seconds(1), "$C"); // next hour: $C goes to another file
        fs::remove_file(&base).unwrap(); // the disk is back
        lg.flush_all();
        assert!(lg.error.is_none());
        let p10 = lg.path(hour_of(t), "nmea");
        assert_eq!(fs::read_to_string(p10).unwrap(), "$A\r\n$B\r\n");
        let p11 = lg.path(hour_of(t + chrono::Duration::seconds(1)), "nmea");
        assert_eq!(fs::read_to_string(p11).unwrap(), "$C\r\n");
    }
}
