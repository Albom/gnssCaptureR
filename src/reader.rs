//! Serial port reading thread: splits the byte stream into NMEA sentences and
//! UBX frames, writes them to capture files and updates the shared state.
//! After a read error the port is reopened automatically.

use crate::config::Settings;
use crate::logger::Logger;
use crate::{nmea, ubx};
use chrono::{DateTime, NaiveDate, Utc};
use serialport::{DataBits, Parity, SerialPort, StopBits};
use std::collections::VecDeque;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex, MutexGuard};
use std::time::{Duration, Instant};

pub const STREAM_LINES: usize = 1000;

/// Data shared between the reading thread and the window.
#[derive(Default)]
pub struct Shared {
    pub state: nmea::GpsState,
    pub stream: VecDeque<String>,
    pub nmea_count: u64,
    pub ubx_count: u64,
    pub bad_count: u64,
    pub error: Option<String>,      // serial port problem
    pub file_error: Option<String>, // capture file problem
    pub running: bool,
    pub reconnecting: bool,       // the port was lost and is being reopened
    pub last_rx: Option<Instant>, // when the last message was received
}

/// Locks shared data. A panic in another thread must not stop this one too.
pub fn lock<T>(m: &Mutex<T>) -> MutexGuard<'_, T> {
    m.lock().unwrap_or_else(|e| e.into_inner())
}

impl Shared {
    fn push_line(&mut self, s: String) {
        if self.stream.len() >= STREAM_LINES {
            self.stream.pop_front();
        }
        self.stream.push_back(s);
    }
}

pub enum Frame {
    Nmea(String),
    Ubx(Vec<u8>),
}

/// Collects incoming bytes and cuts complete messages out of them.
#[derive(Default)]
pub struct Splitter {
    buf: Vec<u8>,
}

impl Splitter {
    pub fn push(&mut self, data: &[u8]) {
        self.buf.extend_from_slice(data);
    }

    pub fn next(&mut self) -> Option<Frame> {
        loop {
            // skip everything up to the start of a message
            let Some(start) = self
                .buf
                .iter()
                .position(|&b| b == b'$' || b == b'!' || b == 0xB5)
            else {
                self.buf.clear();
                return None;
            };
            self.buf.drain(..start);

            if self.buf[0] == 0xB5 {
                if self.buf.len() < 6 {
                    return None;
                }
                let len = u16::from_le_bytes([self.buf[4], self.buf[5]]) as usize;
                if self.buf[1] != 0x62 || len > 8192 {
                    self.buf.drain(..1);
                    continue;
                }
                if self.buf.len() < len + 8 {
                    return None;
                }
                if ubx::checksum_ok(&self.buf[..len + 8]) {
                    return Some(Frame::Ubx(self.buf.drain(..len + 8).collect()));
                }
                self.buf.drain(..1);
                continue;
            }

            // NMEA: printable ASCII up to '\n'
            let end = self.buf.iter().position(|&b| b == b'\n');
            // a new sentence starts inside this one (its line end was lost):
            // return the damaged beginning alone, so the next sentence stays intact
            let inner = self.buf[1..].iter().position(|&b| b == b'$' || b == b'!');
            if let Some(p) = inner.map(|p| p + 1).filter(|&p| end.is_none_or(|e| p < e)) {
                let line: Vec<u8> = self.buf.drain(..p).collect();
                return Some(Frame::Nmea(
                    String::from_utf8_lossy(&line).trim().to_string(),
                ));
            }
            let bad = self
                .buf
                .iter()
                .skip(1)
                .position(|&b| !(32..127).contains(&b) && b != b'\r' && b != b'\n');
            if let Some(bad) = bad.map(|p| p + 1).filter(|&p| end.is_none_or(|e| p < e)) {
                self.buf.drain(..bad); // binary data inside a sentence: resync there
                continue;
            }
            match end {
                Some(e) if e <= 120 => {
                    let line: Vec<u8> = self.buf.drain(..=e).collect();
                    let s = String::from_utf8_lossy(&line).trim().to_string();
                    return Some(Frame::Nmea(s));
                }
                None if self.buf.len() <= 120 => return None,
                _ => {
                    self.buf.drain(..1); // too long to be NMEA
                }
            }
        }
    }
}

/// Decides where one "second" of the signal quality file ends.
/// A second ends when the receiver's time moves to the next second, so the satellites
/// of one receiver epoch are never split between two lines. While the receiver
/// reports no time (e.g. before the first fix) the PC clock is used instead.
struct QualityCut {
    cur: Option<(u32, String)>, // receiver second of the day and date being collected
    last_cut: Instant,          // last cut made by receiver time
    pc_second: i64,
}

impl QualityCut {
    fn new(now: DateTime<Utc>) -> QualityCut {
        QualityCut {
            cur: None,
            last_cut: Instant::now(),
            pc_second: now.timestamp(),
        }
    }

    /// Call after every message. Returns the finished line and the second it belongs to.
    fn on_message(
        &mut self,
        st: &mut nmea::GpsState,
        now: DateTime<Utc>,
    ) -> Option<(DateTime<Utc>, String)> {
        let new = st.epoch?;
        if let Some((epoch, date)) = &mut self.cur {
            if *epoch == new {
                if *date != st.utc_date {
                    *date = st.utc_date.clone(); // the date may arrive after the time
                }
                return None;
            }
        }
        let second = match self.cur.take() {
            Some((epoch, date)) => receiver_time(epoch, &date, now),
            None => whole_second(now.timestamp()),
        };
        self.cur = Some((new, st.utc_date.clone()));
        self.last_cut = Instant::now();
        st.quality_line().map(|line| (second.unwrap_or(now), line))
    }

    /// The second being collected when capture stops.
    fn current(&self, now: DateTime<Utc>) -> DateTime<Utc> {
        let receiver = self
            .cur
            .as_ref()
            .filter(|_| self.last_cut.elapsed() < Duration::from_millis(1500));
        receiver
            .and_then(|(epoch, date)| receiver_time(*epoch, date, now))
            .unwrap_or(now)
    }

    /// Call regularly. True when a second of the PC clock has ended while the
    /// receiver reports no time; returns that second.
    fn pc_second_ended(&mut self, now: DateTime<Utc>) -> Option<DateTime<Utc>> {
        let ended = self.pc_second;
        if now.timestamp() == ended {
            return None;
        }
        self.pc_second = now.timestamp();
        if self.last_cut.elapsed() < Duration::from_millis(1500) {
            return None; // the receiver's time is in charge
        }
        whole_second(ended)
    }
}

fn whole_second(timestamp: i64) -> Option<DateTime<Utc>> {
    DateTime::from_timestamp(timestamp, 0)
}

/// Receiver second of the day + date ("YYYY-MM-DD", or the PC date when not known yet).
fn receiver_time(epoch: u32, date: &str, now: DateTime<Utc>) -> Option<DateTime<Utc>> {
    let date = NaiveDate::parse_from_str(date, "%Y-%m-%d").unwrap_or(now.date_naive());
    Some(
        date.and_hms_opt(epoch / 3600, epoch / 60 % 60, epoch % 60)?
            .and_utc(),
    )
}

fn open(cfg: &Settings) -> serialport::Result<Box<dyn SerialPort>> {
    let parity = match cfg.parity.as_str() {
        "Odd" => Parity::Odd,
        "Even" => Parity::Even,
        _ => Parity::None,
    };
    let data_bits = match cfg.data_bits {
        5 => DataBits::Five,
        6 => DataBits::Six,
        7 => DataBits::Seven,
        _ => DataBits::Eight,
    };
    let stop_bits = if cfg.stop_bits == 2 {
        StopBits::Two
    } else {
        StopBits::One
    };
    let port = serialport::new(&cfg.port, cfg.baud)
        .data_bits(data_bits)
        .parity(parity)
        .stop_bits(stop_bits)
        .timeout(Duration::from_millis(100))
        .open();
    if let Ok(p) = &port {
        // drop anything the driver buffered before the port was opened: old messages
        // would be logged with the present time and spoil the time difference
        let _ = p.clear(serialport::ClearBuffer::Input);
    }
    port
}

const RECONNECT_EVERY: Duration = Duration::from_secs(2);
/// A port that delivers nothing for this long is reopened too: some systems report
/// an unplugged receiver only as silence.
const SILENCE: Duration = Duration::from_secs(10);

pub fn run(cfg: Settings, shared: Arc<Mutex<Shared>>, stop: Arc<AtomicBool>) {
    // the first opening must succeed; later the port is reopened automatically
    let mut port = match open(&cfg) {
        Ok(p) => Some(p),
        Err(e) => {
            let mut sh = lock(&shared);
            sh.error = Some(format!("Cannot open {}: {e}", cfg.port));
            sh.running = false;
            return;
        }
    };

    let mut logger = Logger::new(&cfg);
    let mut splitter = Splitter::default();
    let mut buf = [0u8; 4096];
    let mut cut = QualityCut::new(Utc::now());
    let mut retry_at = Instant::now();
    let mut last_bytes = Instant::now();
    let mut file_error: Option<String> = None;

    while !stop.load(Ordering::Relaxed) {
        if let Some(p) = port.as_mut() {
            let result = p.read(&mut buf);
            match result {
                Ok(0) => std::thread::sleep(Duration::from_millis(10)),
                Ok(n) => {
                    splitter.push(&buf[..n]);
                    last_bytes = Instant::now();
                }
                Err(e)
                    if matches!(
                        e.kind(),
                        std::io::ErrorKind::TimedOut | std::io::ErrorKind::Interrupted
                    ) => {}
                Err(e) => {
                    // port lost (cable, USB): keep what was received and reopen
                    port = None;
                    retry_at = Instant::now() + RECONNECT_EVERY;
                    splitter = Splitter::default();
                    logger.flush_all();
                    let mut sh = lock(&shared);
                    sh.error = Some(format!("{}: {e}. Reconnecting…", cfg.port));
                    sh.reconnecting = true;
                }
            }
        } else if Instant::now() >= retry_at {
            match open(&cfg) {
                Ok(p) => {
                    port = Some(p);
                    last_bytes = Instant::now();
                    let mut sh = lock(&shared);
                    sh.error = None;
                    sh.reconnecting = false;
                }
                Err(_) => retry_at = Instant::now() + RECONNECT_EVERY,
            }
        } else {
            std::thread::sleep(Duration::from_millis(100));
        }

        if port.is_some() && last_bytes.elapsed() >= SILENCE {
            port = None; // closed here, reopened on the next turn
            retry_at = Instant::now();
            let mut sh = lock(&shared);
            sh.error = Some(format!("{}: no data, reopening the port…", cfg.port));
            sh.reconnecting = true;
        }

        while let Some(frame) = splitter.next() {
            let now = Utc::now();
            // files first, then the shared data: the window never waits for the disk
            let line = match frame {
                Frame::Nmea(line) => {
                    logger.nmea(now, &line);
                    let mut sh = lock(&shared);
                    sh.last_rx = Some(Instant::now());
                    if nmea::valid(&line) {
                        sh.nmea_count += 1;
                        nmea::parse(&line, &mut sh.state);
                    } else {
                        sh.bad_count += 1;
                    }
                    sh.push_line(line);
                    cut.on_message(&mut sh.state, now)
                }
                Frame::Ubx(f) => {
                    let text = ubx::decode(&f);
                    logger.ubx(now, &f, &text);
                    let mut sh = lock(&shared);
                    sh.last_rx = Some(Instant::now());
                    ubx::apply(&f, &mut sh.state);
                    sh.ubx_count += 1;
                    cut.on_message(&mut sh.state, now)
                }
            };
            if let Some((second, line)) = line {
                logger.quality(now, second, &line);
            }
        }

        let now = Utc::now();
        if let Some(second) = cut.pc_second_ended(now) {
            let line = lock(&shared).state.quality_line();
            if let Some(line) = line {
                logger.quality(now, second, &line);
            }
        }
        logger.tick(now);
        if logger.error != file_error {
            file_error = logger.error.clone();
            lock(&shared).file_error = file_error.clone();
        }
    }

    // the satellites of the last, unfinished second
    let now = Utc::now();
    let line = lock(&shared).state.quality_line();
    if let Some(line) = line {
        logger.quality(now, cut.current(now), &line);
    }
    logger.flush_all();
    let mut sh = lock(&shared);
    sh.file_error = logger.error.take();
    sh.running = false;
    sh.reconnecting = false;
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn split_mixed_stream() {
        let mut ubx = vec![0xB5, 0x62, 0x05, 0x01, 0x02, 0x00, 0x06, 0x01];
        let (mut a, mut b) = (0u8, 0u8);
        for &x in &ubx[2..] {
            a = a.wrapping_add(x);
            b = b.wrapping_add(a);
        }
        ubx.extend([a, b]);
        let mut sp = Splitter::default();
        sp.push(b"junk$GPGGA,1*00\r\n");
        sp.push(&ubx[..5]);
        assert!(matches!(sp.next(), Some(Frame::Nmea(s)) if s == "$GPGGA,1*00"));
        assert!(sp.next().is_none()); // UBX frame incomplete
        sp.push(&ubx[5..]);
        sp.push(b"$GPRMC,2\r\n");
        let Some(Frame::Ubx(f)) = sp.next() else {
            panic!()
        };
        assert_eq!(ubx::decode(&f), "ACK-ACK len=2 for CFG-MSG");
        assert!(matches!(sp.next(), Some(Frame::Nmea(s)) if s == "$GPRMC,2"));
    }

    #[test]
    fn merged_sentences() {
        // the line end of the first sentence was lost: the second one must survive (B9)
        let mut sp = Splitter::default();
        sp.push(b"$GPGGA,12$GPRMC,1*56\r\n");
        assert!(matches!(sp.next(), Some(Frame::Nmea(s)) if s == "$GPGGA,12"));
        assert!(matches!(sp.next(), Some(Frame::Nmea(s)) if s == "$GPRMC,1*56" && nmea::valid(&s)));
        assert!(sp.next().is_none());
    }

    fn cs(body: &str) -> String {
        format!("${body}*{:02X}", body.bytes().fold(0u8, |a, c| a ^ c))
    }

    #[test]
    fn quality_lines_follow_receiver_seconds() {
        // B8: GSV of one receiver second stay on one line, stamped with receiver time
        use chrono::TimeZone;
        let pc = Utc.with_ymd_and_hms(2026, 11, 15, 12, 0, 0).unwrap();
        let mut st = nmea::GpsState::default();
        let mut cut = QualityCut::new(pc);
        let mut lines = Vec::new();
        let mut feed = |body: &str, st: &mut nmea::GpsState| {
            nmea::parse(&cs(body), st);
            lines.extend(cut.on_message(st, pc));
        };
        for (time, date) in [
            ("235958.00", "151126"),
            ("235959.00", "151126"),
            ("000000.00", "161126"),
        ] {
            feed(
                &format!("GPGGA,{time},4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,"),
                &mut st,
            );
            feed(
                &format!("GPRMC,{time},A,4807.038,N,01131.000,E,0.0,0.0,{date},,"),
                &mut st,
            );
            feed(
                "GPGSV,2,1,05,01,40,083,46,02,17,308,41,12,07,344,39,14,22,228,45",
                &mut st,
            );
            feed("GPGSV,2,2,05,15,10,100,30", &mut st);
        }
        let text: Vec<String> = lines
            .iter()
            .map(|(t, l)| format!("{} {}", t.format("%F %T"), &l[..8]))
            .collect();
        // the date of 23:59:59 is the old one although the next RMC already carries the new date
        assert_eq!(
            text,
            [
                "2026-11-15 23:59:58 inView=5",
                "2026-11-15 23:59:59 inView=5"
            ]
        );
    }
}
