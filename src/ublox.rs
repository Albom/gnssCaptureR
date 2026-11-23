//! u-blox receivers: detection (UBX-MON-VER poll) and configuration
//! (UBX output on, NMEA kept, baud rate 115200, saved). See UBX.md.

use crate::nmea;
use crate::reader::{Frame, Splitter};
use serialport::SerialPort;
use std::thread::sleep;
use std::time::{Duration, Instant};

/// Baud rates tried when looking for a receiver, most common first.
const PROBE_BAUDS: [u32; 9] = [
    9600, 115200, 38400, 57600, 230400, 460800, 921600, 19200, 4800,
];
/// Baud rate set in the receiver.
pub const TARGET_BAUD: u32 = 115200;
/// Period of the navigation solution set in the receiver: 100 ms = 10 Hz.
/// The navigation rate stays 1, so one solution is computed per measurement.
pub const MEAS_MS: u16 = 100;
/// Periods tried when the receiver refuses the one asked for.
const MEAS_FALLBACK: [u16; 3] = [200, 500, 1000];

#[derive(Clone)]
pub struct Receiver {
    pub model: String, // e.g. "U-Blox 6", "U-Blox NEO-M8N"
    pub hw: String,    // hwVersion from MON-VER, e.g. "00040007"
    pub protver: f32,  // protocol version (0 if not reported)
}

impl Receiver {
    /// u-blox F9 and M10 (protocol 27+) are configured with CFG-VALSET, older ones with CFG-PRT/CFG-MSG.
    fn uses_valset(&self) -> bool {
        self.protver >= 27.0 || self.hw == "00190000" || self.hw == "000A0000"
    }

    /// Shortest solution period this generation can do, from the data sheets:
    /// u-blox 5 up to 4 Hz, u-blox 6 up to 5 Hz, u-blox 7 and later at least 10 Hz.
    fn min_meas_ms(&self) -> u16 {
        match self.hw.as_str() {
            "00040007" => 200,                 // u-blox 6
            h if h.starts_with("0004") => 250, // u-blox 5
            _ => 100,
        }
    }
}

/// Result of checking one serial port.
#[derive(Clone, Default)]
pub struct Probe {
    pub ublox: Option<Receiver>,
    pub baud: Option<u32>, // baud rate at which valid NMEA / UBX data were received
}

/// Builds a UBX frame: B5 62 class id length payload ck_a ck_b.
pub fn frame(class: u8, id: u8, payload: &[u8]) -> Vec<u8> {
    let mut f = vec![0xB5, 0x62, class, id];
    f.extend((payload.len() as u16).to_le_bytes());
    f.extend(payload);
    let (mut a, mut b) = (0u8, 0u8);
    for &x in &f[2..] {
        a = a.wrapping_add(x);
        b = b.wrapping_add(a);
    }
    f.extend([a, b]);
    f
}

fn text(b: &[u8]) -> String {
    String::from_utf8_lossy(b)
        .trim_end_matches('\0')
        .trim()
        .to_string()
}

/// Model and versions from a MON-VER frame.
fn parse_monver(f: &[u8]) -> Receiver {
    let p = &f[6..f.len() - 2];
    let hw = text(p.get(30..40).unwrap_or(&[]));
    let ext: Vec<String> = p.get(40..).unwrap_or(&[]).chunks(30).map(text).collect();
    let protver = ext
        .iter()
        .find_map(|e| e.strip_prefix("PROTVER"))
        .and_then(|v| v.trim_start_matches(['=', ' ']).trim().parse().ok())
        .unwrap_or(0.0);
    let model = match ext.iter().find_map(|e| e.strip_prefix("MOD=")) {
        Some(m) => format!("U-Blox {m}"),
        None => match hw.as_str() {
            "00040005" => "U-Blox 5".into(),
            "00040007" => "U-Blox 6".into(),
            "00070000" => "U-Blox 7".into(),
            "00080000" => "U-Blox M8".into(),
            "00190000" => "U-Blox F9".into(),
            "000A0000" => "U-Blox M10".into(),
            _ => format!("U-Blox (hw {hw})"),
        },
    };
    Receiver { model, hw, protver }
}

/// Reads incoming messages until `found` returns true or `time` has passed.
fn read_until(
    port: &mut dyn SerialPort,
    sp: &mut Splitter,
    time: Duration,
    mut found: impl FnMut(&Frame) -> bool,
) -> bool {
    let start = Instant::now();
    let mut buf = [0u8; 1024];
    while start.elapsed() < time {
        if let Ok(n) = port.read(&mut buf) {
            sp.push(&buf[..n]);
        }
        while let Some(f) = sp.next() {
            if found(&f) {
                return true;
            }
        }
    }
    false
}

/// Sends a CFG command and waits for ACK-ACK (true) or ACK-NAK / timeout (false).
fn command(port: &mut dyn SerialPort, id: u8, payload: &[u8]) -> bool {
    let _ = port.write_all(&frame(0x06, id, payload));
    let mut acked = false;
    read_until(
        port,
        &mut Splitter::default(),
        Duration::from_millis(1500),
        |f| match f {
            Frame::Ubx(u) if u[2] == 0x05 && u.get(6..8) == Some(&[0x06, id]) => {
                acked = u[3] == 0x01;
                true
            }
            _ => false,
        },
    );
    acked
}

/// Polls the settings of the port the receiver is talking to us on (CFG-PRT, 20 bytes).
fn poll_port(port: &mut dyn SerialPort) -> Option<Vec<u8>> {
    let _ = port.write_all(&frame(0x06, 0x00, &[]));
    let mut answer = None;
    read_until(
        port,
        &mut Splitter::default(),
        Duration::from_millis(1500),
        |f| match f {
            Frame::Ubx(u) if u[2] == 0x06 && u[3] == 0x00 && u.len() == 28 => {
                answer = Some(u[6..26].to_vec());
                true
            }
            _ => false,
        },
    );
    answer
}

/// Polls MON-VER and waits for the answer.
fn poll_version(port: &mut dyn SerialPort, time: Duration, valid: &mut bool) -> Option<Receiver> {
    let _ = port.clear(serialport::ClearBuffer::Input);
    let _ = port.write_all(&frame(0x0A, 0x04, &[]));
    let mut rx = None;
    let mut sp = Splitter::default();
    read_until(port, &mut sp, time, |f| match f {
        Frame::Nmea(l) => {
            *valid |= l.contains('*') && nmea::checksum_ok(l);
            false
        }
        Frame::Ubx(u) => {
            *valid = true;
            if u[2] == 0x0A && u[3] == 0x04 {
                rx = Some(parse_monver(u));
            }
            rx.is_some()
        }
    });
    rx
}

/// Checks a serial port: tries the baud rates and asks for the receiver version.
pub fn probe(name: &str) -> Probe {
    let mut result = Probe::default();
    let Ok(mut port) = serialport::new(name, PROBE_BAUDS[0])
        .timeout(Duration::from_millis(50))
        .open()
    else {
        return result;
    };
    for baud in PROBE_BAUDS {
        if port.set_baud_rate(baud).is_err() {
            continue;
        }
        sleep(Duration::from_millis(50));
        let mut valid = false;
        // a receiver at this baud rate answers within ~1 s (it may be busy sending NMEA)
        let mut rx = poll_version(port.as_mut(), Duration::from_millis(700), &mut valid);
        if rx.is_none() && valid {
            rx = poll_version(port.as_mut(), Duration::from_millis(1500), &mut valid);
        }
        if valid && result.baud.is_none() {
            result.baud = Some(baud);
        }
        if rx.is_some() {
            result = Probe {
                ublox: rx,
                baud: Some(baud),
            };
            break;
        }
        if valid {
            break; // NMEA receiver, but not u-blox
        }
    }
    result
}

/// Messages switched on, per receiver generation: (class, id, every solution?).
/// `true` runs at the full solution rate (10 Hz), `false` once per second.
fn legacy_messages(rx: &Receiver) -> &'static [(u8, u8, bool)] {
    if rx.hw.starts_with("0004") {
        // u-blox 5 / 6 have no NAV-PVT; POSLLH + VELNED + TIMEUTC carry the same data
        &[
            (1, 0x02, true),  // NAV-POSLLH
            (1, 0x12, true),  // NAV-VELNED
            (1, 0x21, true),  // NAV-TIMEUTC
            (1, 0x03, false), // NAV-STATUS
            (1, 0x04, false), // NAV-DOP
            (1, 0x22, false), // NAV-CLOCK
            (1, 0x30, false), // NAV-SVINFO
            (2, 0x10, false), // RXM-RAW
            (2, 0x11, false), // RXM-SFRB
        ]
    } else if rx.hw == "00070000" {
        // u-blox 7: PVT DOP TIMEUTC CLOCK SVINFO
        &[
            (1, 0x07, true),
            (1, 0x04, false),
            (1, 0x21, false),
            (1, 0x22, false),
            (1, 0x30, false),
        ]
    } else {
        // u-blox M8: PVT DOP TIMEUTC CLOCK SAT, RXM-RAWX SFRBX
        &[
            (1, 0x07, true),
            (1, 0x04, false),
            (1, 0x21, false),
            (1, 0x22, false),
            (1, 0x35, false),
            (2, 0x15, false),
            (2, 0x13, false),
        ]
    }
}

/// NMEA messages whose rate is set (class 0xF0): id, every solution?
/// GGA and RMC carry position and speed, the others are only needed once per second.
const NMEA_MSGS: [(u8, bool); 6] = [
    (0x00, true),  // GGA
    (0x04, true),  // RMC
    (0x02, false), // GSA
    (0x03, false), // GSV
    (0x01, false), // GLL
    (0x05, false), // VTG
];

/// Output rate keys (UART1, USB, every solution?) for F9 / M10:
/// UBX NAV-PVT DOP TIMEUTC CLOCK SAT, RXM-RAWX SFRBX, then NMEA GGA RMC GSA GSV GLL VTG.
const MSGOUT_KEYS: [(u32, u32, bool); 13] = [
    (0x20910007, 0x20910009, true),  // NAV-PVT
    (0x20910039, 0x2091003b, false), // NAV-DOP
    (0x2091005c, 0x2091005e, false), // NAV-TIMEUTC
    (0x20910066, 0x20910068, false), // NAV-CLOCK
    (0x20910016, 0x20910018, false), // NAV-SAT
    (0x209102a5, 0x209102a7, false), // RXM-RAWX
    (0x20910232, 0x20910234, false), // RXM-SFRBX
    (0x209100bb, 0x209100bd, true),  // NMEA GGA
    (0x209100ac, 0x209100ae, true),  // NMEA RMC
    (0x209100c0, 0x209100c2, false), // NMEA GSA
    (0x209100c5, 0x209100c7, false), // NMEA GSV
    (0x209100ca, 0x209100cc, false), // NMEA GLL
    (0x209100b1, 0x209100b3, false), // NMEA VTG
];

/// CFG-RATE-MEAS and CFG-RATE-NAV (F9 / M10).
const RATE_MEAS_KEY: u32 = 0x30210001;
const RATE_NAV_KEY: u32 = 0x30210002;

/// CFG-VALSET payload with one key; the value size follows from the key ID.
fn valset_cmd(layers: u8, key: u32, value: u32) -> Vec<u8> {
    let mut p = vec![0, layers, 0, 0];
    p.extend(key.to_le_bytes());
    let size = match (key >> 28) & 7 {
        3 => 2,
        4 => 4,
        _ => 1,
    };
    p.extend(&value.to_le_bytes()[..size]);
    p
}

/// Polls CFG-RATE and returns its timeRef (0 = UTC, 1 = GPS), so it can be kept.
fn poll_time_ref(port: &mut dyn SerialPort) -> Option<u16> {
    let _ = port.write_all(&frame(0x06, 0x08, &[]));
    let mut answer = None;
    read_until(
        port,
        &mut Splitter::default(),
        Duration::from_millis(1500),
        |f| match f {
            Frame::Ubx(u) if u[2] == 0x06 && u[3] == 0x08 && u.len() == 14 => {
                answer = Some(u16::from_le_bytes([u[10], u[11]]));
                true
            }
            _ => false,
        },
    );
    answer
}

/// Sets the solution period, trying longer ones if the receiver refuses.
/// `layers` is Some for CFG-VALSET receivers. Returns the period that was accepted.
fn set_rate(port: &mut dyn SerialPort, layers: Option<u8>, want: u16) -> u16 {
    let time_ref = match layers {
        Some(_) => 1,
        None => poll_time_ref(port).unwrap_or(1), // keep the receiver's own setting
    };
    let tries = [want]
        .into_iter()
        .chain(MEAS_FALLBACK.into_iter().filter(|&m| m > want));
    for ms in tries {
        let ok = match layers {
            Some(l) => {
                command(port, 0x8A, &valset_cmd(l, RATE_MEAS_KEY, ms as u32))
                    && command(port, 0x8A, &valset_cmd(l, RATE_NAV_KEY, 1))
            }
            None => {
                // CFG-RATE: measRate, navRate = 1 solution per measurement, timeRef
                let mut p = ms.to_le_bytes().to_vec();
                p.extend(1u16.to_le_bytes());
                p.extend(time_ref.to_le_bytes());
                command(port, 0x08, &p)
            }
        };
        if ok {
            return ms;
        }
    }
    1000
}

/// The rate the receiver really delivers (Hz), from the time between the first and the
/// last GGA of a three-second look. Data already waiting are dropped first, so a backlog
/// in the driver's buffer cannot make the result too high.
fn measure_rate(port: &mut dyn SerialPort) -> Option<f64> {
    let _ = port.clear(serialport::ClearBuffer::Input);
    let (mut first, mut last, mut n) = (None, Instant::now(), 0u32);
    read_until(
        port,
        &mut Splitter::default(),
        Duration::from_secs(3),
        |f| {
            if matches!(f, Frame::Nmea(l) if l.contains("GGA")) {
                first.get_or_insert_with(Instant::now);
                last = Instant::now();
                n += 1;
            }
            false
        },
    );
    let seconds = (last - first?).as_secs_f64();
    (n > 1 && seconds > 0.5).then(|| (n - 1) as f64 / seconds)
}

/// Switches UBX output on (NMEA stays on), sets 115200 baud, 100 ms solutions and the
/// message rates, and saves the configuration.
/// `baud` is the current baud rate, `usb` is true for a native u-blox USB port (no baud rate).
/// Returns the baud rate to use and a status text.
pub fn configure(name: &str, rx: &Receiver, baud: u32, usb: bool) -> Result<(u32, String), String> {
    let mut port = serialport::new(name, baud)
        .timeout(Duration::from_millis(50))
        .open()
        .map_err(|e| format!("{name}: {e}"))?;
    let port = port.as_mut();
    let mut saved = "not saved";
    let mut change_baud = !usb && baud != TARGET_BAUD;
    let valset = rx.uses_valset();

    // ---- 1. the port itself: UBX output and the faster baud rate.
    // This comes first, so the messages switched on below cannot flood a slow port.
    let layers = if valset {
        // layers: RAM + BBR + Flash; receivers without flash refuse that, then RAM + BBR
        let ubx_out = if usb { 0x10780001 } else { 0x10740001 };
        let found = [0x07u8, 0x03]
            .into_iter()
            .find(|&l| command(port, 0x8A, &valset_cmd(l, ubx_out, 1)));
        let Some(l) = found else {
            return Err(format!("{name}: receiver refused CFG-VALSET"));
        };
        if change_baud {
            let _ = port.write_all(&frame(0x06, 0x8A, &valset_cmd(l, 0x40520001, TARGET_BAUD)));
        }
        saved = match l {
            0x07 => "saved to flash",
            _ => "saved to battery-backed RAM only (kept while the backup battery lasts)",
        };
        Some(l)
    } else {
        // CFG-PRT: read the settings of our port and change only what is needed, so
        // everything else (e.g. RTCM input) stays as it was
        let prt = match poll_port(port) {
            Some(mut p) => {
                let uart = matches!(p[0], 1 | 2); // 3 = USB: no baud rate
                change_baud &= uart;
                let old = p.clone();
                if change_baud {
                    p[8..12].copy_from_slice(&TARGET_BAUD.to_le_bytes());
                }
                p[12] |= 0x01; // input: UBX (the commands)
                p[14] |= 0x03; // output: UBX + NMEA
                (p != old).then_some(p)
            }
            // no answer: the usual settings of UART1 (8N1, in UBX+NMEA+RTCM, out UBX+NMEA)
            None if change_baud => {
                let mut p = vec![1, 0, 0, 0];
                p.extend(0x08D0u32.to_le_bytes());
                p.extend(TARGET_BAUD.to_le_bytes());
                p.extend([0x07, 0x00, 0x03, 0x00, 0, 0, 0, 0]);
                Some(p)
            }
            None => None,
        };
        if let Some(p) = prt {
            if change_baud {
                let _ = port.write_all(&frame(0x06, 0x00, &p)); // the ACK comes at the new baud rate
            } else {
                command(port, 0x00, &p);
            }
        }
        None
    };

    let new_baud = if change_baud { TARGET_BAUD } else { baud };
    if change_baud {
        let _ = port.flush();
        sleep(Duration::from_millis(200));
        port.set_baud_rate(new_baud)
            .map_err(|e| format!("{name}: {e}"))?;
        sleep(Duration::from_millis(100));
    }

    // ---- 2. solution period, as short as this receiver can do down to MEAS_MS
    let meas = set_rate(port, layers, MEAS_MS.max(rx.min_meas_ms()));
    let slow = (1000 / meas).max(1) as u32; // every n-th solution = once per second

    // ---- 3. message rates
    let rate = |fast: bool| if fast { 1 } else { slow };
    let (ok, total);
    match layers {
        Some(l) => {
            total = MSGOUT_KEYS.len();
            ok = MSGOUT_KEYS
                .iter()
                .filter(|&&(uart, usb_key, fast)| {
                    let key = if usb { usb_key } else { uart };
                    command(port, 0x8A, &valset_cmd(l, key, rate(fast)))
                })
                .count();
        }
        None => {
            let ubx = legacy_messages(rx);
            let nmea = NMEA_MSGS.map(|(id, fast)| (0xF0, id, fast));
            total = ubx.len() + nmea.len();
            // CFG-MSG: class, id, rate on the port the command arrives on
            ok = ubx
                .iter()
                .chain(nmea.iter())
                .filter(|&&(class, id, fast)| command(port, 0x01, &[class, id, rate(fast) as u8]))
                .count();
            // save I/O port, message and rate settings to BBR / Flash / EEPROM
            if command(port, 0x09, &[0, 0, 0, 0, 0x03, 0, 0, 0, 0, 0, 0, 0, 0x17]) {
                saved = "saved in the receiver";
            }
        }
    }

    // ---- 4. check that the receiver still answers, and how fast it sends
    if poll_version(port, Duration::from_millis(2000), &mut false).is_none() {
        return Err(format!(
            "{name}: no answer from the receiver at {new_baud} baud"
        ));
    }
    let measured = match measure_rate(port) {
        Some(hz) => format!("{hz:.1} Hz"),
        None => "no NMEA".to_string(),
    };
    let speed = match usb {
        _ if change_baud => format!("set to {new_baud} baud"),
        true => "on USB".to_string(),
        false => format!("at {new_baud} baud"),
    };
    Ok((
        new_baud,
        format!(
            "{name}: {} {speed}, {meas} ms solutions ({measured}), \
             {ok} of {total} message rates set, {saved}",
            rx.model
        ),
    ))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn frames() {
        // well-known frames: MON-VER poll, CFG-MSG NAV-PVT rate 1
        assert_eq!(
            frame(0x0A, 0x04, &[]),
            [0xB5, 0x62, 0x0A, 0x04, 0, 0, 0x0E, 0x34]
        );
        assert_eq!(
            frame(0x06, 0x01, &[1, 7, 1]),
            [0xB5, 0x62, 6, 1, 3, 0, 1, 7, 1, 0x13, 0x51]
        );
        assert_eq!(
            valset_cmd(1, 0x40520001, 115200),
            [0, 1, 0, 0, 1, 0, 0x52, 0x40, 0x00, 0xC2, 0x01, 0x00]
        );
        assert_eq!(valset_cmd(1, 0x20910007, 1).len(), 9);
        // CFG-RATE-MEAS is a 16-bit key: 100 ms
        assert_eq!(
            valset_cmd(7, RATE_MEAS_KEY, 100),
            [0, 7, 0, 0, 1, 0, 0x21, 0x30, 100, 0]
        );
    }

    #[test]
    fn monver() {
        let mut p = vec![0u8; 40];
        p[..4].copy_from_slice(b"7.03");
        p[30..38].copy_from_slice(b"00040007");
        let rx = parse_monver(&frame(0x0A, 0x04, &p));
        assert_eq!((rx.model.as_str(), rx.protver), ("U-Blox 6", 0.0));
        let mut e = [0u8; 30];
        e[..11].copy_from_slice(b"MOD=NEO-M8N");
        p.extend(e);
        let mut e = [0u8; 30];
        e[..13].copy_from_slice(b"PROTVER=18.00");
        p.extend(e);
        p[30..38].copy_from_slice(b"00080000");
        let rx = parse_monver(&frame(0x0A, 0x04, &p));
        assert_eq!(
            (rx.model.as_str(), rx.protver, rx.uses_valset()),
            ("U-Blox NEO-M8N", 18.0, false)
        );
    }

    /// Run with a receiver (or simulator):  GNSS_TEST_PORT=/dev/ttyACM0 cargo test -- --ignored
    #[test]
    #[ignore]
    fn real_port() {
        let name = std::env::var("GNSS_TEST_PORT").unwrap();
        let p = probe(&name);
        let rx = p.ublox.expect("no u-blox receiver");
        println!("{} at {:?}", rx.model, p.baud);
        println!("{:?}", configure(&name, &rx, p.baud.unwrap(), false));
    }
}
