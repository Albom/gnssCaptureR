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

/// UBX messages switched on, per receiver generation (class, id).
fn legacy_messages(rx: &Receiver) -> &'static [(u8, u8)] {
    if rx.hw.starts_with("0004") {
        // u-blox 5 / 6: POSLLH STATUS DOP VELNED TIMEUTC CLOCK SVINFO, RXM-RAW SFRB
        &[
            (1, 0x02),
            (1, 0x03),
            (1, 0x04),
            (1, 0x12),
            (1, 0x21),
            (1, 0x22),
            (1, 0x30),
            (2, 0x10),
            (2, 0x11),
        ]
    } else if rx.hw == "00070000" {
        // u-blox 7: PVT DOP TIMEUTC CLOCK SVINFO
        &[(1, 0x07), (1, 0x04), (1, 0x21), (1, 0x22), (1, 0x30)]
    } else {
        // u-blox M8: PVT DOP TIMEUTC CLOCK SAT, RXM-RAWX SFRBX
        &[
            (1, 0x07),
            (1, 0x04),
            (1, 0x21),
            (1, 0x22),
            (1, 0x35),
            (2, 0x15),
            (2, 0x13),
        ]
    }
}

/// CFG-MSGOUT keys (UART1, USB) for F9 / M10: NAV-PVT DOP TIMEUTC CLOCK SAT, RXM-RAWX SFRBX
const MSGOUT_KEYS: [(u32, u32); 7] = [
    (0x20910007, 0x20910009),
    (0x20910039, 0x2091003b),
    (0x2091005c, 0x2091005e),
    (0x20910066, 0x20910068),
    (0x20910016, 0x20910018),
    (0x209102a5, 0x209102a7),
    (0x20910232, 0x20910234),
];

/// CFG-VALSET payload with one key; the value size follows from the key ID.
fn valset(layers: u8, key: u32, value: u32) -> Vec<u8> {
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

/// Switches UBX output on (NMEA stays on), sets 115200 baud and saves the configuration.
/// `baud` is the current baud rate, `usb` is true for a native u-blox USB port (no baud rate).
/// Returns the baud rate to use and a status text.
pub fn configure(name: &str, rx: &Receiver, baud: u32, usb: bool) -> Result<(u32, String), String> {
    let mut port = serialport::new(name, baud)
        .timeout(Duration::from_millis(50))
        .open()
        .map_err(|e| format!("{name}: {e}"))?;
    let port = port.as_mut();
    let (mut ok, total);
    let mut saved = "not saved";
    let mut change_baud = !usb && baud != TARGET_BAUD;

    if rx.uses_valset() {
        // layers: RAM + BBR + Flash; receivers without flash refuse that, then RAM + BBR
        let ubx_out = if usb { 0x10780001 } else { 0x10740001 };
        let layers = [0x07u8, 0x03]
            .into_iter()
            .find(|&l| command(port, 0x8A, &valset(l, ubx_out, 1)));
        let Some(layers) = layers else {
            return Err(format!("{name}: receiver refused CFG-VALSET"));
        };
        total = MSGOUT_KEYS.len();
        ok = 0;
        for (uart, usb_key) in MSGOUT_KEYS {
            ok += command(
                port,
                0x8A,
                &valset(layers, if usb { usb_key } else { uart }, 1),
            ) as usize;
        }
        if change_baud {
            let _ = port.write_all(&frame(0x06, 0x8A, &valset(layers, 0x40520001, TARGET_BAUD)));
        }
        saved = match layers {
            0x07 => "saved to flash",
            _ => "saved to battery-backed RAM only (kept while the backup battery lasts)",
        };
    } else {
        let msgs = legacy_messages(rx);
        total = msgs.len();
        ok = 0;
        for &(class, id) in msgs {
            // CFG-MSG: class, id, rate 1 on the port the command arrives on
            ok += command(port, 0x01, &[class, id, 1]) as usize;
        }
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
    }

    let new_baud = if change_baud { TARGET_BAUD } else { baud };
    if change_baud {
        let _ = port.flush();
        sleep(Duration::from_millis(200));
        port.set_baud_rate(new_baud)
            .map_err(|e| format!("{name}: {e}"))?;
        sleep(Duration::from_millis(100));
    }
    // legacy receivers: save I/O port and message settings (CFG-CFG) to BBR / Flash / EEPROM
    if !rx.uses_valset() && command(port, 0x09, &[0, 0, 0, 0, 0x03, 0, 0, 0, 0, 0, 0, 0, 0x17]) {
        saved = "saved in the receiver";
    }
    // the receiver must answer at the (new) baud rate
    if poll_version(port, Duration::from_millis(2000), &mut false).is_none() {
        return Err(format!(
            "{name}: no answer from the receiver at {new_baud} baud"
        ));
    }
    let speed = match usb {
        _ if change_baud => format!("set to {new_baud} baud"),
        true => "on USB".to_string(),
        false => format!("at {new_baud} baud"),
    };
    Ok((
        new_baud,
        format!(
            "{name}: {} {speed}, {ok} of {total} UBX messages on, {saved}",
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
            valset(1, 0x40520001, 115200),
            [0, 1, 0, 0, 1, 0, 0x52, 0x40, 0x00, 0xC2, 0x01, 0x00]
        );
        assert_eq!(valset(1, 0x20910007, 1).len(), 9);
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
