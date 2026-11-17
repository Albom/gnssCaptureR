//! u-blox UBX binary protocol: decoding to text and updating the receiver state.
//! A frame is: B5 62 class id len(2) payload ck_a ck_b

use crate::nmea::{GpsState, KNOTS_TO_MS};
use std::fmt::Write;
use std::time::Instant;

pub fn checksum_ok(frame: &[u8]) -> bool {
    let n = frame.len();
    let (mut a, mut b) = (0u8, 0u8);
    for &x in &frame[2..n - 2] {
        a = a.wrapping_add(x);
        b = b.wrapping_add(a);
    }
    a == frame[n - 2] && b == frame[n - 1]
}

// Little-endian readers. Out-of-range reads return 0 instead of panicking.
fn u1(p: &[u8], o: usize) -> u8 {
    p.get(o).copied().unwrap_or(0)
}
fn i1(p: &[u8], o: usize) -> i8 {
    u1(p, o) as i8
}
fn bytes<const N: usize>(p: &[u8], o: usize) -> [u8; N] {
    p.get(o..o + N)
        .and_then(|s| s.try_into().ok())
        .unwrap_or([0; N])
}
fn u2(p: &[u8], o: usize) -> u16 {
    u16::from_le_bytes(bytes(p, o))
}
fn i2(p: &[u8], o: usize) -> i16 {
    i16::from_le_bytes(bytes(p, o))
}
fn u4(p: &[u8], o: usize) -> u32 {
    u32::from_le_bytes(bytes(p, o))
}
fn i4(p: &[u8], o: usize) -> i32 {
    i32::from_le_bytes(bytes(p, o))
}
fn r4(p: &[u8], o: usize) -> f32 {
    f32::from_le_bytes(bytes(p, o))
}
fn r8(p: &[u8], o: usize) -> f64 {
    f64::from_le_bytes(bytes(p, o))
}
fn text(p: &[u8]) -> String {
    String::from_utf8_lossy(p)
        .trim_end_matches('\0')
        .trim()
        .to_string()
}

pub fn gnss_name(id: u8) -> &'static str {
    match id {
        0 => "GPS",
        1 => "SBAS",
        2 => "Galileo",
        3 => "BeiDou",
        4 => "IMES",
        5 => "QZSS",
        6 => "GLONASS",
        7 => "NavIC",
        _ => "Unknown",
    }
}

pub fn name(class: u8, id: u8) -> String {
    let c = match class {
        0x01 => "NAV",
        0x02 => "RXM",
        0x04 => "INF",
        0x05 => "ACK",
        0x06 => "CFG",
        0x09 => "UPD",
        0x0A => "MON",
        0x0D => "TIM",
        0x10 => "ESF",
        0x13 => "MGA",
        0x21 => "LOG",
        0x27 => "SEC",
        0x28 => "HNR",
        _ => return format!("{class:02X}-{id:02X}"),
    };
    let i = match (class, id) {
        (0x01, 0x01) => "POSECEF",
        (0x01, 0x02) => "POSLLH",
        (0x01, 0x03) => "STATUS",
        (0x01, 0x04) => "DOP",
        (0x01, 0x07) => "PVT",
        (0x01, 0x12) => "VELNED",
        (0x01, 0x20) => "TIMEGPS",
        (0x01, 0x21) => "TIMEUTC",
        (0x01, 0x22) => "CLOCK",
        (0x01, 0x35) => "SAT",
        (0x01, 0x3C) => "RELPOSNED",
        (0x02, 0x13) => "SFRBX",
        (0x02, 0x15) => "RAWX",
        (0x04, 0x00) => "ERROR",
        (0x04, 0x01) => "WARNING",
        (0x04, 0x02) => "NOTICE",
        (0x04, 0x03) => "TEST",
        (0x04, 0x04) => "DEBUG",
        (0x05, 0x00) => "NAK",
        (0x05, 0x01) => "ACK",
        (0x06, 0x00) => "PRT",
        (0x06, 0x01) => "MSG",
        (0x06, 0x08) => "RATE",
        (0x06, 0x8A) => "VALSET",
        (0x0A, 0x04) => "VER",
        (0x0A, 0x09) => "HW",
        (0x0D, 0x01) => "TP",
        (0x0D, 0x03) => "TM2",
        _ => return format!("{c}-{id:02X}"),
    };
    format!("{c}-{i}")
}

/// Decodes a complete UBX frame into human readable text.
/// The first line is a summary; some messages add indented detail lines.
pub fn decode(frame: &[u8]) -> String {
    let (class, id) = (frame[2], frame[3]);
    let p = &frame[6..frame.len() - 2];
    let mut s = format!("{} len={}", name(class, id), p.len());
    let w = &mut s;
    match (class, id) {
        (0x01, 0x02) => {
            let _ = write!(
                w,
                " iTOW={} lat={:.7} lon={:.7} height={:.3}m hMSL={:.3}m hAcc={:.3}m vAcc={:.3}m",
                u4(p, 0),
                i4(p, 8) as f64 * 1e-7,
                i4(p, 4) as f64 * 1e-7,
                i4(p, 12) as f64 / 1e3,
                i4(p, 16) as f64 / 1e3,
                u4(p, 20) as f64 / 1e3,
                u4(p, 24) as f64 / 1e3
            );
        }
        (0x01, 0x03) => {
            let _ = write!(
                w,
                " iTOW={} gpsFix={} flags=0x{:02X} fixStat=0x{:02X} flags2=0x{:02X} ttff={}ms msss={}ms",
                u4(p, 0),
                u1(p, 4),
                u1(p, 5),
                u1(p, 6),
                u1(p, 7),
                u4(p, 8),
                u4(p, 12)
            );
        }
        (0x01, 0x04) => {
            let d = |o| u2(p, o) as f64 * 0.01;
            let _ = write!(
                w,
                " iTOW={} gDOP={:.2} pDOP={:.2} tDOP={:.2} vDOP={:.2} hDOP={:.2} nDOP={:.2} eDOP={:.2}",
                u4(p, 0),
                d(4),
                d(6),
                d(8),
                d(10),
                d(12),
                d(14),
                d(16)
            );
        }
        (0x01, 0x07) => {
            let g = i4(p, 60) as f64 / 1e3;
            let _ = write!(
                w,
                " iTOW={} {:04}-{:02}-{:02} {:02}:{:02}:{:02} valid=0x{:02X} fixType={} flags=0x{:02X} numSV={} \
                 lat={:.7} lon={:.7} height={:.3}m hMSL={:.3}m hAcc={:.3}m vAcc={:.3}m \
                 velN={:.3} velE={:.3} velD={:.3} gSpeed={:.3}m/s ({:.3}km/h, {:.3}kn) headMot={:.5} pDOP={:.2}",
                u4(p, 0),
                u2(p, 4),
                u1(p, 6),
                u1(p, 7),
                u1(p, 8),
                u1(p, 9),
                u1(p, 10),
                u1(p, 11),
                u1(p, 20),
                u1(p, 21),
                u1(p, 23),
                i4(p, 28) as f64 * 1e-7,
                i4(p, 24) as f64 * 1e-7,
                i4(p, 32) as f64 / 1e3,
                i4(p, 36) as f64 / 1e3,
                u4(p, 40) as f64 / 1e3,
                u4(p, 44) as f64 / 1e3,
                i4(p, 48) as f64 / 1e3,
                i4(p, 52) as f64 / 1e3,
                i4(p, 56) as f64 / 1e3,
                g,
                g * 3.6,
                g / KNOTS_TO_MS,
                i4(p, 64) as f64 * 1e-5,
                u2(p, 76) as f64 * 0.01
            );
        }
        (0x01, 0x12) => {
            let _ = write!(
                w,
                " iTOW={} velN={:.2} velE={:.2} velD={:.2} speed={:.2}m/s gSpeed={:.2}m/s heading={:.5}",
                u4(p, 0),
                i4(p, 4) as f64 / 100.0,
                i4(p, 8) as f64 / 100.0,
                i4(p, 12) as f64 / 100.0,
                u4(p, 16) as f64 / 100.0,
                u4(p, 20) as f64 / 100.0,
                i4(p, 24) as f64 * 1e-5
            );
        }
        (0x01, 0x21) => {
            let _ = write!(
                w,
                " iTOW={} {:04}-{:02}-{:02} {:02}:{:02}:{:02} nano={} tAcc={}ns valid=0x{:02X}",
                u4(p, 0),
                u2(p, 12),
                u1(p, 14),
                u1(p, 15),
                u1(p, 16),
                u1(p, 17),
                u1(p, 18),
                i4(p, 8),
                u4(p, 4),
                u1(p, 19)
            );
        }
        (0x01, 0x22) => {
            let _ = write!(
                w,
                " iTOW={} clkB={}ns clkD={}ns/s tAcc={}ns fAcc={}ps/s",
                u4(p, 0),
                i4(p, 4),
                i4(p, 8),
                u4(p, 12),
                u4(p, 16)
            );
        }
        (0x01, 0x35) => {
            let n = u1(p, 5) as usize;
            let _ = write!(w, " iTOW={} numSvs={n}", u4(p, 0));
            for k in 0..n {
                let o = 8 + 12 * k;
                let _ = write!(
                    w,
                    "\n    {:8} sv={:3} cno={:2}dBHz elev={:3} azim={:3} prRes={:.1}m flags=0x{:08X} used={}",
                    gnss_name(u1(p, o)),
                    u1(p, o + 1),
                    u1(p, o + 2),
                    i1(p, o + 3),
                    i2(p, o + 4),
                    i2(p, o + 6) as f64 * 0.1,
                    u4(p, o + 8),
                    (u4(p, o + 8) >> 3) & 1
                );
            }
        }
        (0x02, 0x13) => {
            let n = u1(p, 4) as usize;
            let _ = write!(
                w,
                " gnss={} sv={} sigId={} freqId={} numWords={n} chn={} words=",
                gnss_name(u1(p, 0)),
                u1(p, 1),
                u1(p, 2),
                u1(p, 3),
                u1(p, 5)
            );
            for k in 0..n {
                let _ = write!(w, "{:08X} ", u4(p, 8 + 4 * k));
            }
        }
        (0x02, 0x15) => {
            let n = u1(p, 11) as usize;
            let _ = write!(
                w,
                " rcvTow={:.3} week={} leapS={} numMeas={n} recStat=0x{:02X}",
                r8(p, 0),
                u2(p, 8),
                i1(p, 10),
                u1(p, 12)
            );
            for k in 0..n {
                let o = 16 + 32 * k;
                let _ = write!(
                    w,
                    "\n    {:8} sv={:3} sig={} freq={:2} pr={:.3} cp={:.3} do={:.3} cno={:2} lock={}ms trkStat=0x{:02X}",
                    gnss_name(u1(p, o + 20)),
                    u1(p, o + 21),
                    u1(p, o + 22),
                    u1(p, o + 23),
                    r8(p, o),
                    r8(p, o + 8),
                    r4(p, o + 16),
                    u1(p, o + 26),
                    u2(p, o + 24),
                    u1(p, o + 30)
                );
            }
        }
        (0x04, _) => {
            let _ = write!(w, " \"{}\"", text(p));
        }
        (0x05, _) => {
            let _ = write!(w, " for {}", name(u1(p, 0), u1(p, 1)));
        }
        (0x0A, 0x04) => {
            let _ = write!(
                w,
                " sw=\"{}\" hw=\"{}\"",
                text(&p[..30.min(p.len())]),
                text(p.get(30..40).unwrap_or(&[]))
            );
            for ext in p.get(40..).unwrap_or(&[]).chunks(30) {
                let _ = write!(w, " \"{}\"", text(ext));
            }
        }
        _ => {
            w.push_str(" payload=");
            for b in p {
                let _ = write!(w, "{b:02X}");
            }
        }
    }
    s
}

/// Updates the displayed receiver state from NAV-PVT, NAV-DOP and NAV-SAT.
pub fn apply(frame: &[u8], st: &mut GpsState) {
    let p = &frame[6..frame.len() - 2];
    match (frame[2], frame[3]) {
        (0x01, 0x07) if p.len() >= 92 => {
            // date and time only when the receiver marks them valid
            let valid = u1(p, 11);
            if valid & 1 != 0 {
                st.utc_date = format!("{:04}-{:02}-{:02}", u2(p, 4), u1(p, 6), u1(p, 7));
            }
            if valid & 2 != 0 {
                let (h, m, s) = (u1(p, 8), u1(p, 9), u1(p, 10));
                st.utc_time = format!("{h:02}:{m:02}:{s:02}");
                st.time_pc = Some(chrono::Utc::now());
                st.epoch = Some(h as u32 * 3600 + m as u32 * 60 + s as u32);
            }
            let fix = u1(p, 20);
            let flags = u1(p, 21);
            st.fix_mode = match fix {
                2 => 2,
                3 | 4 => 3,
                _ => 1,
            };
            st.quality = match (flags & 1, flags >> 6, flags & 2) {
                (0, _, _) => 0,
                (_, 2, _) => 4,
                (_, 1, _) => 5,
                (_, _, 2) => 2,
                _ => 1,
            };
            st.sats_used = Some(u1(p, 23) as u32);
            // position and velocity only with a valid fix (before it they are zeros)
            let ok = (flags & 1 != 0).then_some(());
            st.lon = ok.map(|_| i4(p, 24) as f64 * 1e-7);
            st.lat = ok.map(|_| i4(p, 28) as f64 * 1e-7);
            st.alt = ok.map(|_| i4(p, 36) as f64 / 1e3);
            st.speed_knots = ok.map(|_| i4(p, 60) as f64 / 1e3 / KNOTS_TO_MS);
            st.heading = ok.map(|_| i4(p, 64) as f64 * 1e-5);
            st.pdop = ok.map(|_| u2(p, 76) as f64 * 0.01);
        }
        (0x01, 0x04) if p.len() >= 18 => {
            st.pdop = Some(u2(p, 6) as f64 * 0.01);
            st.vdop = Some(u2(p, 10) as f64 * 0.01);
            st.hdop = Some(u2(p, 12) as f64 * 0.01);
        }
        (0x01, 0x35) => {
            let now = Instant::now();
            for k in 0..u1(p, 5) as usize {
                let o = 8 + 12 * k;
                if o + 12 > p.len() {
                    break;
                }
                let key = (gnss_name(u1(p, o)), u1(p, o + 1) as u16);
                let elev = i1(p, o + 3);
                st.add_sat(
                    key,
                    (elev >= -90).then_some(elev as f64),
                    Some(i2(p, o + 4) as f64),
                    Some(u1(p, o + 2) as f64).filter(|&c| c > 0.0),
                );
                if (u4(p, o + 8) >> 3) & 1 == 1 {
                    st.used.insert(key, now);
                }
            }
        }
        _ => {}
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn pvt(valid: u8, fix: u8, flags: u8) -> Vec<u8> {
        let mut p = vec![0u8; 92];
        p[4..6].copy_from_slice(&2026u16.to_le_bytes());
        (p[6], p[7], p[8], p[9], p[10], p[11]) = (11, 15, 10, 20, 30, valid);
        (p[20], p[21]) = (fix, flags);
        p[24..28].copy_from_slice(&305_243_170i32.to_le_bytes());
        p[28..32].copy_from_slice(&504_494_000i32.to_le_bytes());
        crate::ublox::frame(0x01, 0x07, &p)
    }

    #[test]
    fn pvt_validity() {
        // before the first fix nothing is taken from NAV-PVT (B2)
        let mut st = GpsState::default();
        apply(&pvt(0, 0, 0), &mut st);
        assert!(st.lat.is_none() && st.lon.is_none() && st.speed_knots.is_none());
        assert!(st.utc_date.is_empty() && st.utc_time.is_empty() && st.epoch.is_none());
        // valid date, time and fix
        apply(&pvt(3, 3, 1), &mut st);
        assert!((st.lat.unwrap() - 50.4494).abs() < 1e-9 && st.speed_knots == Some(0.0));
        assert_eq!(
            (st.utc_date.as_str(), st.utc_time.as_str()),
            ("2026-11-15", "10:20:30")
        );
        assert_eq!(st.epoch, Some(10 * 3600 + 20 * 60 + 30));
        // fix lost again: the old position is not kept
        apply(&pvt(3, 0, 0), &mut st);
        assert!(st.lat.is_none());
    }
}
