//! Current receiver state and NMEA sentence parsing.

use std::collections::HashMap;
use std::time::{Duration, Instant};

pub const KNOTS_TO_KMH: f64 = 1.852;
pub const KNOTS_TO_MS: f64 = 1852.0 / 3600.0;

pub struct Sat {
    pub elev: Option<f64>,
    pub az: Option<f64>,
    pub snr: Option<f64>,
    pub seen: Instant,
    snr_at: Instant, // when `snr` was reported (several signals of one satellite are merged)
}

/// C/N0 statistics of one satellite, collected for the Quality tab.
#[derive(Clone, Copy)]
pub struct SnrStat {
    pub sum: f64,
    pub min: f64,
    pub max: f64,
    pub n: u32,
}

impl SnrStat {
    fn new() -> SnrStat {
        SnrStat {
            sum: 0.0,
            min: f64::MAX,
            max: f64::MIN,
            n: 0,
        }
    }
    fn add(&mut self, v: f64) {
        self.sum += v;
        self.min = self.min.min(v);
        self.max = self.max.max(v);
        self.n += 1;
    }
    pub fn mean(&self) -> Option<f64> {
        (self.n > 0).then(|| self.sum / self.n as f64)
    }
}

/// Key of a satellite: (constellation name, PRN / SV number)
pub type SatKey = (&'static str, u16);

#[derive(Default)]
pub struct GpsState {
    pub utc_time: String,
    pub utc_date: String,
    pub time_pc: Option<chrono::DateTime<chrono::Utc>>, // PC time when utc_time was received
    pub epoch: Option<u32>, // receiver time of the last time message, whole seconds of the day
    pub lat: Option<f64>,
    pub lon: Option<f64>,
    pub alt: Option<f64>,         // metres above mean sea level
    pub speed_knots: Option<f64>, // speed is always stored in knots
    pub heading: Option<f64>,     // degrees, true
    pub quality: u8,              // GGA fix quality 0..8
    pub fix_mode: u8,             // GSA: 1 = no fix, 2 = 2D, 3 = 3D
    pub sats_used: Option<u32>,
    pub hdop: Option<f64>,
    pub vdop: Option<f64>,
    pub pdop: Option<f64>,
    pub sats: HashMap<SatKey, Sat>,
    pub used: HashMap<SatKey, Instant>,
    pub snr_stats: HashMap<SatKey, SnrStat>, // for the Quality tab (only satellites with C/N0)
    pub snr_sec: HashMap<SatKey, SnrStat>,   // for the quality file (all reported satellites)
}

impl GpsState {
    pub fn speed_kmh(&self) -> Option<f64> {
        self.speed_knots.map(|v| v * KNOTS_TO_KMH)
    }
    pub fn speed_ms(&self) -> Option<f64> {
        self.speed_knots.map(|v| v * KNOTS_TO_MS)
    }

    pub fn gps_type(&self) -> String {
        let q = quality_name(self.quality);
        match self.fix_mode {
            2 => format!("2D, {q}"),
            3 => format!("3D, {q}"),
            _ => q.to_string(),
        }
    }

    /// Forget satellites that have not been reported for a while.
    pub fn expire(&mut self) {
        let now = Instant::now();
        self.sats
            .retain(|_, s| now - s.seen < Duration::from_secs(10));
        self.used.retain(|_, t| now - *t < Duration::from_secs(5));
    }

    /// Stores a satellite from GSV / NAV-SAT and adds its C/N0 to the statistics.
    pub fn add_sat(&mut self, key: SatKey, elev: Option<f64>, az: Option<f64>, snr: Option<f64>) {
        // A dual-band receiver reports a satellite once per signal, within one burst.
        // Show the strongest signal; an empty C/N0 must not erase a value just reported.
        let now = Instant::now();
        let burst = Duration::from_millis(500);
        let old = self.sats.get(&key).filter(|s| now - s.snr_at < burst);
        let (shown, snr_at) = match (snr, old.and_then(|s| s.snr.map(|v| (v, s.snr_at)))) {
            (Some(a), Some((b, at))) => (Some(a.max(b)), at),
            (None, Some((b, at))) => (Some(b), at),
            (a, None) => (a, now),
        };
        self.sats.insert(
            key,
            Sat {
                elev,
                az,
                snr: shown,
                seen: now,
                snr_at,
            },
        );
        let sec = self.snr_sec.entry(key).or_insert_with(SnrStat::new);
        if let Some(v) = snr {
            sec.add(v);
            self.snr_stats
                .entry(key)
                .or_insert_with(SnrStat::new)
                .add(v);
        }
    }

    /// Signal quality of the satellites reported since the last call (one second):
    /// "inView=N used=N meanCN0=x meanCN0used=x G05:U:45.0 R07:V:-  ..."
    /// U = used in fix, V = in view only, "-" = no C/N0 reported. None if nothing was reported.
    pub fn quality_line(&mut self) -> Option<String> {
        let stats = std::mem::take(&mut self.snr_sec);
        if stats.is_empty() {
            return None;
        }
        let mut sats: Vec<_> = stats.into_iter().collect();
        sats.sort_by_key(|(k, _)| sat_order(k));
        let used = |k: &SatKey| self.used.contains_key(k);
        let avg = |v: Vec<f64>| match v.len() {
            0 => "-".to_string(),
            n => format!("{:.1}", v.iter().sum::<f64>() / n as f64),
        };
        let all: Vec<f64> = sats.iter().filter_map(|(_, s)| s.mean()).collect();
        let in_fix: Vec<f64> = sats
            .iter()
            .filter(|(k, _)| used(k))
            .filter_map(|(_, s)| s.mean())
            .collect();
        let mut line = format!(
            "inView={} used={} meanCN0={} meanCN0used={}",
            sats.len(),
            sats.iter().filter(|(k, _)| used(k)).count(),
            avg(all),
            avg(in_fix)
        );
        for (k, s) in &sats {
            let cn0 = s.mean().map(|v| format!("{v:.1}")).unwrap_or("-".into());
            line += &format!(
                " {}{:02}:{}:{cn0}",
                sys_letter(k.0),
                k.1,
                if used(k) { "U" } else { "V" }
            );
        }
        Some(line)
    }

    pub fn is_used(&self, key: &SatKey) -> bool {
        self.used.contains_key(key)
    }
}

/// One-letter constellation code (RINEX style).
pub fn sys_letter(sys: &str) -> &'static str {
    match sys {
        "GPS" => "G",
        "GLONASS" => "R",
        "Galileo" => "E",
        "BeiDou" => "C",
        "QZSS" => "J",
        "NavIC" => "I",
        "SBAS" => "S",
        _ => "?",
    }
}

/// Sort key: GPS, GLONASS, Galileo, BeiDou, QZSS, NavIC, SBAS, then by number.
pub fn sat_order(k: &SatKey) -> (usize, u16) {
    ("GRECJIS".find(sys_letter(k.0)).unwrap_or(9), k.1)
}

pub fn quality_name(q: u8) -> &'static str {
    match q {
        0 => "No fix",
        1 => "GPS (SPS)",
        2 => "DGPS",
        3 => "PPS",
        4 => "RTK Fixed",
        5 => "RTK Float",
        6 => "Dead reckoning",
        7 => "Manual",
        8 => "Simulation",
        _ => "Unknown",
    }
}

/// Checks "*hh" checksum. Sentences without a checksum are accepted.
pub fn checksum_ok(line: &str) -> bool {
    let Some(body) = line.strip_prefix('$').or_else(|| line.strip_prefix('!')) else {
        return false;
    };
    match body.split_once('*') {
        None => true,
        Some((data, cs)) => {
            let calc = data.bytes().fold(0u8, |a, b| a ^ b);
            u8::from_str_radix(cs.get(..2).unwrap_or(""), 16) == Ok(calc)
        }
    }
}

/// A sentence that may update the receiver state: it must carry a correct checksum.
/// (A sentence cut short by a lost byte has none and would give truncated values.)
pub fn valid(line: &str) -> bool {
    line.contains('*') && checksum_ok(line)
}

/// "hhmmss.ss" -> whole seconds of the day
fn epoch_of(s: &str) -> Option<u32> {
    let n = |r: std::ops::Range<usize>| s.get(r)?.parse::<u32>().ok();
    Some(n(0..2)? * 3600 + n(2..4)? * 60 + n(4..6)?)
}

fn num(s: Option<&&str>) -> Option<f64> {
    s.and_then(|v| v.trim().parse().ok())
}

/// "ddmm.mmmm" + hemisphere -> decimal degrees
fn coord(v: Option<&&str>, hemi: Option<&&str>) -> Option<f64> {
    let v = num(v)?;
    let deg = (v / 100.0).trunc() + (v % 100.0) / 60.0;
    match hemi.copied() {
        Some("S") | Some("W") => Some(-deg),
        _ => Some(deg),
    }
}

fn hhmmss(s: &str) -> String {
    if s.len() >= 6 {
        format!("{}:{}:{}", &s[0..2], &s[2..4], &s[4..])
    } else {
        String::new()
    }
}

/// Constellation from NMEA talker ID (and PRN for old-style GP sentences).
fn system(talker: &str, prn: u16) -> &'static str {
    match talker {
        "GL" => "GLONASS",
        "GA" => "Galileo",
        "GB" | "BD" => "BeiDou",
        "GQ" | "QZ" => "QZSS",
        "GI" => "NavIC",
        _ => match prn {
            33..=64 | 120..=158 => "SBAS",
            65..=96 => "GLONASS",
            193..=202 => "QZSS",
            301..=336 => "Galileo", // u-blox "extended" numbering in NMEA 4.0
            401..=437 => "BeiDou",
            _ => "GPS",
        },
    }
}

/// NMEA numbers some satellites differently from UBX; use the UBX numbering
/// (GLONASS slot 1-32, SBAS PRN 120-158, QZSS 1-10, Galileo 1-36, BeiDou 1-37) so both
/// sources agree.
fn sat_key(sys: &'static str, prn: u16) -> SatKey {
    match (sys, prn) {
        ("GLONASS", 65..=96) => (sys, prn - 64),
        ("SBAS", 33..=64) => (sys, prn + 87),
        ("QZSS", 193..=202) => (sys, prn - 192),
        ("Galileo", 301..=336) => (sys, prn - 300),
        ("BeiDou", 401..=437) => (sys, prn - 400),
        _ => (sys, prn),
    }
}

/// System ID used in NMEA 4.10+ GSA sentences.
fn system_by_id(id: &str) -> Option<&'static str> {
    Some(match id {
        "1" => "GPS",
        "2" => "GLONASS",
        "3" => "Galileo",
        "4" => "BeiDou",
        "5" => "QZSS",
        "6" => "NavIC",
        _ => return None,
    })
}

/// Updates the state from one NMEA sentence. Bad sentences are ignored.
pub fn parse(line: &str, st: &mut GpsState) {
    if !valid(line) {
        return;
    }
    let body = line[1..].split('*').next().unwrap_or("");
    let f: Vec<&str> = body.split(',').collect();
    if f[0].len() < 5 || f[0].starts_with('P') {
        return; // proprietary sentences are not decoded
    }
    let talker = &f[0][..2];
    match &f[0][2..] {
        "GGA" => {
            st.utc_time = hhmmss(f.get(1).unwrap_or(&""));
            st.time_pc = Some(chrono::Utc::now());
            st.epoch = epoch_of(f.get(1).unwrap_or(&"")).or(st.epoch);
            st.lat = coord(f.get(2), f.get(3));
            st.lon = coord(f.get(4), f.get(5));
            st.quality = num(f.get(6)).unwrap_or(0.0) as u8;
            st.sats_used = num(f.get(7)).map(|v| v as u32);
            st.hdop = num(f.get(8)).or(st.hdop);
            st.alt = num(f.get(9));
        }
        "RMC" => {
            st.utc_time = hhmmss(f.get(1).unwrap_or(&""));
            st.time_pc = Some(chrono::Utc::now());
            st.epoch = epoch_of(f.get(1).unwrap_or(&"")).or(st.epoch);
            if f.get(2) == Some(&"A") {
                st.lat = coord(f.get(3), f.get(4));
                st.lon = coord(f.get(5), f.get(6));
            }
            st.speed_knots = num(f.get(7)); // RMC speed is in KNOTS
            st.heading = num(f.get(8)).or(st.heading);
            if let Some(d) = f.get(9).filter(|d| d.len() == 6) {
                st.utc_date = format!("20{}-{}-{}", &d[4..6], &d[2..4], &d[0..2]);
            }
        }
        "VTG" => {
            st.heading = num(f.get(1)).or(st.heading);
            // field 5 = knots, field 7 = km/h; keep knots as the base unit
            st.speed_knots = num(f.get(5)).or(num(f.get(7)).map(|k| k / KNOTS_TO_KMH));
        }
        "GSA" => {
            st.fix_mode = num(f.get(2)).unwrap_or(0.0) as u8;
            st.pdop = num(f.get(15));
            st.hdop = num(f.get(16));
            st.vdop = num(f.get(17));
            let sys_id = f.get(18).and_then(|s| system_by_id(s));
            let now = Instant::now();
            for prn in f
                .iter()
                .skip(3)
                .take(12)
                .filter_map(|s| s.parse::<u16>().ok())
            {
                let sys = sys_id.unwrap_or_else(|| system(talker, prn));
                st.used.insert(sat_key(sys, prn), now);
            }
        }
        "GSV" => {
            // NMEA 4.10+: one more field at the end, the signal ID - not a satellite
            let extra = f.len().saturating_sub(4) % 4 == 1;
            for g in f[4.min(f.len())..f.len() - extra as usize].chunks(4) {
                let Some(prn) = g.first().and_then(|s| s.parse::<u16>().ok()) else {
                    continue;
                };
                let key = sat_key(system(talker, prn), prn);
                st.add_sat(key, num(g.get(1)), num(g.get(2)), num(g.get(3)));
            }
        }
        _ => {}
    }
}

fn talker_name(t: &str) -> &str {
    match t {
        "GP" => "GPS",
        "GL" => "GLONASS",
        "GA" => "Galileo",
        "GB" | "BD" => "BeiDou",
        "GQ" | "QZ" => "QZSS",
        "GI" => "NavIC",
        "GN" => "multi-GNSS",
        _ => t,
    }
}

fn fmt_coord(v: Option<f64>, pos: char, neg: char) -> String {
    match v {
        Some(x) => format!("{:.7}{}", x.abs(), if x < 0.0 { neg } else { pos }),
        None => "-".into(),
    }
}

fn fmt_speed(knots: Option<f64>) -> String {
    match knots {
        Some(k) => format!(
            "{k:.3} kn = {:.3} km/h = {:.3} m/s",
            k * KNOTS_TO_KMH,
            k * KNOTS_TO_MS
        ),
        None => "-".into(),
    }
}

/// Decodes one NMEA sentence into human readable text.
/// The first line is a summary; GSV adds one indented line per satellite.
pub fn decode(line: &str) -> String {
    if !checksum_ok(line) {
        return format!("CHECKSUM ERROR {line}");
    }
    let body = line[1..].split('*').next().unwrap_or("");
    let f: Vec<&str> = body.split(',').collect();
    let g = |i: usize| f.get(i).copied().unwrap_or("");
    let id = f[0];
    if id.len() < 5 || id.starts_with('P') {
        return format!("{id} (proprietary) {}", f[1..].join(","));
    }
    let (talker, typ) = id.split_at(2);
    let pos = |i: usize| {
        format!(
            "lat={} lon={}",
            fmt_coord(coord(f.get(i), f.get(i + 1)), 'N', 'S'),
            fmt_coord(coord(f.get(i + 2), f.get(i + 3)), 'E', 'W')
        )
    };
    let head = format!("{id} [{}]", talker_name(talker));
    match typ {
        "GGA" => {
            let q = g(6).parse().unwrap_or(0);
            format!(
                "{head} time={} {} quality={q} ({}) satsUsed={} HDOP={} alt={} m geoidSep={} m dgpsAge={} station={}",
                hhmmss(g(1)),
                pos(2),
                quality_name(q),
                g(7),
                g(8),
                g(9),
                g(11),
                g(13),
                g(14)
            )
        }
        "RMC" => format!(
            "{head} time={} date={} status={} {} speed={} course={} magVar={}{} mode={}",
            hhmmss(g(1)),
            g(9),
            if g(2) == "A" { "valid" } else { "INVALID" },
            pos(3),
            fmt_speed(num(f.get(7))),
            g(8),
            g(10),
            g(11),
            g(12)
        ),
        "VTG" => {
            let kn = num(f.get(5)).or(num(f.get(7)).map(|k| k / KNOTS_TO_KMH));
            format!(
                "{head} courseTrue={} courseMag={} speed={} mode={}",
                g(1),
                g(3),
                fmt_speed(kn),
                g(9)
            )
        }
        "GSA" => {
            let sats: Vec<&str> = f
                .iter()
                .skip(3)
                .take(12)
                .copied()
                .filter(|s| !s.is_empty())
                .collect();
            let fix = match g(2) {
                "2" => "2D",
                "3" => "3D",
                _ => "none",
            };
            format!(
                "{head} mode={} fix={fix} sats=[{}] PDOP={} HDOP={} VDOP={} system={}",
                g(1),
                sats.join(" "),
                g(15),
                g(16),
                g(17),
                system_by_id(g(18)).unwrap_or(talker_name(talker))
            )
        }
        "GSV" => {
            let extra = f.len().saturating_sub(4) % 4 == 1; // NMEA 4.10: signal ID at the end
            let mut s = format!("{head} msg {}/{} inView={}", g(2), g(1), g(3));
            if extra {
                s += &format!(" signal={}", f[f.len() - 1]);
            }
            for c in f[4.min(f.len())..f.len() - extra as usize].chunks(4) {
                let c = |i: usize| c.get(i).copied().unwrap_or("");
                s += &format!(
                    "\n    PRN={:>3} elev={:>2} az={:>3} snr={:>2}",
                    c(0),
                    c(1),
                    c(2),
                    c(3)
                );
            }
            s
        }
        "GLL" => format!(
            "{head} {} time={} status={} mode={}",
            pos(1),
            hhmmss(g(5)),
            g(6),
            g(7)
        ),
        "GNS" => format!(
            "{head} time={} {} mode={} satsUsed={} HDOP={} alt={} m geoidSep={} m",
            hhmmss(g(1)),
            pos(2),
            g(6),
            g(7),
            g(8),
            g(9),
            g(10)
        ),
        "ZDA" => format!(
            "{head} time={} date={}-{}-{} tz={}:{}",
            hhmmss(g(1)),
            g(4),
            g(3),
            g(2),
            g(5),
            g(6)
        ),
        "GST" => format!(
            "{head} time={} rms={} stdMajor={} stdMinor={} orient={} stdLat={} m stdLon={} m stdAlt={} m",
            hhmmss(g(1)),
            g(2),
            g(3),
            g(4),
            g(5),
            g(6),
            g(7),
            g(8)
        ),
        "TXT" => format!("{head} msg {}/{} type={} \"{}\"", g(2), g(1), g(3), g(4)),
        _ => format!("{head} (not decoded) {}", f[1..].join(",")),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rmc_speed_units() {
        let mut st = GpsState::default();
        let s = "$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A";
        assert!(checksum_ok(s));
        parse(s, &mut st);
        assert!((st.lat.unwrap() - 48.1173).abs() < 1e-4);
        assert_eq!(st.speed_knots, Some(22.4));
        assert!((st.speed_kmh().unwrap() - 41.4848).abs() < 1e-3);
        assert!((st.speed_ms().unwrap() - 11.5236).abs() < 1e-3);
        assert_eq!(st.utc_date, "2094-03-23");
    }

    #[test]
    fn gga_gsv() {
        let mut st = GpsState::default();
        parse(
            "$GPGGA,123519,4807.038,N,01131.000,W,1,08,0.9,545.4,M,46.9,M,,*55",
            &mut st,
        );
        assert!(st.lon.unwrap() < 0.0);
        assert_eq!(st.sats_used, Some(8));
        assert_eq!(st.alt, Some(545.4));
        parse(
            "$GPGSV,2,1,08,01,40,083,46,02,17,308,41,12,07,344,39,14,22,228,45*75",
            &mut st,
        );
        assert_eq!(st.sats.len(), 4);
        assert_eq!(st.snr_stats[&("GPS", 1)].sum, 46.0);
        st.used.insert(("GPS", 1), Instant::now());
        parse("$GPGSV,2,2,08,15,10,100,*45", &mut st); // no C/N0
        assert_eq!(
            st.quality_line().unwrap(),
            "inView=5 used=1 meanCN0=42.8 meanCN0used=46.0 G01:U:46.0 G02:V:41.0 G12:V:39.0 G14:V:45.0 G15:V:-"
        );
        assert!(st.quality_line().is_none());
    }

    #[test]
    fn decode_text() {
        let d = decode("$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A");
        assert!(d.contains("lat=48.1173000N lon=11.5166667E"), "{d}");
        assert!(
            d.contains("speed=22.400 kn = 41.485 km/h = 11.524 m/s"),
            "{d}"
        );
        let d = decode("$GPGSV,2,1,08,01,40,083,46,02,17,308,41,12,07,344,39,14,22,228,45*75");
        assert_eq!(d.lines().count(), 5, "{d}");
        assert!(decode("$GPGGA,1*00").starts_with("CHECKSUM ERROR"));
    }

    fn cs(body: &str) -> String {
        format!("${body}*{:02X}", body.bytes().fold(0u8, |a, c| a ^ c))
    }

    fn keys(m: impl Iterator<Item = SatKey>) -> Vec<SatKey> {
        let mut v: Vec<_> = m.collect();
        v.sort();
        v
    }

    #[test]
    fn gsv_signal_id() {
        // NMEA 4.10: two satellites and a trailing signal ID (B1)
        let mut st = GpsState::default();
        parse(&cs("GPGSV,1,1,02,01,40,083,46,05,17,308,41,1"), &mut st);
        assert_eq!(keys(st.sats.keys().copied()), [("GPS", 1), ("GPS", 5)]);
        assert_eq!(st.sats[&("GPS", 1)].elev, Some(40.0));
        // second signal of the same satellites: weaker / not tracked - strongest is shown
        parse(&cs("GPGSV,1,1,02,01,40,083,38,05,17,308,,6"), &mut st);
        assert_eq!(st.sats[&("GPS", 1)].snr, Some(46.0));
        assert_eq!(st.sats[&("GPS", 5)].snr, Some(41.0));
        assert_eq!(st.sats.len(), 2);
    }

    #[test]
    fn extended_numbering() {
        // NMEA 4.0 GNGSA without system ID, u-blox extended numbers (B6)
        let mut st = GpsState::default();
        parse(&cs("GNGSA,A,3,301,402,70,,,,,,,,,,1.5,0.8,1.2"), &mut st);
        parse(&cs("GAGSV,1,1,01,301,40,083,46"), &mut st);
        parse(&cs("GBGSV,1,1,01,402,40,083,46"), &mut st);
        parse(&cs("GLGSV,1,1,01,70,40,083,46"), &mut st);
        let sats = keys(st.sats.keys().copied());
        assert_eq!(sats, [("BeiDou", 2), ("GLONASS", 6), ("Galileo", 1)]);
        assert_eq!(keys(st.used.keys().copied()), sats);
    }

    #[test]
    fn checksum_required() {
        // a sentence without checksum must not change the state (B9)
        let mut st = GpsState::default();
        parse(
            "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,",
            &mut st,
        );
        assert!(st.lat.is_none() && st.epoch.is_none());
        parse(
            &cs("GPGGA,123519.00,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,"),
            &mut st,
        );
        assert_eq!(st.epoch, Some(12 * 3600 + 35 * 60 + 19));
    }
}
