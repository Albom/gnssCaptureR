//! Program settings, stored in a standard INI file next to the executable.

use ini::Ini;
use std::path::PathBuf;

#[derive(Clone)]
pub struct Settings {
    // [Serial]
    pub port: String,
    pub baud: u32,
    pub data_bits: u8,  // 5..8
    pub parity: String, // None / Odd / Even
    pub stop_bits: u8,  // 1 or 2
    // [Capture]
    pub directory: String,
    pub prefix: String,
    pub block_size: usize,
    pub save_nmea: bool,
    pub save_nmea_text: bool,
    pub save_ubx: bool,
    pub save_ubx_text: bool,
    pub save_quality: bool, // signal quality every second
    // [Display]
    pub quality_period: u64, // seconds between Quality tab updates
    pub time_font_size: f32, // font size of the time on the Date and time tab
    // [Map]
    pub maps_dir: String, // relative: next to the program, else the current folder
    pub map_file: String, // shapefile for wide views
    pub map_detailed: String, // shapefile used when the visible width is below detail_below_deg
    pub detail_below_deg: f64,
}

impl Default for Settings {
    fn default() -> Self {
        Settings {
            port: if cfg!(windows) {
                "COM1".into()
            } else {
                "/dev/ttyUSB0".into()
            },
            baud: 9600,
            data_bits: 8,
            parity: "None".into(),
            stop_bits: 1,
            directory: "data".into(),
            prefix: "gps".into(),
            block_size: 20,
            save_nmea: true,
            save_nmea_text: false,
            save_ubx: true,
            save_ubx_text: true,
            save_quality: true,
            quality_period: 10,
            time_font_size: 96.0,
            maps_dir: "Maps".into(),
            map_file: "50m/ne_50m_land.shp".into(),
            map_detailed: "10m/ne_10m_land.shp".into(),
            detail_below_deg: 30.0,
        }
    }
}

/// Folder of the executable.
pub fn exe_dir() -> PathBuf {
    std::env::current_exe()
        .ok()
        .and_then(|p| p.parent().map(|d| d.to_path_buf()))
        .unwrap_or_else(|| PathBuf::from("."))
}

pub fn ini_path() -> PathBuf {
    exe_dir().join("gnssCaptureR.ini")
}

/// Reads one key. A comment after the value ("10  ; seconds") is cut off.
/// A key that is present but cannot be read is added to `bad` and the default is used.
fn get<T: std::str::FromStr>(
    ini: &Ini,
    sec: &str,
    key: &str,
    default: T,
    bad: &mut Vec<String>,
) -> T {
    let Some(v) = ini.get_from(Some(sec), key) else {
        return default;
    };
    let comment = v
        .char_indices()
        .find(|&(i, c)| c == ';' && v[..i].ends_with([' ', '\t']));
    let v = comment.map_or(v, |(i, _)| &v[..i]).trim();
    v.parse().unwrap_or_else(|_| {
        bad.push(key.to_string());
        default
    })
}

/// Earlier versions wrote every backslash twice (D:\\data); such a path is read back as intended.
fn path(s: String) -> String {
    let keep = if s.starts_with(r"\\\\") {
        0
    } else {
        s.len().min(2)
    }; // "\\server" stays
    match s.is_char_boundary(keep) {
        true => format!("{}{}", &s[..keep], s[keep..].replace(r"\\", r"\")),
        false => s,
    }
}

impl Settings {
    /// Returns the settings and the names of keys that could not be read.
    pub fn load() -> (Settings, Vec<String>) {
        // "noescape": a backslash is an ordinary character, so Windows paths can be
        // typed as they are (D:\data\new)
        match Ini::load_from_file_noescape(ini_path()) {
            Ok(ini) => Settings::from_ini(&ini),
            Err(ini::Error::Io(_)) => (Settings::default(), Vec::new()), // no file yet
            Err(e) => (Settings::default(), vec![e.to_string()]),
        }
    }

    fn from_ini(ini: &Ini) -> (Settings, Vec<String>) {
        let d = Settings::default();
        let mut bad = Vec::new();
        let b = &mut bad;
        let s = Settings {
            port: get(ini, "Serial", "Port", d.port, b),
            baud: get(ini, "Serial", "BaudRate", d.baud, b),
            data_bits: get(ini, "Serial", "DataBits", d.data_bits, b),
            parity: get(ini, "Serial", "Parity", d.parity, b),
            stop_bits: get(ini, "Serial", "StopBits", d.stop_bits, b),
            directory: path(get(ini, "Capture", "Directory", d.directory, b)),
            prefix: get(ini, "Capture", "Prefix", d.prefix, b),
            block_size: get(ini, "Capture", "BlockSize", d.block_size, b).max(1),
            save_nmea: get(ini, "Capture", "SaveNMEA", d.save_nmea, b),
            save_ubx: get(ini, "Capture", "SaveUBX", d.save_ubx, b),
            save_ubx_text: get(ini, "Capture", "SaveUBXDecoded", d.save_ubx_text, b),
            save_nmea_text: get(ini, "Capture", "SaveNMEADecoded", d.save_nmea_text, b),
            save_quality: get(ini, "Capture", "SaveQuality", d.save_quality, b),
            quality_period: get(ini, "Display", "QualityPeriod", d.quality_period, b).max(1),
            time_font_size: get(ini, "Display", "TimeFontSize", d.time_font_size, b)
                .clamp(8.0, 500.0),
            maps_dir: path(get(ini, "Map", "Directory", d.maps_dir, b)),
            map_file: path(get(ini, "Map", "File", d.map_file, b)),
            map_detailed: path(get(ini, "Map", "DetailedFile", d.map_detailed, b)),
            detail_below_deg: get(ini, "Map", "DetailedBelowDegrees", d.detail_below_deg, b),
        };
        (s, bad)
    }

    pub fn save(&self) -> std::io::Result<()> {
        let mut ini = Ini::new();
        ini.with_section(Some("Serial"))
            .set("Port", &self.port)
            .set("BaudRate", self.baud.to_string())
            .set("DataBits", self.data_bits.to_string())
            .set("Parity", &self.parity)
            .set("StopBits", self.stop_bits.to_string());
        ini.with_section(Some("Capture"))
            .set("Directory", &self.directory)
            .set("Prefix", &self.prefix)
            .set("BlockSize", self.block_size.to_string())
            .set("SaveNMEA", self.save_nmea.to_string())
            .set("SaveNMEADecoded", self.save_nmea_text.to_string())
            .set("SaveUBX", self.save_ubx.to_string())
            .set("SaveUBXDecoded", self.save_ubx_text.to_string())
            .set("SaveQuality", self.save_quality.to_string());
        ini.with_section(Some("Display"))
            .set("QualityPeriod", self.quality_period.to_string())
            .set("TimeFontSize", self.time_font_size.to_string());
        ini.with_section(Some("Map"))
            .set("Directory", &self.maps_dir)
            .set("File", &self.map_file)
            .set("DetailedFile", &self.map_detailed)
            .set("DetailedBelowDegrees", self.detail_below_deg.to_string());
        // backslashes are written as they are, not doubled
        ini.write_to_file_policy(ini_path(), ini::EscapePolicy::Nothing)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn comments_and_bad_values() {
        // B7: comments after values are accepted, unreadable keys are reported
        let text = "[Serial]\nPort=COM4 ; receiver\nBaudRate=115200\t; fast\nParity=None   ; None, Odd, Even\n\
                    DataBits=eight\n[Capture]\nDirectory=D:\\data;old\n[Display]\nQualityPeriod=10   ; seconds\n";
        let (s, bad) = Settings::from_ini(&Ini::load_from_str_noescape(text).unwrap());
        assert_eq!(
            (s.port.as_str(), s.baud, s.parity.as_str()),
            ("COM4", 115200, "None")
        );
        assert_eq!((s.quality_period, s.data_bits), (10, 8));
        assert_eq!(s.directory, r"D:\data;old"); // ';' inside a value is kept
        assert_eq!(bad, ["DataBits"]);
    }

    #[test]
    fn windows_paths() {
        // B17: backslashes are ordinary characters; doubled ones of old files are repaired
        for (written, read) in [
            (r"D:\Albom\new\xdata", r"D:\Albom\new\xdata"),
            (r"D:\\Albom\\data", r"D:\Albom\data"),
            (r"\\server\share\gps", r"\\server\share\gps"),
            (r"\\\\server\\share", r"\\server\share"),
            ("data", "data"),
        ] {
            let ini =
                Ini::load_from_str_noescape(&format!("[Capture]\nDirectory={written}\n")).unwrap();
            assert_eq!(Settings::from_ini(&ini).0.directory, read);
        }
    }
}
