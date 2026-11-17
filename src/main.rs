// Hide the console window on Windows in release builds.
#![cfg_attr(not(debug_assertions), windows_subsystem = "windows")]

mod config;
mod logger;
mod map;
mod nmea;
mod reader;
mod ublox;
mod ubx;

use config::Settings;
use eframe::egui::{self, Align2, Color32, FontId, Pos2, RichText, Sense, Stroke, Vec2};
use reader::{Shared, lock};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

const BAUD_RATES: [u32; 9] = [
    4800, 9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600,
];
const BLOCK_SIZES: [usize; 4] = [10, 20, 50, 100];

#[derive(PartialEq, Clone, Copy)]
enum Tab {
    Connection,
    Information,
    SkyMap,
    Quality,
    Map,
    DateTime,
}

const TABS: [(Tab, &str); 6] = [
    (Tab::Connection, "Serial Port & Capture"),
    (Tab::Information, "Information"),
    (Tab::SkyMap, "Satellite map"),
    (Tab::Quality, "Quality"),
    (Tab::Map, "Map"),
    (Tab::DateTime, "Date and time"),
];

/// One bar of the Quality tab: C/N0 of a satellite averaged over the period.
struct QualityRow {
    sys: &'static str,
    prn: u16,
    mean: f64,
    min: f64,
    max: f64,
    used: bool,
}

struct App {
    cfg: Settings,
    tab: Tab,
    shared: Arc<Mutex<Shared>>,
    stop: Arc<AtomicBool>,
    thread: Option<JoinHandle<()>>,
    scan: Arc<Mutex<PortScan>>,
    paused: bool,
    stream: Vec<String>,
    quality: Vec<QualityRow>,
    quality_next: Instant,
    quality_time: String,
    map: map::MapView,
    clock_diff: String,     // time difference shown on the Date and time tab
    clock_second: i64,      // PC second for which clock_diff was taken
    notice: Option<String>, // problem with the settings file, shown in the status bar
}

/// One serial port of the port list.
#[derive(Clone)]
struct PortEntry {
    name: String,
    label: String,               // name, with the receiver model when it is a u-blox
    usb_ublox: bool,             // native u-blox USB port (no baud rate)
    probe: Option<ublox::Probe>, // result of the check made by Refresh
}

/// Port check and receiver configuration running in background threads.
#[derive(Default)]
struct PortScan {
    ports: Vec<PortEntry>,
    running: usize, // threads still working
    status: String,
    /// Set when configuring a u-blox receiver has ended: the receiver's baud rate
    /// (if known) and whether the configuration succeeded.
    configured: Option<(Option<u32>, bool)>,
}

fn list_ports() -> Vec<PortEntry> {
    serialport::available_ports()
        .unwrap_or_default()
        .into_iter()
        .map(|p| PortEntry {
            label: p.port_name.clone(),
            usb_ublox: matches!(p.port_type, serialport::SerialPortType::UsbPort(ref u) if u.vid == 0x1546),
            name: p.port_name,
            probe: None,
        })
        .collect()
}

fn opt(v: Option<f64>, decimals: usize, unit: &str) -> String {
    v.map(|x| format!("{x:.decimals$} {unit}"))
        .unwrap_or_else(|| "—".into())
}

fn deg(v: Option<f64>, pos: char, neg: char) -> String {
    match v {
        Some(x) => {
            // round to 0.001" first, then split, so 59.9996" becomes the next minute
            let mas = (x.abs() * 3_600_000.0).round() as u64;
            let (d, m, s) = (
                mas / 3_600_000,
                mas / 60_000 % 60,
                (mas % 60_000) as f64 / 1000.0,
            );
            format!(
                "{:.7}° {}    ({d}° {m}' {s:.3}\")",
                x.abs(),
                if x < 0.0 { neg } else { pos }
            )
        }
        None => "—".into(),
    }
}

/// PC time minus receiver time, both taken when the last time message arrived (seconds).
fn time_diff(st: &nmea::GpsState) -> Option<f64> {
    let pc = st.time_pc?;
    let t = chrono::NaiveTime::parse_from_str(&st.utc_time, "%H:%M:%S%.f").ok()?;
    let d = chrono::NaiveDate::parse_from_str(&st.utc_date, "%Y-%m-%d").unwrap_or(pc.date_naive());
    let mut diff = (pc - d.and_time(t).and_utc()).num_microseconds()? as f64 / 1e6;
    // receiver date not known yet / from the previous day: correct around midnight
    if diff > 43200.0 {
        diff -= 86400.0;
    } else if diff < -43200.0 {
        diff += 86400.0;
    }
    Some(diff)
}

fn cno_color(v: f64) -> Color32 {
    match v {
        v if v < 25.0 => Color32::from_rgb(210, 50, 50),
        v if v < 35.0 => Color32::from_rgb(230, 150, 30),
        v if v < 45.0 => Color32::from_rgb(170, 200, 40),
        _ => Color32::from_rgb(40, 170, 60),
    }
}

fn sat_color(sys: &str) -> Color32 {
    match sys {
        "GPS" => Color32::from_rgb(40, 160, 60),
        "GLONASS" => Color32::from_rgb(210, 50, 50),
        "Galileo" => Color32::from_rgb(40, 100, 220),
        "BeiDou" => Color32::from_rgb(230, 140, 20),
        "QZSS" => Color32::from_rgb(150, 60, 200),
        "NavIC" => Color32::from_rgb(140, 90, 40),
        _ => Color32::GRAY,
    }
}

impl App {
    fn connected(&self) -> bool {
        lock(&self.shared).running
    }

    fn connect(&mut self) {
        self.disconnect();
        self.save_settings();
        {
            let mut sh = lock(&self.shared);
            *sh = Shared::default();
            sh.running = true;
        }
        self.stop.store(false, Ordering::Relaxed);
        let (cfg, shared, stop) = (self.cfg.clone(), self.shared.clone(), self.stop.clone());
        self.thread = Some(std::thread::spawn(move || {
            // an internal error must not leave the window showing "Connected"
            let run = || reader::run(cfg, shared.clone(), stop);
            if std::panic::catch_unwind(std::panic::AssertUnwindSafe(run)).is_err() {
                let mut sh = lock(&shared);
                sh.running = false;
                sh.reconnecting = false;
                sh.error = Some("Capture stopped by an internal error. Press Connect.".into());
            }
        }));
    }

    /// Saves the INI file; a failure is shown in the status bar.
    fn save_settings(&mut self) {
        self.notice = self
            .cfg
            .save()
            .err()
            .map(|e| format!("Settings not saved ({}): {e}", config::ini_path().display()));
    }

    fn disconnect(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
        if let Some(t) = self.thread.take() {
            let _ = t.join(); // the thread flushes the capture files before it ends
        }
    }

    /// Refresh: lists the ports and checks each one (in parallel) for a u-blox receiver.
    fn refresh_ports(&mut self) {
        let ports = list_ports();
        let mut sc = lock(&self.scan);
        sc.running = ports.len();
        sc.status = if ports.is_empty() {
            String::new()
        } else {
            "Checking ports…".into()
        };
        sc.ports = ports.clone();
        for p in &mut sc.ports {
            p.label = format!("{} (checking…)", p.name);
        }
        for (i, p) in ports.into_iter().enumerate() {
            let scan = self.scan.clone();
            std::thread::spawn(move || {
                let probe = ublox::probe(&p.name);
                let mut sc = lock(&scan);
                if let Some(e) = sc.ports.get_mut(i) {
                    e.label = match &probe.ublox {
                        Some(rx) => format!("{} ({})", e.name, rx.model),
                        None => e.name.clone(),
                    };
                    e.probe = Some(probe);
                }
                sc.running -= 1;
                if sc.running == 0 {
                    sc.status.clear();
                }
            });
        }
    }

    /// A port was chosen from the list: configure a u-blox receiver, otherwise
    /// take the detected baud rate and switch the UBX files off.
    fn choose_port(&mut self, p: PortEntry) {
        self.cfg.port = p.name.clone();
        let Some(probe) = p.probe else { return }; // not checked yet
        let (Some(rx), Some(baud)) = (probe.ublox, probe.baud) else {
            if let Some(b) = probe.baud {
                self.cfg.baud = b;
            }
            self.cfg.save_ubx = false;
            self.cfg.save_ubx_text = false;
            return;
        };
        let scan = self.scan.clone();
        {
            let mut sc = lock(&scan);
            sc.running += 1;
            sc.status = format!("Configuring {} ({})…", p.name, rx.model);
        }
        std::thread::spawn(move || {
            let result = ublox::configure(&p.name, &rx, baud, p.usb_ublox);
            let (configured, status) = match result {
                Ok((b, text)) => ((Some(b), true), text),
                Err(e) => {
                    // the receiver may or may not have changed its baud rate: look again
                    lock(&scan).status = format!("{e}. Checking the port again…");
                    let found = ublox::probe(&p.name).baud;
                    let text = match found {
                        Some(b) => {
                            format!("{e}. Setup not confirmed; the receiver answers at {b} baud.")
                        }
                        None => format!("{e}. The receiver does not answer."),
                    };
                    ((found, false), text)
                }
            };
            let mut sc = lock(&scan);
            sc.configured = Some(configured);
            sc.status = status;
            sc.running -= 1;
        });
    }

    fn connection_tab(&mut self, ui: &mut egui::Ui) {
        let connected = self.connected();
        ui.heading("Serial port");
        ui.add_enabled_ui(!connected, |ui| {
            egui::Grid::new("serial")
                .num_columns(2)
                .spacing([20.0, 8.0])
                .show(ui, |ui| {
                    ui.label("Port");
                    ui.horizontal(|ui| {
                        ui.add(egui::TextEdit::singleline(&mut self.cfg.port).desired_width(160.0));
                        let (ports, running, status) = {
                            let sc = lock(&self.scan);
                            (sc.ports.clone(), sc.running, sc.status.clone())
                        };
                        let mut chosen = None;
                        egui::ComboBox::from_id_salt("ports")
                            .selected_text("Select")
                            .show_ui(ui, |ui| {
                                for p in &ports {
                                    if ui
                                        .selectable_label(self.cfg.port == p.name, &p.label)
                                        .clicked()
                                    {
                                        chosen = Some(p.clone());
                                    }
                                }
                            });
                        if let Some(p) = chosen {
                            self.choose_port(p);
                        }
                        if ui
                            .add_enabled(running == 0, egui::Button::new("Refresh"))
                            .clicked()
                        {
                            self.refresh_ports();
                        }
                        if running > 0 {
                            ui.spinner();
                        }
                        ui.label(status);
                    });
                    ui.end_row();
                    ui.label("Baud rate");
                    egui::ComboBox::from_id_salt("baud")
                        .selected_text(self.cfg.baud.to_string())
                        .show_ui(ui, |ui| {
                            for b in BAUD_RATES {
                                ui.selectable_value(&mut self.cfg.baud, b, b.to_string());
                            }
                        });
                    ui.end_row();
                    ui.label("Data bits");
                    egui::ComboBox::from_id_salt("bits")
                        .selected_text(self.cfg.data_bits.to_string())
                        .show_ui(ui, |ui| {
                            for b in [5u8, 6, 7, 8] {
                                ui.selectable_value(&mut self.cfg.data_bits, b, b.to_string());
                            }
                        });
                    ui.end_row();
                    ui.label("Parity");
                    egui::ComboBox::from_id_salt("parity")
                        .selected_text(self.cfg.parity.clone())
                        .show_ui(ui, |ui| {
                            for p in ["None", "Odd", "Even"] {
                                ui.selectable_value(&mut self.cfg.parity, p.to_string(), p);
                            }
                        });
                    ui.end_row();
                    ui.label("Stop bits");
                    egui::ComboBox::from_id_salt("stop")
                        .selected_text(self.cfg.stop_bits.to_string())
                        .show_ui(ui, |ui| {
                            for s in [1u8, 2] {
                                ui.selectable_value(&mut self.cfg.stop_bits, s, s.to_string());
                            }
                        });
                    ui.end_row();
                });

            ui.separator();
            ui.heading("Capture (hourly files)");
            egui::Grid::new("capture")
                .num_columns(2)
                .spacing([20.0, 8.0])
                .show(ui, |ui| {
                    ui.label("Directory");
                    ui.add(
                        egui::TextEdit::singleline(&mut self.cfg.directory).desired_width(360.0),
                    );
                    ui.end_row();
                    ui.label("File prefix");
                    ui.add(egui::TextEdit::singleline(&mut self.cfg.prefix).desired_width(160.0));
                    ui.end_row();
                    ui.label("Write block (messages)");
                    egui::ComboBox::from_id_salt("block")
                        .selected_text(self.cfg.block_size.to_string())
                        .show_ui(ui, |ui| {
                            for b in BLOCK_SIZES {
                                ui.selectable_value(&mut self.cfg.block_size, b, b.to_string());
                            }
                        });
                    ui.end_row();
                    ui.label("Files");
                    ui.vertical(|ui| {
                        ui.checkbox(&mut self.cfg.save_nmea, "NMEA messages (.nmea)");
                        ui.checkbox(
                            &mut self.cfg.save_nmea_text,
                            "NMEA decoded text (.nmea.txt)",
                        );
                        ui.checkbox(&mut self.cfg.save_ubx, "UBX raw binary (.ubx)");
                        ui.checkbox(&mut self.cfg.save_ubx_text, "UBX decoded text (.ubx.txt)");
                        ui.checkbox(
                            &mut self.cfg.save_quality,
                            "Signal quality every second (.quality.txt)",
                        );
                    });
                    ui.end_row();
                });
        });
        let example =
            logger::Logger::new(&self.cfg).path(logger::hour_of(chrono::Utc::now()), "nmea");
        ui.label(RichText::new(format!("Current file: {}", example.display())).weak());

        ui.add_space(12.0);
        let text = if connected { "Disconnect" } else { "Connect" };
        let busy = lock(&self.scan).running > 0; // ports are being checked / configured
        if ui
            .add_enabled(
                connected || !busy,
                egui::Button::new(RichText::new(text).size(16.0)).min_size(Vec2::new(140.0, 32.0)),
            )
            .clicked()
        {
            if connected {
                self.disconnect()
            } else {
                self.connect()
            }
        }
        ui.separator();
        self.stream_tab(ui);
    }

    fn datetime_tab(&mut self, ui: &mut egui::Ui) {
        // The times are shown as they are now. The difference is taken once per PC
        // second, so it does not flicker between the receiver's time messages.
        let now = chrono::Utc::now();
        let (rx_time, rx_date) = {
            let sh = lock(&self.shared);
            if now.timestamp() != self.clock_second {
                self.clock_second = now.timestamp();
                self.clock_diff = match time_diff(&sh.state) {
                    Some(d) => format!("{d:+.3} s"),
                    None => "—".to_string(),
                };
            }
            let time = sh
                .state
                .utc_time
                .split('.')
                .next()
                .unwrap_or("")
                .to_string();
            (time, sh.state.utc_date.clone())
        };
        // repaint exactly when the PC's next second starts
        let to_next = 1000 - now.timestamp_subsec_millis().min(999) as u64;
        ui.ctx()
            .request_repaint_after(Duration::from_millis(to_next + 2));
        let c = [
            rx_time,
            rx_date,
            now.format("%H:%M:%S").to_string(),
            now.format("%Y-%m-%d").to_string(),
            self.clock_diff.clone(),
        ];
        let size = self.cfg.time_font_size;
        let or_dash = |s: &String| {
            if s.is_empty() {
                "--:--:--".to_string()
            } else {
                s.clone()
            }
        };
        let panels = [
            ("Receiver (NMEA)", or_dash(&c[0]), format!("{}  UTC", c[1])),
            ("PC", c[2].clone(), format!("{}  UTC", c[3])),
            ("Difference PC − receiver", c[4].clone(), String::new()),
        ];
        let w = ui.available_width();
        let h = (ui.available_height() - 2.0 * ui.spacing().item_spacing.y - 12.0) / 3.0;
        let content = 22.0 + size * 1.25 + size * 0.45;
        for (i, (title, big, small)) in panels.into_iter().enumerate() {
            if i > 0 {
                ui.separator();
            }
            ui.allocate_ui(Vec2::new(w, h), |ui| {
                ui.set_min_size(Vec2::new(w, h));
                ui.vertical_centered(|ui| {
                    ui.add_space(((h - content) / 2.0).max(0.0));
                    ui.label(RichText::new(title).size(16.0).weak());
                    ui.label(RichText::new(big).size(size).monospace().strong());
                    ui.label(RichText::new(small).size(size * 0.35));
                });
            });
        }
    }

    fn info_tab(&self, ui: &mut egui::Ui) {
        let sh = lock(&self.shared);
        let s = &sh.state;
        let big = |t: String| RichText::new(t).size(16.0).monospace();
        egui::Grid::new("info")
            .num_columns(2)
            .spacing([30.0, 10.0])
            .striped(true)
            .show(ui, |ui| {
                let mut row = |name: &str, value: String| {
                    ui.label(RichText::new(name).size(16.0));
                    ui.label(big(value));
                    ui.end_row();
                };
                row("UTC date / time", format!("{}  {}", s.utc_date, s.utc_time));
                row("Latitude", deg(s.lat, 'N', 'S'));
                row("Longitude", deg(s.lon, 'E', 'W'));
                row("Altitude (MSL)", opt(s.alt, 2, "m"));
                row("Heading", opt(s.heading, 1, "°"));
                row("Speed", opt(s.speed_knots, 2, "knots"));
                row("", opt(s.speed_kmh(), 2, "km/h"));
                row("", opt(s.speed_ms(), 2, "m/s"));
                row("GPS type", s.gps_type());
                row(
                    "Satellites tracked",
                    format!(
                        "{} used / {} in view",
                        s.sats_used.map(|v| v.to_string()).unwrap_or("—".into()),
                        s.sats.len()
                    ),
                );
                row("HDOP", opt(s.hdop, 2, ""));
                row("VDOP", opt(s.vdop, 2, ""));
                row("PDOP", opt(s.pdop, 2, ""));
            });
    }

    fn sky_tab(&self, ui: &mut egui::Ui) {
        let sh = lock(&self.shared);
        let mut sats: Vec<_> = sh.state.sats.iter().collect();
        sats.sort_by_key(|(k, _)| nmea::sat_order(k));
        ui.horizontal_top(|ui| {
            let side = ui
                .available_height()
                .min(ui.available_width() - 330.0)
                .max(200.0);
            let (resp, painter) = ui.allocate_painter(Vec2::splat(side), Sense::hover());
            let c = resp.rect.center();
            let r = side / 2.0 - 24.0;
            let line = Stroke::new(1.0, Color32::GRAY);
            let text_col = ui.visuals().text_color();
            for e in [0.0, 30.0, 60.0] {
                painter.circle_stroke(c, r * (90.0 - e) / 90.0, line);
                if e > 0.0 {
                    painter.text(
                        c + Vec2::new(3.0, -r * (90.0 - e as f32) / 90.0),
                        Align2::LEFT_BOTTOM,
                        format!("{e}°"),
                        FontId::proportional(10.0),
                        Color32::GRAY,
                    );
                }
            }
            painter.line_segment([c - Vec2::new(r, 0.0), c + Vec2::new(r, 0.0)], line);
            painter.line_segment([c - Vec2::new(0.0, r), c + Vec2::new(0.0, r)], line);
            for (t, d) in [
                ("N", Vec2::new(0.0, -1.0)),
                ("E", Vec2::new(1.0, 0.0)),
                ("S", Vec2::new(0.0, 1.0)),
                ("W", Vec2::new(-1.0, 0.0)),
            ] {
                painter.text(
                    c + d * (r + 12.0),
                    Align2::CENTER_CENTER,
                    t,
                    FontId::proportional(14.0),
                    text_col,
                );
            }
            for ((sys, prn), sat) in &sats {
                let (Some(el), Some(az)) = (sat.elev, sat.az) else {
                    continue;
                };
                let d = r * (90.0 - el.clamp(0.0, 90.0) as f32) / 90.0;
                let a = (az as f32).to_radians();
                let p = Pos2::new(c.x + d * a.sin(), c.y - d * a.cos());
                let col = sat_color(sys);
                if sh.state.is_used(&(*sys, *prn)) {
                    painter.circle_filled(p, 9.0, col);
                } else {
                    painter.circle_stroke(p, 9.0, Stroke::new(2.0, col));
                }
                painter.text(
                    p + Vec2::new(0.0, -15.0),
                    Align2::CENTER_CENTER,
                    prn.to_string(),
                    FontId::proportional(11.0),
                    text_col,
                );
            }

            ui.vertical(|ui| {
                ui.horizontal_wrapped(|ui| {
                    for s in ["GPS", "GLONASS", "Galileo", "BeiDou", "QZSS", "SBAS"] {
                        ui.label(RichText::new("●").color(sat_color(s)));
                        ui.label(s);
                    }
                });
                ui.label(RichText::new("Filled = used in fix").weak());
                egui::ScrollArea::vertical().show(ui, |ui| {
                    egui::Grid::new("sats").striped(true).show(ui, |ui| {
                        for h in ["System", "PRN", "Elev", "Azim", "SNR", "Used"] {
                            ui.strong(h);
                        }
                        ui.end_row();
                        let f = |v: Option<f64>| v.map(|x| format!("{x:.0}")).unwrap_or_default();
                        for ((sys, prn), sat) in &sats {
                            ui.label(RichText::new(*sys).color(sat_color(sys)));
                            ui.label(prn.to_string());
                            ui.label(f(sat.elev));
                            ui.label(f(sat.az));
                            ui.label(f(sat.snr));
                            ui.label(if sh.state.is_used(&(*sys, *prn)) {
                                "✔"
                            } else {
                                ""
                            });
                            ui.end_row();
                        }
                    });
                });
            });
        });
    }

    /// Every QualityPeriod seconds: average the collected C/N0 values per satellite.
    fn update_quality(&mut self) {
        if Instant::now() < self.quality_next {
            return;
        }
        self.quality_next = Instant::now() + Duration::from_secs(self.cfg.quality_period.max(1));
        let mut sh = lock(&self.shared);
        let stats = std::mem::take(&mut sh.state.snr_stats);
        self.quality = stats
            .into_iter()
            .map(|(key, st)| QualityRow {
                sys: key.0,
                prn: key.1,
                mean: st.sum / st.n as f64,
                min: st.min,
                max: st.max,
                used: sh.state.is_used(&key),
            })
            .collect();
        self.quality
            .sort_by_key(|r| nmea::sat_order(&(r.sys, r.prn)));
        self.quality_time = chrono::Utc::now().format("%H:%M:%S UTC").to_string();
    }

    fn quality_tab(&mut self, ui: &mut egui::Ui) {
        ui.horizontal(|ui| {
            ui.label("Update period");
            ui.add(
                egui::DragValue::new(&mut self.cfg.quality_period)
                    .range(1..=3600)
                    .suffix(" s"),
            );
            ui.separator();
            ui.label(format!("Last update: {}", self.quality_time));
            ui.separator();
            ui.label(
                RichText::new(
                    "Bar = mean C/N0 over the period, whisker = min..max, bold label = used in fix",
                )
                .weak(),
            );
        });
        if self.quality.is_empty() {
            ui.label("No signal data yet.");
            return;
        }

        // ---- bar chart, 0..60 dBHz
        let h = (ui.available_height() * 0.6).max(200.0);
        let (resp, painter) =
            ui.allocate_painter(Vec2::new(ui.available_width(), h), Sense::hover());
        let rect = resp.rect;
        let (left, bottom, top) = (rect.left() + 36.0, rect.bottom() - 34.0, rect.top() + 22.0);
        let y = |v: f64| bottom - (v.clamp(0.0, 60.0) / 60.0) as f32 * (bottom - top);
        let text_col = ui.visuals().text_color();
        for v in (0..=60).step_by(10) {
            let yy = y(v as f64);
            painter.line_segment(
                [Pos2::new(left, yy), Pos2::new(rect.right(), yy)],
                Stroke::new(1.0, Color32::from_gray(70)),
            );
            painter.text(
                Pos2::new(left - 4.0, yy),
                Align2::RIGHT_CENTER,
                v.to_string(),
                FontId::proportional(11.0),
                text_col,
            );
        }
        painter.text(
            Pos2::new(rect.left(), rect.top()),
            Align2::LEFT_TOP,
            "dBHz",
            FontId::proportional(10.0),
            Color32::GRAY,
        );
        let slot = ((rect.right() - left) / self.quality.len() as f32).min(40.0);
        for (i, r) in self.quality.iter().enumerate() {
            let x0 = left + slot * i as f32 + slot * 0.15;
            let x1 = left + slot * (i + 1) as f32 - slot * 0.15;
            let xm = (x0 + x1) / 2.0;
            painter.rect_filled(
                egui::Rect::from_min_max(Pos2::new(x0, y(r.mean)), Pos2::new(x1, bottom)),
                2.0,
                cno_color(r.mean),
            );
            painter.line_segment(
                [Pos2::new(xm, y(r.min)), Pos2::new(xm, y(r.max))],
                Stroke::new(1.5, text_col),
            );
            painter.text(
                Pos2::new(xm, y(r.max) - 2.0),
                Align2::CENTER_BOTTOM,
                format!("{:.0}", r.mean),
                FontId::proportional(10.0),
                text_col,
            );
            let label = format!("{}{:02}", nmea::sys_letter(r.sys), r.prn);
            let font = FontId::proportional(if r.used { 12.0 } else { 10.0 });
            let col = if r.used {
                sat_color(r.sys)
            } else {
                Color32::GRAY
            };
            painter.text(
                Pos2::new(xm, bottom + 4.0),
                Align2::CENTER_TOP,
                label,
                font,
                col,
            );
        }

        // ---- summary per constellation
        ui.add_space(6.0);
        egui::Grid::new("quality_sum")
            .striped(true)
            .spacing([24.0, 4.0])
            .show(ui, |ui| {
                for h in [
                    "System",
                    "Satellites",
                    "Used",
                    "Mean C/N0",
                    "Min",
                    "Max",
                    "≥ 35 dBHz",
                ] {
                    ui.strong(h);
                }
                ui.end_row();
                let mut systems: Vec<&str> = self.quality.iter().map(|r| r.sys).collect();
                systems.dedup();
                for sys in systems.into_iter().chain(["All"]) {
                    let rows: Vec<&QualityRow> = self
                        .quality
                        .iter()
                        .filter(|r| sys == "All" || r.sys == sys)
                        .collect();
                    let n = rows.len() as f64;
                    let mean = rows.iter().map(|r| r.mean).sum::<f64>() / n;
                    ui.label(RichText::new(sys).color(sat_color(sys)));
                    ui.label(rows.len().to_string());
                    ui.label(rows.iter().filter(|r| r.used).count().to_string());
                    ui.label(RichText::new(format!("{mean:.1}")).color(cno_color(mean)));
                    ui.label(format!(
                        "{:.0}",
                        rows.iter().map(|r| r.min).fold(f64::MAX, f64::min)
                    ));
                    ui.label(format!(
                        "{:.0}",
                        rows.iter().map(|r| r.max).fold(f64::MIN, f64::max)
                    ));
                    ui.label(rows.iter().filter(|r| r.mean >= 35.0).count().to_string());
                    ui.end_row();
                }
            });
    }

    fn stream_tab(&mut self, ui: &mut egui::Ui) {
        ui.horizontal(|ui| {
            ui.checkbox(&mut self.paused, "Pause");
            if ui.button("Clear").clicked() {
                lock(&self.shared).stream.clear();
                self.stream.clear();
            }
        });
        if !self.paused {
            self.stream = lock(&self.shared).stream.iter().cloned().collect();
        }
        let h = ui.text_style_height(&egui::TextStyle::Monospace);
        egui::ScrollArea::both()
            .stick_to_bottom(true)
            .auto_shrink(false)
            .show_rows(ui, h, self.stream.len(), |ui, range| {
                for line in &self.stream[range] {
                    ui.add(egui::Label::new(RichText::new(line).monospace()).extend());
                }
            });
    }
}

impl eframe::App for App {
    fn ui(&mut self, ui: &mut egui::Ui, _frame: &mut eframe::Frame) {
        ui.ctx().request_repaint_after(Duration::from_millis(200));
        lock(&self.shared).state.expire();
        self.update_quality();
        // configuring a u-blox receiver has ended: its baud rate; UBX files on if it succeeded
        if let Some((baud, ok)) = lock(&self.scan).configured.take() {
            if let Some(b) = baud {
                self.cfg.baud = b;
            }
            if ok {
                self.cfg.save_ubx = true;
                self.cfg.save_ubx_text = true;
            }
        }

        // Ctrl+Tab: next tab
        if ui
            .ctx()
            .input_mut(|i| i.consume_key(egui::Modifiers::CTRL, egui::Key::Tab))
        {
            let i = TABS.iter().position(|(t, _)| *t == self.tab).unwrap_or(0);
            self.tab = TABS[(i + 1) % TABS.len()].0;
        }

        egui::Panel::top("tabs").show(ui, |ui| {
            ui.horizontal(|ui| {
                for (t, name) in TABS {
                    ui.selectable_value(&mut self.tab, t, RichText::new(name).size(15.0));
                }
            });
        });
        egui::Panel::bottom("status").show(ui, |ui| {
            let sh = lock(&self.shared);
            let red = Color32::RED;
            ui.horizontal(|ui| {
                if sh.reconnecting {
                    ui.label(RichText::new(format!("◌ Reconnecting {}", self.cfg.port)).color(red));
                } else if sh.running {
                    ui.label(
                        RichText::new(format!("● Connected {}", self.cfg.port))
                            .color(Color32::from_rgb(40, 160, 60)),
                    );
                } else {
                    ui.label("○ Disconnected");
                }
                ui.separator();
                ui.label(format!(
                    "NMEA: {}   UBX: {}   Bad: {}",
                    sh.nmea_count, sh.ubx_count, sh.bad_count
                ));
                // a port that is open but silent
                let silent = sh.last_rx.map(|t| t.elapsed().as_secs());
                if sh.running && !sh.reconnecting && silent.is_none_or(|s| s >= 5) {
                    ui.separator();
                    let text = match silent {
                        Some(s) => format!("No data for {s} s"),
                        None => "No data received".to_string(),
                    };
                    ui.label(RichText::new(text).color(red));
                }
                for e in [&sh.error, &sh.file_error, &self.notice]
                    .into_iter()
                    .flatten()
                {
                    ui.separator();
                    ui.label(RichText::new(e).color(red));
                }
            });
        });
        egui::CentralPanel::default().show(ui, |ui| match self.tab {
            Tab::Connection => self.connection_tab(ui),
            Tab::Information => self.info_tab(ui),
            Tab::SkyMap => self.sky_tab(ui),
            Tab::Quality => self.quality_tab(ui),
            Tab::Map => {
                let pos = {
                    let sh = lock(&self.shared);
                    sh.state.lat.zip(sh.state.lon)
                };
                self.map.ui(ui, pos, &self.cfg);
            }
            Tab::DateTime => self.datetime_tab(ui),
        });
    }
}

impl Drop for App {
    fn drop(&mut self) {
        self.disconnect();
        let _ = self.cfg.save(); // the window is closing: nowhere to show an error
    }
}

fn main() -> eframe::Result {
    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_inner_size([900.0, 620.0])
            .with_title("gnssCaptureR"),
        ..Default::default()
    };
    let (cfg, bad) = Settings::load();
    let app = App {
        notice: (!bad.is_empty()).then(|| {
            format!(
                "gnssCaptureR.ini: cannot read {} – default used",
                bad.join(", ")
            )
        }),
        cfg,
        tab: Tab::Connection,
        shared: Arc::new(Mutex::new(Shared::default())),
        stop: Arc::new(AtomicBool::new(false)),
        thread: None,
        scan: Arc::new(Mutex::new(PortScan {
            ports: list_ports(),
            ..Default::default()
        })),
        paused: false,
        stream: Vec::new(),
        quality: Vec::new(),
        quality_next: Instant::now(),
        quality_time: String::new(),
        map: map::MapView::default(),
        clock_diff: String::new(),
        clock_second: 0,
    };
    eframe::run_native("gnssCaptureR", options, Box::new(|_cc| Ok(Box::new(app))))
}
