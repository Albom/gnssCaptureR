//! World map: loading of ESRI shapefiles (.shp) and the zoomable map view.
//!
//! Any polygon or polyline shapefile in WGS-84 degrees can be used, e.g. the
//! Natural Earth "ne_50m_land.shp" / "ne_10m_land.shp". Only the .shp file is read.
//! Shapefile layout: 100-byte header, then records of
//!   record header (2 x i32 big-endian: number, length in 16-bit words),
//!   shape type i32, bbox 4 x f64, numParts i32, numPoints i32,
//!   parts numParts x i32, points numPoints x (f64 x, f64 y)  (little-endian).

use crate::config::{Settings, exe_dir};
use eframe::egui::{self, Align2, Color32, FontId, Pos2, Rect, Sense, Shape, Stroke, Vec2};
use std::collections::HashMap;
use std::path::{Path, PathBuf};
use std::sync::{Arc, OnceLock};
use std::time::{Duration, Instant};

struct Line {
    bbox: [f32; 4], // min lon, min lat, max lon, max lat
    pts: Vec<[f32; 2]>,
}

type MapData = Result<Vec<Line>, String>;

/// Adds a polygon ring / polyline part, split where it runs along the map frame
/// (the ±180° meridian or the South Pole edge of Antarctica), which is not a coastline.
fn add_part(lines: &mut Vec<Line>, part: &[[f32; 2]]) {
    let frame = |a: [f32; 2], b: [f32; 2]| {
        (a[0].abs() > 179.99 && b[0].abs() > 179.99) || (a[1] < -89.9 && b[1] < -89.9)
    };
    let mut start = 0;
    for i in 1..=part.len() {
        if i == part.len() || frame(part[i - 1], part[i]) {
            // long lines (e.g. the Eurasia coast) are cut into short pieces, so only
            // the pieces inside the view are drawn
            for a in (start..i.saturating_sub(1)).step_by(CHUNK) {
                let pts = part[a..(a + CHUNK + 1).min(i)].to_vec();
                let mut bbox = [f32::MAX, f32::MAX, f32::MIN, f32::MIN];
                for p in &pts {
                    bbox = [
                        bbox[0].min(p[0]),
                        bbox[1].min(p[1]),
                        bbox[2].max(p[0]),
                        bbox[3].max(p[1]),
                    ];
                }
                lines.push(Line { bbox, pts });
            }
            start = i;
        }
    }
}

const CHUNK: usize = 256; // points per drawn piece

fn load(path: &Path) -> MapData {
    let b = std::fs::read(path).map_err(|e| format!("{}: {e}", path.display()))?;
    let bad = || format!("{}: not a valid shapefile", path.display());
    let i4 = |o: usize| {
        b.get(o..o + 4)
            .map(|s| i32::from_le_bytes(s.try_into().unwrap()) as usize)
    };
    let be4 = |o: usize| {
        b.get(o..o + 4)
            .map(|s| i32::from_be_bytes(s.try_into().unwrap()) as usize)
    };
    let f8 = |o: usize| f64::from_le_bytes(b[o..o + 8].try_into().unwrap()) as f32;
    if be4(0) != Some(9994) {
        return Err(bad());
    }
    let mut lines = Vec::new();
    let mut o = 100;
    while o + 8 <= b.len() {
        let len = be4(o + 4).ok_or_else(bad)? * 2; // content length in bytes
        let rec = o + 8;
        o = rec + len;
        // 3 / 5 = polyline / polygon (13/15, 23/25: same with Z / M values)
        if !matches!(i4(rec), Some(3 | 5 | 13 | 15 | 23 | 25)) {
            continue;
        }
        let (np, n) = (i4(rec + 36).ok_or_else(bad)?, i4(rec + 40).ok_or_else(bad)?);
        let pts_at = rec + 44 + 4 * np;
        if pts_at + 16 * n > b.len() {
            return Err(bad());
        }
        let pts: Vec<[f32; 2]> = (0..n)
            .map(|k| [f8(pts_at + 16 * k), f8(pts_at + 16 * k + 8)])
            .collect();
        let mut starts: Vec<usize> = (0..np).filter_map(|k| i4(rec + 44 + 4 * k)).collect();
        starts.push(n);
        for w in starts.windows(2) {
            add_part(&mut lines, &pts[w[0].min(n)..w[1].min(n)]);
        }
    }
    Ok(lines)
}

/// A map file being loaded, or already loaded, by a background thread.
struct MapFile {
    started: Instant,
    data: Arc<OnceLock<MapData>>, // empty while loading
}

impl MapFile {
    fn start(path: PathBuf) -> MapFile {
        let data = Arc::new(OnceLock::new());
        let result = data.clone();
        std::thread::spawn(move || {
            let _ = result.set(load(&path));
        });
        MapFile {
            started: Instant::now(),
            data,
        }
    }
}

pub struct MapView {
    lon: f64, // centre of the view
    lat: f64,
    span: f64, // visible width in degrees of longitude
    follow: bool,
    maps: HashMap<String, MapFile>, // map files by name
}

impl Default for MapView {
    fn default() -> Self {
        MapView {
            lon: 0.0,
            lat: 20.0,
            span: 360.0,
            follow: true,
            maps: HashMap::new(),
        }
    }
}

impl MapView {
    pub fn ui(&mut self, ui: &mut egui::Ui, pos: Option<(f64, f64)>, cfg: &Settings) {
        // ---- toolbar
        ui.horizontal(|ui| {
            if ui.button(" + ").clicked() {
                self.span /= 2.0;
            }
            if ui.button(" − ").clicked() {
                self.span *= 2.0;
            }
            if ui.button("World").clicked() {
                (self.lon, self.lat, self.span, self.follow) = (0.0, 20.0, 360.0, false);
            }
            if ui.button("Position").clicked() {
                self.follow = true;
                self.span = self.span.min(5.0);
            }
            ui.checkbox(&mut self.follow, "Follow position");
            ui.label(egui::RichText::new("Drag to move, mouse wheel to zoom").weak());
        });

        if self.follow {
            if let Some((lat, lon)) = pos {
                (self.lat, self.lon) = (lat, lon);
            }
        }

        // ---- choose the map file by scale
        let dir = PathBuf::from(&cfg.maps_dir);
        // a relative folder is looked for next to the program, then in the current folder
        let dir = if dir.is_absolute() || !exe_dir().join(&dir).is_dir() {
            dir
        } else {
            exe_dir().join(dir)
        };
        let name = if self.span < cfg.detail_below_deg {
            &cfg.map_detailed
        } else {
            &cfg.map_file
        };
        // a file that could not be loaded is tried again every 5 seconds
        let failed = |m: &MapFile| matches!(m.data.get(), Some(Err(_)));
        if self
            .maps
            .get(name)
            .is_some_and(|m| failed(m) && m.started.elapsed() > Duration::from_secs(5))
        {
            self.maps.remove(name);
        }
        // files are loaded in the background, so the window does not freeze
        let data = self
            .maps
            .entry(name.clone())
            .or_insert_with(|| MapFile::start(dir.join(name)))
            .data
            .clone();
        // while the wanted file is loading, the other one is shown if it is ready
        let other = [&cfg.map_file, &cfg.map_detailed]
            .into_iter()
            .filter_map(|n| self.maps.get(n))
            .find_map(|m| {
                m.data
                    .get()
                    .is_some_and(|d| d.is_ok())
                    .then(|| m.data.clone())
            });

        // ---- view geometry (equirectangular, corrected for the centre latitude)
        let (resp, painter) = ui.allocate_painter(ui.available_size(), Sense::drag());
        let rect = resp.rect;
        if resp.dragged() {
            self.follow = false;
        }
        if resp.hovered() {
            let dy = ui.input(|i| i.smooth_scroll_delta.y) as f64;
            self.span *= (-dy * 0.003).exp();
        }
        self.span = self.span.clamp(0.005, 360.0);
        let kx = rect.width() as f64 / self.span; // pixels per degree of longitude
        // pixels per degree of latitude; the cos(lat) correction fades out when zoomed out to the world
        let lat_c = self.lat * (60.0 / self.span).min(1.0);
        let ky = kx / lat_c.to_radians().cos().max(0.2);
        let d = resp.drag_delta();
        // the map is continuous across the 180° meridian
        self.lon = (self.lon - d.x as f64 / kx + 540.0).rem_euclid(360.0) - 180.0;
        self.lat = (self.lat + d.y as f64 / ky).clamp(-85.0, 85.0);

        let c = rect.center();
        let to_screen = |lon: f64, lat: f64| {
            Pos2::new(
                c.x + ((lon - self.lon) * kx) as f32,
                c.y - ((lat - self.lat) * ky) as f32,
            )
        };
        let half_w = rect.width() as f64 / 2.0 / kx;
        let half_h = rect.height() as f64 / 2.0 / ky;
        let view = [
            self.lon - half_w,
            self.lat - half_h,
            self.lon + half_w,
            self.lat + half_h,
        ];

        painter.rect_filled(rect, 0.0, Color32::from_rgb(22, 42, 66));
        let painter = painter.with_clip_rect(rect);

        // ---- graticule
        let step = [
            30.0, 10.0, 5.0, 2.0, 1.0, 0.5, 0.2, 0.1, 0.05, 0.02, 0.01, 0.005, 0.002, 0.001,
        ]
        .into_iter()
        .find(|s| self.span / s >= 4.0)
        .unwrap_or(0.001);
        let grid = Stroke::new(1.0, Color32::from_rgb(45, 70, 100));
        let font = FontId::proportional(10.0);
        let label_col = Color32::from_rgb(140, 160, 190);
        let decimals = if step >= 1.0 {
            0
        } else {
            (-step.log10()).ceil() as usize
        };
        let mut g = (view[0] / step).floor() * step;
        while g <= view[2] {
            let x = to_screen(g, 0.0).x;
            painter.line_segment(
                [Pos2::new(x, rect.top()), Pos2::new(x, rect.bottom())],
                grid,
            );
            let label = (g + 540.0).rem_euclid(360.0) - 180.0; // beyond ±180° the numbers start again
            painter.text(
                Pos2::new(x + 2.0, rect.bottom() - 2.0),
                Align2::LEFT_BOTTOM,
                format!("{label:.decimals$}°"),
                font.clone(),
                label_col,
            );
            g += step;
        }
        let mut g = (view[1] / step).floor() * step;
        while g <= view[3] {
            let y = to_screen(0.0, g).y;
            painter.line_segment(
                [Pos2::new(rect.left(), y), Pos2::new(rect.right(), y)],
                grid,
            );
            painter.text(
                Pos2::new(rect.left() + 2.0, y - 1.0),
                Align2::LEFT_BOTTOM,
                format!("{g:.decimals$}°"),
                font.clone(),
                label_col,
            );
            g += step;
        }

        // ---- coastlines and borders (only lines inside the view are drawn)
        let coast = Stroke::new(1.0, Color32::from_rgb(200, 210, 170));
        // the world is drawn up to three times, 360° apart, so a view across the
        // 180° meridian is complete
        let worlds: Vec<f64> = [-360.0, 0.0, 360.0]
            .into_iter()
            .filter(|w| view[0] <= 180.0 + w && view[2] >= -180.0 + w)
            .collect();
        let draw = |lines: &Vec<Line>| {
            for &w in &worlds {
                for l in lines.iter().filter(|l| {
                    (l.bbox[2] as f64 + w) >= view[0]
                        && (l.bbox[0] as f64 + w) <= view[2]
                        && (l.bbox[3] as f64) >= view[1]
                        && (l.bbox[1] as f64) <= view[3]
                }) {
                    let pts = l
                        .pts
                        .iter()
                        .map(|p| to_screen(p[0] as f64 + w, p[1] as f64))
                        .collect();
                    painter.add(Shape::line(pts, coast));
                }
            }
        };
        let message = |text: &str, color: Color32| {
            painter.text(
                c,
                Align2::CENTER_CENTER,
                text,
                FontId::proportional(14.0),
                color,
            );
        };
        match data.get() {
            Some(Ok(lines)) => draw(lines),
            Some(Err(e)) => message(e, Color32::LIGHT_RED),
            None => {
                if let Some(Ok(lines)) = other.as_ref().and_then(|d| d.get()) {
                    draw(lines);
                }
                message(&format!("Loading {name}…"), Color32::WHITE);
            }
        }

        // ---- current position
        if let Some((lat, lon)) = pos {
            for &w in &worlds {
                let p = to_screen(lon + w, lat);
                painter.circle(p, 6.0, Color32::RED, Stroke::new(2.0, Color32::WHITE));
            }
        }

        // ---- info box
        let info = format!(
            "{name}   width {:.3}°   centre {:.5}, {:.5}{}",
            self.span,
            self.lat,
            self.lon,
            if pos.is_none() {
                "   (no position)"
            } else {
                ""
            }
        );
        let galley = painter.layout_no_wrap(info, FontId::monospace(12.0), Color32::WHITE);
        let at = rect.left_top() + Vec2::new(6.0, 6.0);
        let bg = Rect::from_min_size(at, galley.size()).expand(4.0);
        painter.rect_filled(bg, 3.0, Color32::from_black_alpha(170));
        painter.galley(at, galley, Color32::WHITE);
    }
}

#[cfg(test)]
mod tests {
    #[test]
    fn load_map() {
        let dir = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("Maps");
        let lines = super::load(&dir.join("50m/ne_50m_land.shp")).unwrap();
        assert!(lines.len() > 1000);
        assert!(
            lines
                .iter()
                .all(|l| l.pts.len() >= 2 && l.pts.len() <= super::CHUNK + 1)
        );
        let fine = super::load(&dir.join("10m/ne_10m_land.shp")).unwrap();
        assert!(fine.len() > lines.len());
        // nothing may run along the map frame
        assert!(
            lines
                .iter()
                .all(|l| l.pts.windows(2).all(|w| w[0][1] > -89.9 || w[1][1] > -89.9))
        );
        assert!(super::load(&dir.join("50m/ne_50m_land.dbf")).is_err());
        assert!(super::load(&dir.join("missing.shp")).is_err());
    }
}
