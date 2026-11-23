# gnssCaptureR

Logs NMEA and u-blox UBX messages from a GPS receiver on a serial (or virtual COM) port.
Runs on Windows and Linux.

## Build

Install Rust from https://rustup.rs, then in this folder:

```
cargo build --release
```

The program is `target/release/gnssCaptureR` (Linux) or `target\release\gnssCaptureR.exe` (Windows).
Run the tests with `cargo test`.

To install the program elsewhere, copy the executable together with the `Maps` folder.

Linux notes: the user must be allowed to open serial ports
(`sudo usermod -aG dialout $USER`, then log in again). No extra `-dev` packages are needed to build.

## Tabs

| Tab | Contents |
|---|---|
| Serial Port & Capture | Port, baud rate, data bits, parity, stop bits; capture directory, file prefix, write block size, which files to write; Connect / Disconnect. **Refresh** lists the ports and checks each for a u-blox receiver (shown as e.g. `COM4 (U-Blox 6)`). Choosing a u-blox receiver sets it to 115200 baud, 10 Hz solutions and the message rates below, saves this in the receiver, and sets the Baud rate box and ticks the UBX files; choosing another receiver sets its detected baud rate and unticks the UBX files. See `UBX.md`. Below the button: data stream (last 1000 NMEA sentences; Pause / Clear) |
| Information | UTC date/time, latitude, longitude, altitude, heading, speed in **knots, km/h and m/s**, GPS type (2D/3D + fix quality), satellites used / in view, HDOP, VDOP, PDOP |
| Satellite map | Sky plot (filled = used in fix, colour = constellation) and a satellite table |
| Quality | Signal quality (C/N0, dBHz) of every satellite: bar = mean over the update period, whisker = min..max, bar colour red < 25 ≤ orange < 35 ≤ yellow-green < 45 ≤ green. Below: summary per constellation. Updated every `QualityPeriod` seconds (default 10, can also be changed on the tab) |
| Map | World map with the current position (red point). Mouse wheel / + / − to zoom, drag to move, *World*, *Position*, *Follow position*. The map scale changes with the zoom: `Maps/50m/ne_50m_land.shp` for wide views, `Maps/10m/ne_10m_land.shp` when the visible width is below `DetailedBelowDegrees` (default 30°). The file in use is shown in the top-left corner |
| Date and time | Three panels separated by lines, updated every second, big font (size `TimeFontSize`): top – UTC date and time from the receiver (NMEA, or UBX NAV-PVT); centre – PC date and time (UTC); bottom – difference PC − receiver in seconds, both times taken when the last time message (GGA, RMC, NAV-PVT) arrived |

The Information, Satellite map and Quality tabs are filled from NMEA (GGA, RMC, VTG, GSA, GSV)
and/or UBX (NAV-PVT, NAV-DOP, NAV-SAT), whichever the receiver sends.

Ctrl+Tab switches to the next tab.

The status bar shows the connection state, message counters and problems in red:
`Reconnecting` after the port was lost, `No data for N s` when the port is open but silent,
file-writing and settings-file errors.

## The u-blox receiver

Choosing a u-blox receiver from the port list sets it up for fast recording and saves the
settings in the receiver, so they survive a power cycle:

| Setting | Value |
|---|---|
| UART baud rate | 115200 |
| Measurement period | 100 ms |
| Navigation rate | 1 (one solution per measurement) |
| UBX-NAV-PVT | 10 Hz |
| NMEA GGA, RMC | 10 Hz |
| NMEA GSA, GSV | 1 Hz |

Everything else the program logs (NAV-DOP, NAV-TIMEUTC, NAV-CLOCK, NAV-SAT, the raw-data
messages, NMEA GLL and VTG) is set to 1 Hz as well.

Older receivers cannot do 10 Hz: a u-blox 6 (NEO-6M) is limited to 5 Hz and a u-blox 5 to
4 Hz, so they are set to their own maximum and the messages above follow it (5 Hz and 1 Hz on
a NEO-6M). u-blox 7 and later reach 10 Hz. The status text next to *Refresh* reports the
period that was set and the rate the receiver really delivers, measured over three seconds,
for example `200 ms solutions (4.9 Hz)`.

At 10 Hz the files grow by roughly 20-40 MB per hour with all four of them switched on
(about 0.5 GB a day); the decoded UBX text file is the largest. Switching off the decoded
files, or lowering the rate, cuts this.

## Connection

- If the port is lost while connected (cable, USB), the program keeps trying to reopen it every
  2 seconds and continues recording by itself. A port that delivers nothing for 10 seconds is
  closed and reopened as well. Only the first opening, on Connect, must succeed.
- Only sentences with a correct checksum update the tabs; every sentence is still written to
  the `.nmea` file.

## Capture files

One set of files per UTC hour (the computer clock is assumed to be UTC):

```
<Directory>/YYYY/MM/DD/<prefix>_YYYYMMDD_HH00.nmea       NMEA sentences as received
<Directory>/YYYY/MM/DD/<prefix>_YYYYMMDD_HH00.nmea.txt   decoded NMEA (optional)
<Directory>/YYYY/MM/DD/<prefix>_YYYYMMDD_HH00.ubx        raw UBX binary frames
<Directory>/YYYY/MM/DD/<prefix>_YYYYMMDD_HH00.ubx.txt    decoded UBX
<Directory>/YYYY/MM/DD/<prefix>_YYYYMMDD_HH00.quality.txt  signal quality every second
```

- On Connect, writing starts at once into the file of the current hour; a new file is started
  exactly at mm:ss = 00:00 of every new hour.
- Data are always written while connected; the checkboxes choose which files.
- Messages are written in blocks (10, 20, 50 or 100 messages). Before every block write
  the directory tree is checked and created if missing. The last partial block is written at the
  hour change and on Disconnect / exit.
- A block that cannot be written (disk away, file locked) is kept in memory and written again
  every second until it succeeds, into the file of the hour it belongs to. The status bar shows
  how much is waiting. Above 16 MB the oldest data are dropped, and the amount lost is shown.
- Decoded files have one entry per message, prefixed with the UTC reception time.
- Decoded NMEA: GGA, RMC, VTG, GSA, GSV (one line per satellite), GLL, GNS, ZDA, GST, TXT;
  speeds in knots, km/h and m/s; checksum errors are marked. Other sentences are written as fields.
- Decoded UBX: NAV-PVT, POSLLH, STATUS, DOP, VELNED, TIMEUTC, CLOCK, SAT,
  RXM-RAWX, RXM-SFRBX, INF-*, ACK/NAK, MON-VER. Other messages are written as name + hex payload.

Example of `.nmea.txt`:

```
2026-11-12T19:26:25.923Z GPRMC [GPS] time=19:26:13.00 date=121126 status=valid lat=50.4494000N lon=30.5243167E speed=10.800 kn = 20.002 km/h = 5.556 m/s course=87.5 magVar= mode=A
2026-11-12T19:26:25.923Z GPGSV [GPS] msg 1/1 inView=03
    PRN= 05 elev=62 az=140 snr=45
```

### Signal quality file (.quality.txt)

One line per second (for every second in which GSV or UBX NAV-SAT data were received).
A line covers one second of the **receiver's** time and is stamped with it, so the satellites of
one receiver epoch are always on one line. While the receiver reports no time (before the first
fix) the PC clock is used. Values are averages over that second; for a receiver that reports
several signals per satellite, over all its signals:

```
2026-11-13T18:11:09.000Z inView=6 used=4 meanCN0=36.8 meanCN0used=37.4 G05:U:45.4 G12:U:29.6 G24:U:40.0 R07:U:34.9 E11:V:38.9 C21:V:32.1
```

- `inView` – satellites reported in that second; `used` – of them, used in the fix
- `meanCN0` – mean C/N0 (dBHz) of all satellites with a signal; `meanCN0used` – of the used ones
- then one item per satellite: `<system letter><number>:<U = used | V = in view only>:<C/N0>`,
  `-` when no C/N0 was reported. Letters: G GPS, R GLONASS, E Galileo, C BeiDou, J QZSS,
  I NavIC, S SBAS. GLONASS numbers are slot numbers (1–32), SBAS numbers are PRNs (120–158).

## Settings

Stored in `gnssCaptureR.ini` next to the executable, saved on Connect and on exit.
A comment may follow a value after a space and `;`. Windows paths are written as they are
(`D:\data\gps`). A value that cannot be read is replaced by its default, and the status bar
names it at start; a failure to save the file is shown there too.

```ini
[Serial]
Port=COM3
BaudRate=115200
DataBits=8
Parity=None              ; None, Odd, Even
StopBits=1

[Capture]
Directory=data
Prefix=gps
BlockSize=20
SaveNMEA=true
SaveNMEADecoded=false
SaveUBX=true
SaveUBXDecoded=true
SaveQuality=true

[Display]
QualityPeriod=10         ; seconds between Quality tab updates
TimeFontSize=96          ; font size of the time on the Date and time tab

[Map]
Directory=Maps                  ; relative: next to the program, else the current folder
File=50m/ne_50m_land.shp        ; map for wide views
DetailedFile=10m/ne_10m_land.shp ; map when the visible width is below DetailedBelowDegrees
DetailedBelowDegrees=30
```

## Map files

The map is read directly from ESRI shapefiles (`.shp`, WGS-84 degrees) in the `Maps` folder:

```
Maps/50m/ne_50m_land.shp   Natural Earth 1:50m land, used for wide views
Maps/10m/ne_10m_land.shp   Natural Earth 1:10m land, used when zoomed in
```

Natural Earth data are public domain (https://www.naturalearthdata.com). Any polygon or polyline
shapefile can be used instead: put it into `Maps` and set `File` / `DetailedFile` in
`gnssCaptureR.ini` (paths relative to `Maps`), e.g. `10m/ne_10m_admin_0_countries.shp` to see
country borders when zoomed in. Only the `.shp` file is needed; the other files of a shapefile
set (.shx, .dbf, .prj, ...) are not used. Each file is loaded once, when first needed.

## Source files

| File | Purpose |
|---|---|
| `src/main.rs` | Window and tabs (incl. Quality) |
| `src/reader.rs` | Serial-port thread; splits the byte stream into NMEA and UBX messages |
| `src/nmea.rs` | Receiver state, NMEA parsing and decoding to text (speed is kept in knots and converted for display) |
| `src/ubx.rs` | UBX checksum, decoding to text, state update |
| `src/ublox.rs` | u-blox receiver detection (MON-VER) and configuration (UBX messages, baud rate) |
| `UBX.md` | UBX messages and the commands to enable them and change the baud rate |
| `src/logger.rs` | Hourly files with block writing |
| `src/map.rs` | Shapefile loading and the Map tab |
| `src/config.rs` | INI settings |
