# UBX messages and commands (gnssCaptureR)

This document describes the u-blox UBX messages that gnssCaptureR logs and decodes, how the
program finds and configures a u-blox receiver, and the UBX commands it sends, so that you can
also send them yourself (e.g. with u-center or any terminal that can send hex bytes).

All byte sequences below are complete frames with correct checksums.

## 1. UBX frame format

```
B5 62 | class | id | length (2 bytes, little-endian) | payload | CK_A CK_B
```

- Numbers in the payload are little-endian.
- The checksum is calculated over class, id, length and payload:
  `CK_A = CK_A + byte; CK_B = CK_B + CK_A` (8-bit, starting from 0).
- Every configuration command (class `06`) is answered with **ACK-ACK** (`05 01`) or
  **ACK-NAK** (`05 00`); the payload of the answer is the class and id of the command.
  NAK means "not accepted", usually because the receiver does not support the message.

## 2. What gnssCaptureR does

**Refresh** (Serial Port & Capture tab) checks every serial port, all ports in parallel:

1. The port is opened at 9600, 115200, 38400, 57600, 230400, 460800, 921600, 19200 and 4800 baud
   in turn.
2. At each baud rate the version is polled (UBX-MON-VER) and the incoming data are checked.
3. If MON-VER is answered, the port is shown as a u-blox receiver: `COM4 (U-Blox 6)`.
   The model comes from MON-VER: newer receivers (u-blox M8 and later) report the module name
   (`MOD=NEO-M8N` → `U-Blox NEO-M8N`); u-blox 5/6/7 report only the hardware generation,
   so they are shown as `U-Blox 5`, `U-Blox 6`, `U-Blox 7`.
4. If valid NMEA data but no MON-VER answer are received, the port is an NMEA receiver of
   another make; its baud rate is remembered.

A port without a device is checked at all baud rates, which takes about 7 seconds. The program
writes the MON-VER poll (8 bytes) to every port it checks.

**Choosing a u-blox receiver** from the list:

1. The UBX messages for its generation (section 3) are switched on, at rate 1 (every
   navigation solution), on the port the command arrives on. NMEA output stays on.
2. The settings of the port the receiver is connected by are read (CFG-PRT poll) and only
   three things are changed: the baud rate becomes **115200**, UBX is added to the input
   protocols and UBX + NMEA to the output protocols. Everything else (data format, RTCM input)
   stays as it was. A USB port of the receiver has no baud rate, so only the protocols are set.
   If the receiver does not answer the poll, the usual UART1 settings are sent
   (8N1, input UBX+NMEA+RTCM, output UBX+NMEA).
3. The program switches to 115200 baud itself and saves the configuration in the receiver.
   The status text says where it went: `saved in the receiver` (u-blox 5 to M8: battery-backed
   RAM, flash or EEPROM, whatever the receiver has), `saved to flash`, or
   `saved to battery-backed RAM only` (F9/M10 without flash: kept only while the backup
   battery lasts), or `not saved`.
4. The receiver is polled again at the new baud rate to confirm it works.
5. On success the Baud rate box is set to 115200 and the UBX file checkboxes are ticked.
   The status text shows the result, e.g.
   `COM4: U-Blox 6 set to 115200 baud, 7 of 9 UBX messages on, saved in the receiver`
   ("7 of 9": two messages were refused with NAK, which is normal for messages the
   module does not support).
6. If the receiver does not answer after the change, the port is checked again at all baud
   rates. The Baud rate box then shows the rate at which the receiver really answers, and the
   UBX file checkboxes are left as they were.

**Choosing another receiver**: the detected baud rate is put into the Baud rate box and the
UBX file checkboxes are unticked.

Ports are only checked and configured on **Refresh** and when you choose a port from the list.
**Connect** is disabled while this is running.

## 3. UBX messages switched on, by receiver generation

| Generation (MON-VER hwVersion) | Configured with | Messages switched on |
|---|---|---|
| u-blox 5, 6 (`00040005`, `00040007`), e.g. NEO-6M | CFG-MSG, CFG-PRT, CFG-CFG | NAV-POSLLH, NAV-STATUS, NAV-DOP, NAV-VELNED, NAV-TIMEUTC, NAV-CLOCK, NAV-SVINFO, RXM-RAW, RXM-SFRB |
| u-blox 7 (`00070000`) | CFG-MSG, CFG-PRT, CFG-CFG | NAV-PVT, NAV-DOP, NAV-TIMEUTC, NAV-CLOCK, NAV-SVINFO |
| u-blox M8 (`00080000`) | CFG-MSG, CFG-PRT, CFG-CFG | NAV-PVT, NAV-DOP, NAV-TIMEUTC, NAV-CLOCK, NAV-SAT, RXM-RAWX, RXM-SFRBX |
| u-blox F9 (`00190000`), M10 (`000A0000`), protocol 27+ | CFG-VALSET | NAV-PVT, NAV-DOP, NAV-TIMEUTC, NAV-CLOCK, NAV-SAT, RXM-RAWX, RXM-SFRBX |

Raw measurement messages (RXM-RAW/SFRB on u-blox 6, RXM-RAWX/SFRBX on M8 and later) exist
only on some modules (e.g. NEO-6T, NEO-M8T, ZED-F9P); other modules answer NAK, which is harmless.

## 4. Message reference

All UBX messages are written unchanged to the `.ubx` file. In the `.ubx.txt` file each message
is written as one entry with the reception time; the table shows what is decoded.

| Message | Class Id | Receivers | Contents | In `.ubx.txt` |
|---|---|---|---|---|
| NAV-POSLLH | 01 02 | all | Latitude, longitude, height above ellipsoid and MSL, horizontal/vertical accuracy | decoded |
| NAV-STATUS | 01 03 | all | Fix type, fix flags, time to first fix, time since start | decoded |
| NAV-DOP | 01 04 | all | GDOP, PDOP, TDOP, VDOP, HDOP, NDOP, EDOP | decoded |
| NAV-PVT | 01 07 | u-blox 7 and later | Everything in one message: UTC date/time, fix type, satellites used, position, height, accuracies, velocity N/E/D, ground speed (m/s, km/h, knots), heading, PDOP | decoded |
| NAV-VELNED | 01 12 | all | Velocity north/east/down, 3D and ground speed, heading | decoded |
| NAV-TIMEUTC | 01 21 | all | UTC date and time, nanoseconds, time accuracy, validity flags | decoded |
| NAV-CLOCK | 01 22 | all | Receiver clock bias (ns) and drift (ns/s), accuracies | decoded |
| NAV-SVINFO | 01 30 | u-blox 5 to M8 | Satellites: channel, number, used, quality, C/N0, elevation, azimuth | hex payload |
| NAV-SAT | 01 35 | u-blox M8 and later | Satellites: system, number, C/N0, elevation, azimuth, pseudorange residual, used flag | decoded, one line per satellite |
| RXM-RAW | 02 10 | u-blox 6 (some modules) | Raw measurements: carrier phase, pseudorange, Doppler, C/N0 per satellite | hex payload |
| RXM-SFRB | 02 11 | u-blox 6 (some modules) | GPS navigation subframes | hex payload |
| RXM-SFRBX | 02 13 | M8 and later (some modules) | Navigation data subframes, all systems | decoded (words in hex) |
| RXM-RAWX | 02 15 | M8 and later (some modules) | Raw measurements, all systems and signals | decoded, one line per measurement |
| INF-* | 04 xx | all | Text messages from the receiver | decoded (text) |
| ACK-ACK / ACK-NAK | 05 01 / 05 00 | all | Answer to a CFG command | decoded |
| MON-VER | 0A 04 | all | Software, hardware and extension versions | decoded |

NAV-PVT, NAV-DOP and NAV-SAT also fill the Information, Satellite map, Quality and Map tabs.

## 5. Commands

### 5.1 Poll the receiver version (UBX-MON-VER)

```
B5 62 0A 04 00 00 0E 34
```

### 5.2 Switch a message on or off (UBX-CFG-MSG), u-blox 5 to M8

Payload: class, id, rate. Rate 1 = every navigation solution, 0 = off. This short form sets the
rate on the port the command is received on.

| Message | On (rate 1) | Off (rate 0) |
|---|---|---|
| NAV-POSLLH | `B5 62 06 01 03 00 01 02 01 0E 47` | `B5 62 06 01 03 00 01 02 00 0D 46` |
| NAV-STATUS | `B5 62 06 01 03 00 01 03 01 0F 49` | `B5 62 06 01 03 00 01 03 00 0E 48` |
| NAV-DOP | `B5 62 06 01 03 00 01 04 01 10 4B` | `B5 62 06 01 03 00 01 04 00 0F 4A` |
| NAV-PVT | `B5 62 06 01 03 00 01 07 01 13 51` | `B5 62 06 01 03 00 01 07 00 12 50` |
| NAV-VELNED | `B5 62 06 01 03 00 01 12 01 1E 67` | `B5 62 06 01 03 00 01 12 00 1D 66` |
| NAV-TIMEUTC | `B5 62 06 01 03 00 01 21 01 2D 85` | `B5 62 06 01 03 00 01 21 00 2C 84` |
| NAV-CLOCK | `B5 62 06 01 03 00 01 22 01 2E 87` | `B5 62 06 01 03 00 01 22 00 2D 86` |
| NAV-SVINFO | `B5 62 06 01 03 00 01 30 01 3C A3` | `B5 62 06 01 03 00 01 30 00 3B A2` |
| NAV-SAT | `B5 62 06 01 03 00 01 35 01 41 AD` | `B5 62 06 01 03 00 01 35 00 40 AC` |
| RXM-RAW | `B5 62 06 01 03 00 02 10 01 1D 66` | `B5 62 06 01 03 00 02 10 00 1C 65` |
| RXM-SFRB | `B5 62 06 01 03 00 02 11 01 1E 68` | `B5 62 06 01 03 00 02 11 00 1D 67` |
| RXM-SFRBX | `B5 62 06 01 03 00 02 13 01 20 6C` | `B5 62 06 01 03 00 02 13 00 1F 6B` |
| RXM-RAWX | `B5 62 06 01 03 00 02 15 01 22 70` | `B5 62 06 01 03 00 02 15 00 21 6F` |

### 5.3 Change the baud rate (UBX-CFG-PRT), u-blox 5 to M8

Payload (20 bytes): portID `01` (UART1), reserved, txReady `00 00`, mode `D0 08 00 00` (8N1),
baud rate (4 bytes), inProtoMask `07 00` (UBX+NMEA+RTCM), outProtoMask `03 00` (UBX+NMEA),
flags `00 00`, reserved `00 00`.

| Baud rate | Command |
|---|---|
| 9600 | `B5 62 06 00 14 00 01 00 00 00 D0 08 00 00 80 25 00 00 07 00 03 00 00 00 00 00 A2 B5` |
| 19200 | `B5 62 06 00 14 00 01 00 00 00 D0 08 00 00 00 4B 00 00 07 00 03 00 00 00 00 00 48 57` |
| 38400 | `B5 62 06 00 14 00 01 00 00 00 D0 08 00 00 00 96 00 00 07 00 03 00 00 00 00 00 93 90` |
| 57600 | `B5 62 06 00 14 00 01 00 00 00 D0 08 00 00 00 E1 00 00 07 00 03 00 00 00 00 00 DE C9` |
| **115200** (used by gnssCaptureR) | `B5 62 06 00 14 00 01 00 00 00 D0 08 00 00 00 C2 01 00 07 00 03 00 00 00 00 00 C0 7E` |
| 230400 | `B5 62 06 00 14 00 01 00 00 00 D0 08 00 00 00 84 03 00 07 00 03 00 00 00 00 00 84 E8` |
| 460800 | `B5 62 06 00 14 00 01 00 00 00 D0 08 00 00 00 08 07 00 07 00 03 00 00 00 00 00 0C BC` |
| 921600 | `B5 62 06 00 14 00 01 00 00 00 D0 08 00 00 00 10 0E 00 07 00 03 00 00 00 00 00 1B 5A` |

The receiver switches to the new baud rate at once, so its ACK is usually lost. Switch the PC
port to the new baud rate and poll MON-VER (5.1) to check. Rates above 115200 are not supported
by all receivers (u-blox 6: up to 115200 is safe).

To read the current port settings: `B5 62 06 00 01 00 01 08 22` (UART1), or
`B5 62 06 00 00 00 06 18` (the port the command arrives on; this is what gnssCaptureR sends
before it changes the baud rate). The answer is a CFG-PRT message with the 20-byte payload
described above. The commands in the table set the input protocols to UBX+NMEA+RTCM; to keep
other input protocols (e.g. RTCM3 on u-blox M8, input mask `27 00`), read the settings first
and send them back with only the baud rate changed.

Alternative NMEA command with the same effect (u-blox 5 to M8):
`$PUBX,41,1,0007,0003,115200,0*18` (followed by CR LF).

### 5.4 Save / restore the configuration (UBX-CFG-CFG), u-blox 5 to M8

Payload: clearMask (4), saveMask (4), loadMask (4), deviceMask (1). deviceMask `17` = BBR,
flash, EEPROM and SPI flash (whatever the receiver has).

| Action | Command |
|---|---|
| Save port and message settings (sent by gnssCaptureR) | `B5 62 06 09 0D 00 00 00 00 00 03 00 00 00 00 00 00 00 17 36 EB` |
| Save the whole current configuration | `B5 62 06 09 0D 00 00 00 00 00 FF FF 00 00 00 00 00 00 17 31 BF` |
| **Restore factory defaults** (9600 baud, NMEA only) | `B5 62 06 09 0D 00 FF FF 00 00 00 00 00 00 FF FF 00 00 17 2F AE` |

Without a backup battery or flash, settings are lost when the receiver is switched off;
choose the receiver again in gnssCaptureR after Refresh.

### 5.5 u-blox F9 and M10 (UBX-CFG-VALSET)

These receivers use configuration keys. M10 no longer accepts CFG-PRT, CFG-MSG and CFG-CFG.
Payload: version `00`, layers, reserved `00 00`, then key ID (4 bytes) and value
(1, 2 or 4 bytes, given by the key). Layers: `01` RAM, `02` BBR, `04` flash;
`07` = all three (receivers without flash answer NAK, then use `03` = RAM + BBR).
gnssCaptureR sends one key per command, so an unsupported key does not block the others.

| Key | ID | Value |
|---|---|---|
| CFG-UART1-BAUDRATE | `0x40520001` | baud rate (4 bytes) |
| CFG-UART1OUTPROT-UBX / USBOUTPROT-UBX | `0x10740001` / `0x10780001` | 1 = UBX output on |
| CFG-MSGOUT-UBX_NAV_PVT UART1 / USB | `0x20910007` / `0x20910009` | rate |
| CFG-MSGOUT-UBX_NAV_DOP UART1 / USB | `0x20910039` / `0x2091003B` | rate |
| CFG-MSGOUT-UBX_NAV_TIMEUTC UART1 / USB | `0x2091005C` / `0x2091005E` | rate |
| CFG-MSGOUT-UBX_NAV_CLOCK UART1 / USB | `0x20910066` / `0x20910068` | rate |
| CFG-MSGOUT-UBX_NAV_SAT UART1 / USB | `0x20910016` / `0x20910018` | rate |
| CFG-MSGOUT-UBX_RXM_RAWX UART1 / USB | `0x209102A5` / `0x209102A7` | rate |
| CFG-MSGOUT-UBX_RXM_SFRBX UART1 / USB | `0x20910232` / `0x20910234` | rate |

Examples:

| Action | Layers RAM+BBR+flash (`07`) | Layers RAM+BBR (`03`) |
|---|---|---|
| NAV-PVT on, UART1 | `B5 62 06 8A 09 00 00 07 00 00 07 00 91 20 01 59 78` | `B5 62 06 8A 09 00 00 03 00 00 07 00 91 20 01 55 58` |
| NAV-PVT on, USB | `B5 62 06 8A 09 00 00 07 00 00 09 00 91 20 01 5B 82` | `B5 62 06 8A 09 00 00 03 00 00 09 00 91 20 01 57 62` |
| NAV-SAT on, UART1 | `B5 62 06 8A 09 00 00 07 00 00 16 00 91 20 01 68 C3` | `B5 62 06 8A 09 00 00 03 00 00 16 00 91 20 01 64 A3` |
| UART1 115200 baud | `B5 62 06 8A 0C 00 00 07 00 00 01 00 52 40 00 C2 01 00 F9 E7` | `B5 62 06 8A 0C 00 00 03 00 00 01 00 52 40 00 C2 01 00 F5 BB` |

## 6. Sending commands yourself

- **u-center** (u-blox, Windows): View → Messages View → UBX → CFG → choose the message,
  set the fields and press *Send*.
- **Any terminal program that can send hex bytes** (e.g. RealTerm "Send Numbers"): paste the
  byte sequence from this document. Watch for `B5 62 05 01 …` (ACK) or `B5 62 05 00 …` (NAK).
- Close gnssCaptureR (or press Disconnect) first: only one program can use the port.

Reference: u-blox receiver description / interface description for your receiver generation
(u-blox 6: GPS.G6-SW-10018; u-blox M8: UBX-13003221; ZED-F9P: UBX-18010854;
u-blox M10: UBX-20048810), available from u-blox.com.
