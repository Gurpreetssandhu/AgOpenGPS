# AgOpenGPS → Python Migration Review

## 1. Architecture Overview

AgOpenGPS is a precision agriculture guidance system with two main executables:

```
┌─────────────────────────────────────────────────────────┐
│                     AgOpenGPS                           │
│  (Guidance UI - WinForms/.NET Framework 4.8)            │
│                                                         │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐              │
│  │ CNMEA    │  │CGuidance │  │ CSection │              │
│  │(GPS Parse)│  │(Steering)│  │(Implement)│             │
│  └────┬─────┘  └────┬─────┘  └────┬─────┘              │
│       │              │              │                    │
│  ┌────┴──────────────┴──────────────┴────┐              │
│  │            FormGPS (Central Hub)       │              │
│  │   - OpenGL rendering (OpenTK)         │              │
│  │   - Timer-driven main loop            │              │
│  │   - Coordinate transforms             │              │
│  └────────────────┬──────────────────────┘              │
│                   │ UDP (loopback)                       │
├───────────────────┼─────────────────────────────────────┤
│                   │                                      │
│              ┌────┴─────┐                                │
│              │  AgIO    │                                │
│              │(Comm Hub)│                                │
│              └──┬───┬───┘                                │
│         Serial  │   │  UDP                               │
│    ┌────────────┘   └────────────┐                       │
│    │                             │                        │
│  ┌─┴──────────┐  ┌──────────────┴─┐                     │
│  │GPS Receiver│  │Steer/Section   │                     │
│  │(NMEA 0183) │  │Hardware (PGN)  │                     │
│  └────────────┘  └────────────────┘                     │
└─────────────────────────────────────────────────────────┘
```

## 2. Core Components to Migrate (Priority Order)

### Priority 1: Foundation Layer (~2,500 lines C#)

These are pure math/data with zero UI dependency — migrate first.

| C# File | Lines | Python Module | Description |
|----------|-------|---------------|-------------|
| `vec3.cs` | 172 | `core/vectors.py` | 2D/3D coordinate structs (easting, northing, heading) with operators |
| `LocalPlane.cs` | 58 | `core/coordinate_system.py` | WGS84 ↔ local plane (northing/easting) conversion |
| `GeoCoord.cs`, `Wgs84.cs` | ~60 | `core/geo_types.py` | Geographic coordinate value types |
| `CGLM.cs` | 427 | `core/geometry.py` | Math utilities: point-in-polygon, distance, angles, line intersection |
| `CDubins.cs` | 636 | `core/dubins.py` | Dubins path planning (6 path types: RSR, LSL, RSL, LSR, RLR, LRL) |

### Priority 2: GPS & Communication (~500 lines C#)

| C# File | Lines | Python Module | Description |
|----------|-------|---------------|-------------|
| `CNMEA.cs` | 55 | `comms/nmea_parser.py` | NMEA sentence parsing (GGA, VTG, RMC) — note: most parsing is in FormGPS partials |
| `CModuleComm.cs` | ~200 | `comms/module_comm.py` | PGN message protocol (hardware communication) |
| `CISOBUS.cs` | 180 | `comms/isobus.py` | ISO 11783 CAN bus protocol |
| AgIO UDP/Serial | ~800 | `comms/agio.py` | Serial port + UDP socket gateway |

### Priority 3: Guidance Algorithms (~5,000 lines C#)

The core value — these implement the actual precision agriculture logic.

| C# File | Lines | Python Module | Description |
|----------|-------|---------------|-------------|
| `CGuidance.cs` | 412 | `guidance/steering.py` | **Stanley + Pure Pursuit** steering algorithms |
| `CABLine.cs` | 568 | `guidance/ab_line.py` | AB Line guidance (straight line following) |
| `CABCurve.cs` | 1724 | `guidance/ab_curve.py` | AB Curve guidance (curved line following) |
| `CContour.cs` | 671 | `guidance/contour.py` | Contour guidance (follow previous path) |
| `CYouTurn.cs` | 2926 | `guidance/uturn.py` | Automatic U-turn generation at headlands |
| `CRecordedPath.cs` | 849 | `guidance/recorded_path.py` | Record and replay paths |
| `CVehicle.cs` | 386 | `guidance/vehicle.py` | Vehicle configuration + steering parameters |

### Priority 4: Field Management (~2,000 lines C#)

| C# File | Lines | Python Module | Description |
|----------|-------|---------------|-------------|
| `CBoundary.cs` + `CBoundaryList.cs` | ~300 | `field/boundary.py` | Field boundary polygons |
| `CSection.cs` | 76 | `field/sections.py` | Implement section on/off control |
| `CTool.cs` | 383 | `field/tool.py` | Implement/tool configuration (widths, offsets) |
| `CHead.cs` + `CHeadLine.cs` | ~300 | `field/headland.py` | Headland management |
| `CTrack.cs` | 351 | `field/track.py` | Guidance track storage |
| `CTram.cs` | 243 | `field/tram_lines.py` | Tramline tracking |
| `CFieldData.cs` | ~200 | `field/field_data.py` | Field metadata and storage |
| `CFence.cs` + `CFenceLine.cs` | ~400 | `field/fence.py` | Geo-fencing |

### Priority 5: Simulation & UI (~1,000 lines C#)

| C# File | Lines | Python Module | Description |
|----------|-------|---------------|-------------|
| `CSim.cs` | ~150 | `sim/simulator.py` | GPS simulator for testing without hardware |
| FormGPS partials | ~5000 | `ui/` (separate) | UI — use PyQt5/6, Tkinter, or web-based instead |

## 3. Key Algorithms to Understand

### 3.1 Stanley Steering Controller (`CGuidance.cs:42-100`)

```
Inputs:  heading error, cross-track distance, vehicle speed
Output:  steer angle (degrees)

steerAngle = headingError * headingGain
           + atan(crossTrackDistance * distanceGain / speed)
           + integral term (slow drift correction)
           + side hill compensation (IMU roll)
```

### 3.2 Pure Pursuit Steering (`CABLine.cs`, `CABCurve.cs`)

```
1. Find closest point on guidance line to vehicle
2. Calculate look-ahead distance based on speed
3. Find goal point on line at look-ahead distance
4. Calculate turning radius to reach goal point
5. Convert radius to steer angle: angle = atan(wheelbase / radius)
```

### 3.3 WGS84 → Local Plane Conversion (`LocalPlane.cs`)

```
metersPerDegreeLat = 111132.92 - 559.82*cos(2*lat) + 1.175*cos(4*lat) - 0.0023*cos(6*lat)
metersPerDegreeLon = 111412.84*cos(lat) - 93.5*cos(3*lat) + 0.118*cos(5*lat)

northing = (latitude - originLat) * metersPerDegreeLat
easting  = (longitude - originLon) * metersPerDegreeLon
```

### 3.4 Dubins Path Planning (`CDubins.cs`)

Computes shortest path between two oriented points using combinations of straight segments and circular arcs. Six path types: RSR, LSL, RSL, LSR, RLR, LRL. Used for U-turn generation.

### 3.5 Point-in-Polygon (`CGLM.cs:29-46`)

Ray casting algorithm used extensively for boundary checks, section control, and headland detection.

## 4. Suggested Python Project Structure

```
AgOpenGPS_py/
├── pyproject.toml              # Project config (use poetry or setuptools)
├── requirements.txt
├── README.md
│
├── agopenps/                   # Main package
│   ├── __init__.py
│   │
│   ├── core/                   # Foundation (Priority 1)
│   │   ├── __init__.py
│   │   ├── vectors.py          # vec2, vec3 equivalents (use dataclasses or numpy)
│   │   ├── geo_types.py        # Wgs84, GeoCoord
│   │   ├── coordinate_system.py # LocalPlane (WGS84 ↔ local)
│   │   ├── geometry.py         # Point-in-polygon, distances, angles
│   │   └── dubins.py           # Dubins path planner
│   │
│   ├── comms/                  # Communication (Priority 2)
│   │   ├── __init__.py
│   │   ├── nmea_parser.py      # NMEA 0183 parsing (GGA, VTG, RMC, HDT)
│   │   ├── serial_port.py      # Serial port management (pyserial)
│   │   ├── udp_socket.py       # UDP send/receive
│   │   ├── pgn_protocol.py     # PGN message encode/decode
│   │   └── isobus.py           # ISO 11783 protocol
│   │
│   ├── guidance/               # Guidance algorithms (Priority 3)
│   │   ├── __init__.py
│   │   ├── vehicle.py          # Vehicle config (dimensions, steering limits)
│   │   ├── steering.py         # Stanley + Pure Pursuit controllers
│   │   ├── ab_line.py          # Straight line guidance
│   │   ├── ab_curve.py         # Curve guidance
│   │   ├── contour.py          # Contour following
│   │   ├── uturn.py            # Automatic U-turn generation
│   │   └── recorded_path.py    # Path recording and replay
│   │
│   ├── field/                  # Field management (Priority 4)
│   │   ├── __init__.py
│   │   ├── boundary.py         # Field boundaries
│   │   ├── sections.py         # Section control (on/off per section)
│   │   ├── tool.py             # Implement configuration
│   │   ├── headland.py         # Headland detection and management
│   │   ├── track.py            # Track storage
│   │   ├── tram_lines.py       # Tramline tracking
│   │   └── field_data.py       # Field save/load
│   │
│   ├── sim/                    # Simulation (Priority 5)
│   │   ├── __init__.py
│   │   └── gps_simulator.py    # NMEA simulator for testing
│   │
│   └── app/                    # Application entry point
│       ├── __init__.py
│       ├── main.py             # Main application loop
│       └── config.py           # Settings management (replace Windows Registry)
│
├── ui/                         # UI (separate, choose framework)
│   └── ...                     # PyQt6 / Tkinter / web-based (Flask+JS)
│
└── tests/                      # Tests
    ├── test_vectors.py
    ├── test_coordinate_system.py
    ├── test_geometry.py
    ├── test_nmea_parser.py
    ├── test_steering.py
    ├── test_ab_line.py
    └── ...
```

## 5. Python Library Recommendations

| C# Dependency | Python Replacement | Purpose |
|---------------|-------------------|---------|
| OpenTK (OpenGL) | `pygame` / `moderngl` / `PyOpenGL` | Rendering |
| System.IO.Ports | `pyserial` | Serial communication |
| System.Net.Sockets (UDP) | `socket` (stdlib) | UDP networking |
| WinForms / WPF | `PyQt6` / `tkinter` / `dear imgui` | UI |
| GMap.NET | `folium` / `leaflet` (web) | Map display |
| Newtonsoft.Json | `json` (stdlib) | JSON serialization |
| System.Data.SQLite | `sqlite3` (stdlib) | Local database |
| Accord.Imaging | `opencv-python` / `Pillow` | Image processing |
| Windows Registry | `configparser` / `json` / `toml` | Settings storage |
| NumPy (new) | `numpy` | Vectorized math for coordinates |

## 6. Key Migration Challenges

### 6.1 FormGPS God Object
`FormGPS` is the central hub — nearly every class holds a reference to it (`mf`). In Python, use **dependency injection** and **event-driven architecture** instead:
```python
# Instead of: self.mf = form_gps  (C# pattern)
# Use: pass specific dependencies or use an event bus
class Guidance:
    def __init__(self, vehicle: Vehicle, track: Track):
        self.vehicle = vehicle
        self.track = track
```

### 6.2 Timer-Driven Main Loop
C# uses WinForms timers for the update loop. In Python, use `asyncio` or a simple game-loop pattern:
```python
import asyncio

async def main_loop():
    while running:
        nmea_data = await read_gps()
        position = coordinate_system.convert(nmea_data)
        steer_angle = guidance.calculate(position)
        await send_steer_command(steer_angle)
        await asyncio.sleep(0.1)  # 10 Hz update rate
```

### 6.3 Windows-Only Dependencies
The C# version is Windows-only (.NET Framework 4.8). Python gives you cross-platform by default, but:
- Replace Windows Registry settings with config files (TOML/JSON)
- Serial ports work cross-platform with `pyserial`
- UDP sockets are cross-platform with `socket` stdlib

### 6.4 OpenGL Rendering
The C# version uses OpenTK for field visualization. Options for Python:
- **PyOpenGL** — closest to current code, steep learning curve
- **pygame** — simpler, good for 2D field views
- **matplotlib** — good for debugging/visualization, not real-time
- **Web-based** (Flask + Leaflet.js) — modern approach, cross-platform

### 6.5 Performance Considerations
Some tight loops in C# (section control pixel checks, path calculations) may need:
- **NumPy** for vectorized coordinate math
- **Cython** or `numba` for hot path optimization
- Profile first, optimize only where needed

## 7. Migration Phases

### Phase 1: Core Math + Tests (Week 1-2)
- Implement `vectors.py`, `geo_types.py`, `coordinate_system.py`, `geometry.py`
- Write comprehensive tests using pytest
- Validate against C# outputs with known inputs

### Phase 2: NMEA Parsing + Simulator (Week 2-3)
- Implement NMEA parser for GGA, VTG, RMC sentences
- Build GPS simulator (port `CSim.cs`)
- Test with simulated NMEA data streams

### Phase 3: Steering Algorithms (Week 3-5)
- Implement Stanley controller and Pure Pursuit
- Implement AB Line guidance
- Test with simulator — verify steering outputs match C# version

### Phase 4: Communication (Week 5-6)
- UDP socket layer (AgIO equivalent)
- Serial port management
- PGN message protocol

### Phase 5: Field Management (Week 6-7)
- Boundaries, sections, headlands
- Field save/load (use JSON instead of custom binary)
- AB Curve and Contour guidance

### Phase 6: U-Turn + Dubins (Week 7-8)
- Dubins path planner
- U-turn generation
- Recorded path support

### Phase 7: UI (Week 8-12)
- Choose UI framework
- Build field view with guidance lines
- Configuration panels
- Real-time GPS display

## 8. Quick Start: First File to Port

Start with `vectors.py` — it has zero dependencies and everything builds on it:

```python
# agopenps/core/vectors.py
"""
Port of AgOpenGPS vec2/vec3 structs.
These represent local plane coordinates (easting/northing in meters).
"""
from __future__ import annotations
import math
from dataclasses import dataclass


@dataclass
class Vec2:
    """2D point in local coordinate space (easting, northing in meters)."""
    easting: float = 0.0
    northing: float = 0.0

    def __add__(self, other: Vec2) -> Vec2:
        return Vec2(self.easting + other.easting, self.northing + other.northing)

    def __sub__(self, other: Vec2) -> Vec2:
        return Vec2(self.easting - other.easting, self.northing - other.northing)

    def __mul__(self, scalar: float) -> Vec2:
        return Vec2(self.easting * scalar, self.northing * scalar)

    def heading_xz(self) -> float:
        """Returns heading angle in radians."""
        return math.atan2(self.easting, self.northing)

    def length(self) -> float:
        return math.sqrt(self.easting**2 + self.northing**2)

    def length_squared(self) -> float:
        return self.easting**2 + self.northing**2

    def normalize(self) -> Vec2:
        ln = self.length()
        if abs(ln) < 1e-12:
            raise ZeroDivisionError("Cannot normalize zero-length vector")
        return Vec2(self.easting / ln, self.northing / ln)

    @staticmethod
    def lerp(a: Vec2, b: Vec2, t: float) -> Vec2:
        return Vec2(
            a.easting + (b.easting - a.easting) * t,
            a.northing + (b.northing - a.northing) * t,
        )

    @staticmethod
    def cross(a: Vec2, b: Vec2) -> float:
        return a.easting * b.northing - a.northing * b.easting

    @staticmethod
    def dot(a: Vec2, b: Vec2) -> float:
        return a.easting * b.easting + a.northing * b.northing

    @staticmethod
    def project_on_segment(a: Vec2, b: Vec2, p: Vec2) -> tuple[Vec2, float]:
        ab = b - a
        ab_len_sq = ab.length_squared()
        if ab_len_sq < 1e-6:
            return a, 0.0
        ap = p - a
        t = max(0.0, min(1.0, Vec2.dot(ap, ab) / ab_len_sq))
        return a + ab * t, t


@dataclass
class Vec3:
    """3D point: easting, northing (meters) + heading (radians)."""
    easting: float = 0.0
    northing: float = 0.0
    heading: float = 0.0

    def to_vec2(self) -> Vec2:
        return Vec2(self.easting, self.northing)
```

## 9. Total Estimated Scope

| Layer | C# Lines | Estimated Python Lines | Complexity |
|-------|----------|----------------------|------------|
| Core math | ~1,350 | ~800 | Low |
| Communication | ~1,200 | ~600 | Medium |
| Guidance algorithms | ~7,500 | ~4,500 | High |
| Field management | ~2,300 | ~1,400 | Medium |
| Simulation | ~300 | ~200 | Low |
| UI | ~10,000+ | ~5,000+ | High |
| **Total (non-UI)** | **~12,650** | **~7,500** | |
| **Total (with UI)** | **~22,650+** | **~12,500+** | |

The non-UI core is the most valuable part and is roughly 7,500 lines of Python — very achievable.
