# AgOpenGPS Python Migration Plan

## Target Repository: `AgOpenGps_migr_py`

**Goal**: Rewrite AgOpenGPS core logic in Python with extensibility for LiDAR sensor fusion and LLM-powered intelligent features.

---

## 1. Architecture Design

The original C# app uses `FormGPS` as a god object that everything references. The Python version uses a **message bus + dependency injection** architecture that makes it easy to plug in LiDAR and LLM modules.

```
┌─────────────────────────────────────────────────────────────────┐
│                        AgOpenGPS-Py                             │
│                                                                 │
│  ┌──────────────────────────────────────────────────────────┐   │
│  │                   Message Bus (EventBus)                  │   │
│  │  position.updated │ guidance.calculated │ obstacle.detected│  │
│  │  section.changed  │ boundary.entered   │ llm.suggestion   │  │
│  └─────┬──────┬──────┬──────┬──────┬──────┬──────┬──────────┘   │
│        │      │      │      │      │      │      │              │
│   ┌────┴─┐┌───┴──┐┌──┴───┐┌─┴────┐┌┴─────┐┌┴─────┐┌┴────────┐ │
│   │ GPS  ││Guide ││Field ││Steer ││LiDAR ││ LLM  ││   UI    │ │
│   │Module││ance  ││Mgmt  ││Output││Fusion ││Agent ││(Web/Qt) │ │
│   └──────┘└──────┘└──────┘└──────┘└──────┘└──────┘└─────────┘ │
│        │      │                      │        │                 │
│   ┌────┴──────┴──────────────────────┴────────┘                 │
│   │              Sensor Fusion Layer                            │
│   │  GPS + IMU + LiDAR → Fused Position + Environment Map      │
│   └─────────────────────────────────────────────────────────────┘
│                           │                                      │
│                    ┌──────┴──────┐                                │
│                    │  Comms Hub  │                                │
│                    │  (AgIO-Py)  │                                │
│                    └──┬──────┬──┘                                │
│               Serial  │      │  UDP                              │
│            ┌──────────┘      └──────────┐                        │
│      ┌─────┴─────┐           ┌──────────┴──┐                    │
│      │GPS/IMU HW │           │Steer/Section│                    │
│      │NMEA/UBX   │           │Hardware PGN │                    │
│      └───────────┘           └─────────────┘                    │
│                                                                  │
│      ┌───────────┐                                               │
│      │  LiDAR HW │  (Velodyne, Livox, RPLidar, etc.)           │
│      │  via ROS2 │  or direct SDK                               │
│      └───────────┘                                               │
└──────────────────────────────────────────────────────────────────┘
```

---

## 2. Project Structure

```
AgOpenGps_migr_py/
├── pyproject.toml
├── README.md
├── config/
│   └── default.toml              # Settings (replaces Windows Registry)
│
├── agopenps/                     # Main package
│   ├── __init__.py
│   │
│   ├── core/                     # Phase 1: Foundation
│   │   ├── __init__.py
│   │   ├── vectors.py            # Vec2, Vec3 (from vec3.cs)
│   │   ├── geo_types.py          # Wgs84, GeoCoord, GeoDir
│   │   ├── local_plane.py        # WGS84 ↔ local plane transforms
│   │   ├── geometry.py           # Point-in-polygon, distances, intersections
│   │   ├── dubins.py             # Dubins path planning (6 path types)
│   │   └── config.py             # TOML-based settings manager
│   │
│   ├── comms/                    # Phase 2: Communication
│   │   ├── __init__.py
│   │   ├── event_bus.py          # Pub/sub message bus (replaces FormGPS refs)
│   │   ├── nmea_parser.py        # NMEA 0183 parser (GGA, VTG, RMC, HDT)
│   │   ├── serial_manager.py     # Serial port wrapper (pyserial)
│   │   ├── udp_manager.py        # UDP socket send/receive
│   │   └── pgn_protocol.py       # PGN message encode/decode (0x80 0x81 format)
│   │
│   ├── guidance/                 # Phase 3: Steering algorithms
│   │   ├── __init__.py
│   │   ├── vehicle.py            # Vehicle config (wheelbase, antenna, limits)
│   │   ├── stanley.py            # Stanley steering controller
│   │   ├── pure_pursuit.py       # Pure Pursuit steering controller
│   │   ├── ab_line.py            # Straight AB Line guidance
│   │   ├── ab_curve.py           # Curved path guidance
│   │   ├── contour.py            # Contour following
│   │   ├── uturn.py              # Automatic U-turn at headlands
│   │   └── recorded_path.py      # Path recording and replay
│   │
│   ├── field/                    # Phase 4: Field management
│   │   ├── __init__.py
│   │   ├── boundary.py           # Field boundary polygons
│   │   ├── boundary_builder.py   # Build boundaries from tracks
│   │   ├── sections.py           # Section on/off control (16/64 sections)
│   │   ├── tool.py               # Implement configuration
│   │   ├── headland.py           # Headland detection
│   │   ├── track.py              # Track storage and management
│   │   ├── tram_lines.py         # Tramline tracking
│   │   └── field_io.py           # Field save/load (JSON format)
│   │
│   ├── lidar/                    # Phase 5: LiDAR integration
│   │   ├── __init__.py
│   │   ├── base.py               # Abstract LiDAR interface
│   │   ├── drivers/
│   │   │   ├── __init__.py
│   │   │   ├── rplidar.py        # RPLidar 2D driver
│   │   │   ├── livox.py          # Livox Mid-series driver
│   │   │   └── ros2_bridge.py    # ROS2 PointCloud2 subscriber
│   │   ├── point_cloud.py        # Point cloud data structures (numpy-based)
│   │   ├── ground_filter.py      # Ground plane extraction (RANSAC/cloth)
│   │   ├── obstacle_detector.py  # Real-time obstacle detection
│   │   ├── crop_analyzer.py      # Crop row detection + height estimation
│   │   ├── terrain_mapper.py     # Terrain elevation mapping
│   │   └── fusion.py             # GPS + IMU + LiDAR sensor fusion
│   │
│   ├── llm/                      # Phase 6: LLM integration
│   │   ├── __init__.py
│   │   ├── agent.py              # LLM agent (Claude API) for field decisions
│   │   ├── field_analyst.py      # Analyze field data, suggest optimizations
│   │   ├── voice_commands.py     # Natural language → guidance commands
│   │   ├── anomaly_reporter.py   # Interpret sensor anomalies in plain English
│   │   ├── log_analyzer.py       # Analyze operation logs for insights
│   │   └── prompts/
│   │       ├── field_analysis.py # Prompt templates for field analysis
│   │       ├── guidance.py       # Prompt templates for guidance suggestions
│   │       └── diagnostics.py    # Prompt templates for troubleshooting
│   │
│   ├── sim/                      # Phase 7: Simulation
│   │   ├── __init__.py
│   │   ├── gps_sim.py            # GPS/NMEA simulator (from CSim.cs)
│   │   ├── lidar_sim.py          # Simulated LiDAR point clouds
│   │   └── field_sim.py          # Simulated field environments
│   │
│   └── app/                      # Phase 8: Application
│       ├── __init__.py
│       ├── main.py               # Main entry point + async event loop
│       └── state.py              # Application state container
│
├── ui/                           # Phase 9: User interface
│   ├── web/                      # Web-based UI (recommended for cross-platform)
│   │   ├── app.py                # FastAPI/Flask backend
│   │   ├── static/
│   │   │   ├── js/
│   │   │   │   ├── map.js        # Leaflet.js field map
│   │   │   │   ├── guidance.js   # Guidance line rendering
│   │   │   │   └── dashboard.js  # Real-time telemetry
│   │   │   └── css/
│   │   └── templates/
│   │       └── index.html
│   └── qt/                       # Alternative: PyQt6 desktop UI
│       └── ...
│
└── tests/
    ├── conftest.py               # Shared fixtures
    ├── core/
    │   ├── test_vectors.py
    │   ├── test_local_plane.py
    │   ├── test_geometry.py
    │   └── test_dubins.py
    ├── comms/
    │   ├── test_nmea_parser.py
    │   └── test_pgn_protocol.py
    ├── guidance/
    │   ├── test_stanley.py
    │   ├── test_pure_pursuit.py
    │   └── test_ab_line.py
    ├── lidar/
    │   ├── test_obstacle_detector.py
    │   └── test_ground_filter.py
    └── llm/
        └── test_field_analyst.py
```

---

## 3. Phase Details

### Phase 1: Core Foundation (Week 1-2)

**Files to create**: `core/vectors.py`, `core/geo_types.py`, `core/local_plane.py`, `core/geometry.py`, `core/dubins.py`, `core/config.py`

**Source C# files to port**:
| Python target | C# source | Key logic |
|---|---|---|
| `vectors.py` | `vec3.cs` (172 lines) | Vec2/Vec3 with easting/northing/heading, operators, Lerp, Cross, Dot, ProjectOnSegment |
| `geo_types.py` | `Wgs84.cs`, `GeoCoord.cs`, `GeoDir.cs` | WGS84 lat/lon, local coordinates, direction angles |
| `local_plane.py` | `LocalPlane.cs` (58 lines) | `metersPerDegreeLat/Lon` formulas, WGS84 ↔ GeoCoord conversion |
| `geometry.py` | `CGLM.cs` (427 lines) | `IsPointInPolygon` (ray casting), `InRangeBetweenAB`, `DistanceFrom`, spline interpolation |
| `dubins.py` | `CDubins.cs` (636 lines) | 6 Dubins path types (RSR/LSL/RSL/LSR/RLR/LRL), tangent computation, path discretization |
| `config.py` | (new) | TOML config file reader/writer replacing Windows Registry |

**Key design decisions**:
- Use `@dataclass` for Vec2/Vec3 (not numpy arrays) — keeps API clean, easy to debug
- Use `numpy` internally in geometry.py for batch operations where performance matters
- Config: use `tomllib` (stdlib Python 3.11+) for reading, `tomli-w` for writing

**Tests**: Validate all coordinate transforms against known GPS points. Validate geometry functions against C# test cases.

---

### Phase 2: Communication Layer (Week 2-3)

**Files to create**: `comms/event_bus.py`, `comms/nmea_parser.py`, `comms/serial_manager.py`, `comms/udp_manager.py`, `comms/pgn_protocol.py`

**Source C# files to port**:
| Python target | C# source | Key logic |
|---|---|---|
| `event_bus.py` | (new — replaces FormGPS god object) | Pub/sub with typed events, async-compatible |
| `nmea_parser.py` | `CNMEA.cs` + NMEA parsing in AgIO | Parse GGA (fix/position), VTG (speed/heading), RMC (date/time/position), HDT (true heading) |
| `serial_manager.py` | AgIO serial handling | `pyserial` wrapper with auto-reconnect, baud rate config |
| `udp_manager.py` | `UDPComm.Designer.cs` | Send/receive on loopback sockets, broadcast support |
| `pgn_protocol.py` | `PGN.Designer.cs` | Binary message format: `[0x80][0x81][0x7F][type][length][payload][checksum]` |

**Event Bus design** (critical — this replaces the FormGPS reference pattern):
```python
# comms/event_bus.py
class EventBus:
    """Decoupled message passing — replaces C# FormGPS god object."""

    async def publish(self, event: str, data: Any) -> None: ...
    def subscribe(self, event: str, callback: Callable) -> None: ...

# Events:
# "gps.fix"          → {lat, lon, alt, speed, heading, satellites, hdop}
# "imu.update"       → {roll, pitch, yaw, heading}
# "guidance.output"  → {steer_angle, cross_track_error, mode}
# "section.update"   → {section_states: list[bool]}
# "obstacle.detected"→ {distance, bearing, size, confidence}
# "llm.suggestion"   → {text, action_type, confidence}
```

**PGN message format** (from C# `PGN.Designer.cs`):
```
Header:    0x80 0x81
Source:    0x7F (AgOpenGPS)
Type:      0xFE (autosteer data), 0xFC (autosteer settings),
           0xD6 (GPS data from AgIO), 0xD0 (lat/lon encoded)
Length:    1 byte (payload size)
Payload:   [length] bytes
Checksum:  sum of bytes [2] to [4+length], mod 256
```

---

### Phase 3: Guidance Algorithms (Week 3-5)

**The core value of the project.** These implement the actual precision agriculture logic.

**Files to create**: `guidance/vehicle.py`, `guidance/stanley.py`, `guidance/pure_pursuit.py`, `guidance/ab_line.py`, `guidance/ab_curve.py`, `guidance/contour.py`, `guidance/uturn.py`, `guidance/recorded_path.py`

**Source C# files to port**:
| Python target | C# source | Lines | Key logic |
|---|---|---|---|
| `vehicle.py` | `CVehicle.cs` | 386 | Wheelbase, antenna offsets, steer limits, look-ahead params |
| `stanley.py` | `CGuidance.cs:42-194` | 150 | Stanley controller: heading_error × gain + atan(xtrack × gain / speed) + integral |
| `pure_pursuit.py` | `CABLine.cs:186-300` | 115 | Goal point at look-ahead, radius = D²/(2×crosstrack), angle = atan(wheelbase/radius) |
| `ab_line.py` | `CABLine.cs` | 568 | AB Line management, parallel line offsets, line building |
| `ab_curve.py` | `CABCurve.cs` | 1724 | Curve point lists, closest-point search, Catmull-Rom smoothing |
| `contour.py` | `CContour.cs` | 671 | Strip recording, contour extraction, contour locking |
| `uturn.py` | `CYouTurn.cs` | 2926 | Turn pattern generation, boundary detection, state machine |
| `recorded_path.py` | `CRecordedPath.cs` | 849 | Path capture, Dubins transitions, playback |

**Stanley Controller** (from `CGuidance.cs:42-109`):
```python
# Key algorithm to port:
def calculate_steer_angle(self, heading_error, xtrack_steer, xtrack_pivot,
                           speed, is_reverse):
    if is_reverse:
        heading_error *= -1
    heading_error *= self.heading_error_gain

    # Speed compensation
    sped = abs(speed)
    sped = 1 + 0.277 * (sped - 1) if sped > 1 else 1.0

    # Cross-track correction with low-pass filter
    xtec = math.atan(xtrack_steer * self.distance_error_gain / sped)
    self._xtrack_correction = self._xtrack_correction * 0.5 + xtec * 0.5

    # Steering angle
    steer_angle = math.degrees((self._xtrack_correction + heading_error) * -1.0)

    # Scale based on distance from line
    if abs(xtrack_steer) > 0.5:
        steer_angle *= 0.5
    else:
        steer_angle *= (1 - abs(xtrack_steer))

    # Integral term for steady-state offset
    # ... (see CGuidance.cs:75-93)

    return clamp(steer_angle, -self.max_steer_angle, self.max_steer_angle)
```

---

### Phase 4: Field Management (Week 5-6)

**Files to create**: `field/boundary.py`, `field/boundary_builder.py`, `field/sections.py`, `field/tool.py`, `field/headland.py`, `field/track.py`, `field/tram_lines.py`, `field/field_io.py`

**Source C# files to port**:
| Python target | C# source | Lines |
|---|---|---|
| `boundary.py` | `CBoundary.cs` + `CBoundaryList.cs` | ~300 |
| `boundary_builder.py` | `BoundaryBuilder.cs` | 614 |
| `sections.py` | `CSection.cs` | 76 |
| `tool.py` | `CTool.cs` | 383 |
| `headland.py` | `CHead.cs` + `CHeadLine.cs` | ~300 |
| `track.py` | `CTrack.cs` | 351 |
| `tram_lines.py` | `CTram.cs` | 243 |
| `field_io.py` | Core streamers | ~500 |

**Key change**: Use JSON for field save/load instead of the C# binary streamers. This makes data portable and LLM-readable.

---

### Phase 5: LiDAR Integration (Week 6-8)

**New capability** — not in original AgOpenGPS.

#### 5.1 Hardware Abstraction

```python
# lidar/base.py
from abc import ABC, abstractmethod
import numpy as np

class LiDARBase(ABC):
    """Abstract interface for all LiDAR sensors."""

    @abstractmethod
    async def connect(self) -> None: ...

    @abstractmethod
    async def get_scan(self) -> np.ndarray:
        """Returns Nx3 (x,y,z) or Nx4 (x,y,z,intensity) point cloud."""
        ...

    @abstractmethod
    def get_info(self) -> dict:
        """Returns sensor metadata: model, range, FoV, scan rate."""
        ...
```

#### 5.2 Drivers

| Driver | Hardware | Interface | Use Case |
|--------|----------|-----------|----------|
| `rplidar.py` | RPLidar A1/A2/S1 | USB serial | 2D obstacle detection (low cost, $100-300) |
| `livox.py` | Livox Mid-40/70 | Ethernet UDP | 3D terrain mapping (mid cost, $600-1500) |
| `ros2_bridge.py` | Any ROS2-compatible | ROS2 topics | Universal bridge for Velodyne, Ouster, etc. |

#### 5.3 Processing Pipeline

```
Raw Point Cloud (10-100K pts/scan)
    │
    ▼
Ground Filter (RANSAC or Cloth Simulation)
    │
    ├──► Ground Points → Terrain Mapper
    │                     └─► elevation map, slope analysis
    │                     └─► integrate with field boundary
    │
    └──► Non-Ground Points → Obstacle Detector
                              └─► cluster (DBSCAN)
                              └─► classify (size, shape, motion)
                              └─► publish "obstacle.detected" event
                              │
                              └──► Crop Analyzer
                                   └─► row detection (Hough transform)
                                   └─► height estimation
                                   └─► skip/weed detection
```

#### 5.4 Sensor Fusion

```python
# lidar/fusion.py
class SensorFusion:
    """Fuses GPS + IMU + LiDAR into unified world model."""

    def __init__(self, local_plane, event_bus):
        self.local_plane = local_plane
        self.event_bus = event_bus
        self._gps_position = None   # Updated by GPS events
        self._imu_attitude = None   # Updated by IMU events
        self._point_cloud = None    # Updated by LiDAR events

    def fused_position(self) -> FusedState:
        """
        Returns position with confidence interval.
        GPS provides absolute position, LiDAR provides relative corrections.
        IMU provides attitude (roll/pitch/yaw) for point cloud transform.
        """
        ...

    def obstacle_map(self) -> ObstacleGrid:
        """
        2D grid around vehicle with obstacle probabilities.
        Used by guidance to avoid obstacles and by LLM for situation awareness.
        """
        ...
```

#### 5.5 Guidance Integration

The obstacle detector publishes events that the guidance module subscribes to:

```python
# In guidance/ab_line.py
class ABLineGuidance:
    def __init__(self, event_bus, vehicle, ...):
        event_bus.subscribe("obstacle.detected", self._on_obstacle)

    def _on_obstacle(self, obstacle):
        if obstacle.distance < self.safety_distance:
            # Option 1: Stop and alert
            self.event_bus.publish("guidance.emergency_stop", {})
            # Option 2: Plan avoidance path (with LLM consultation)
            self.event_bus.publish("llm.request", {
                "type": "obstacle_avoidance",
                "obstacle": obstacle,
                "current_guidance": self.current_state
            })
```

---

### Phase 6: LLM Integration (Week 8-10)

**New capability** — not in original AgOpenGPS.

#### 6.1 LLM Agent Architecture

```python
# llm/agent.py
import anthropic

class FieldAgent:
    """Claude-powered agent for intelligent field operations."""

    def __init__(self, event_bus, field_state):
        self.client = anthropic.Anthropic()
        self.event_bus = event_bus
        self.field_state = field_state

        # Subscribe to events that need intelligence
        event_bus.subscribe("obstacle.detected", self._analyze_obstacle)
        event_bus.subscribe("field.anomaly", self._analyze_anomaly)
        event_bus.subscribe("operator.voice_command", self._process_command)

    async def _analyze_obstacle(self, obstacle_data):
        """Decide what to do about a detected obstacle."""
        response = await self.client.messages.create(
            model="claude-sonnet-4-20250514",
            messages=[{
                "role": "user",
                "content": f"""
                Agricultural vehicle obstacle detected:
                - Distance: {obstacle_data['distance']}m
                - Size: {obstacle_data['width']}m x {obstacle_data['height']}m
                - Position: {obstacle_data['bearing']}° from heading
                - Current operation: {self.field_state.current_operation}
                - Speed: {self.field_state.speed} km/h

                Should the vehicle: stop, slow down, steer around, or continue?
                Respond with JSON: {{"action": "...", "reason": "..."}}
                """
            }]
        )
        # Parse and publish decision
        ...
```

#### 6.2 Use Cases

| Feature | Input | LLM Does | Output |
|---------|-------|----------|--------|
| **Voice Commands** | "Start spraying the north field" | Parse intent → map to guidance commands | Set boundary, enable sections, start guidance |
| **Obstacle Decisions** | LiDAR obstacle + context | Classify (rock/animal/person/equipment) and recommend action | Stop/avoid/continue decision |
| **Field Analysis** | Boundary + coverage + yield data | Identify patterns, suggest optimization | "Consider 15° AB angle for better drainage" |
| **Anomaly Reports** | GPS drift, section failures, unusual patterns | Diagnose root cause in plain English | "GPS accuracy degraded — NTRIP connection lost 5 min ago" |
| **Operation Planning** | Field shape, crop type, equipment | Suggest optimal pass pattern | AB angle, headland width, section config |
| **Log Analysis** | Session logs, sensor data | Find trends, predict maintenance | "Steering response time increased 15% — check hydraulic pressure" |

#### 6.3 Tool Use for Autonomous Actions

```python
# The LLM agent can use tools to interact with the guidance system
tools = [
    {
        "name": "set_guidance_line",
        "description": "Set or modify the AB guidance line",
        "input_schema": {
            "type": "object",
            "properties": {
                "point_a": {"type": "object", "properties": {"lat": ..., "lon": ...}},
                "point_b": {"type": "object", "properties": {"lat": ..., "lon": ...}},
            }
        }
    },
    {
        "name": "set_sections",
        "description": "Turn implement sections on or off",
        "input_schema": {
            "type": "object",
            "properties": {
                "sections": {"type": "array", "items": {"type": "boolean"}}
            }
        }
    },
    {
        "name": "emergency_stop",
        "description": "Stop the vehicle immediately",
        "input_schema": {"type": "object", "properties": {}}
    },
    {
        "name": "query_field_data",
        "description": "Get current field statistics",
        "input_schema": {
            "type": "object",
            "properties": {
                "metric": {"type": "string", "enum": ["coverage", "boundary", "elevation", "obstacles"]}
            }
        }
    }
]
```

---

### Phase 7: Simulation (Week 10-11)

**Files to create**: `sim/gps_sim.py`, `sim/lidar_sim.py`, `sim/field_sim.py`

Port `CSim.cs` and extend with:
- **LiDAR simulator**: Generate synthetic point clouds with configurable obstacles, terrain, crop rows
- **Field simulator**: Pre-built field environments for testing (flat field, hilly terrain, irregularly shaped fields)
- **Full-stack sim**: Run entire pipeline (GPS sim → guidance → LiDAR sim → obstacle detection → LLM) without hardware

```python
# sim/gps_sim.py (from CSim.cs)
class GPSSimulator:
    def tick(self, steer_angle: float) -> GPSFix:
        # Port CSim.DoSimTick:
        # 1. Smooth steer angle transitions
        # 2. Calculate heading change: heading += step * tan(angle * 0.0165329252) / 2
        # 3. Calculate new lat/lon from heading + distance
        # 4. Return simulated GPS fix
        ...
```

---

### Phase 8: Application + UI (Week 11-14)

**Main loop** (replaces WinForms timer):
```python
# app/main.py
async def main():
    config = Config.load("config/default.toml")
    bus = EventBus()

    # Initialize modules
    gps = GPSModule(bus, config.gps)
    guidance = GuidanceEngine(bus, config.vehicle, config.guidance)
    field_mgr = FieldManager(bus, config.field_path)
    sections = SectionController(bus, config.sections)

    # Optional modules
    if config.lidar.enabled:
        lidar = LiDARModule(bus, config.lidar)
        fusion = SensorFusion(bus, gps, lidar)

    if config.llm.enabled:
        agent = FieldAgent(bus, config.llm)

    # Start async event loop
    await asyncio.gather(
        gps.run(),           # 10 Hz GPS read
        guidance.run(),      # 10 Hz guidance calc
        sections.run(),      # 5 Hz section updates
        lidar.run() if config.lidar.enabled else asyncio.sleep(0),
        ui.run(),            # WebSocket push to browser
    )
```

**UI recommendation**: Web-based (FastAPI + WebSocket + Leaflet.js)
- Works on any device (tablet in tractor cab, phone, laptop)
- Real-time map with guidance lines via WebSocket
- LLM chat interface built-in
- Easier to iterate on than desktop UI

---

## 4. Dependencies

```toml
# pyproject.toml
[project]
name = "agopenps"
requires-python = ">=3.11"
dependencies = [
    # Core
    "numpy>=1.26",
    "tomli-w>=1.0",          # TOML writing (reading via stdlib tomllib)

    # Communication
    "pyserial>=3.5",          # Serial port

    # LiDAR
    "open3d>=0.18",           # Point cloud processing
    "scikit-learn>=1.4",      # DBSCAN clustering, RANSAC
    "rplidar-roboticia>=0.9", # RPLidar driver (optional)

    # LLM
    "anthropic>=0.40",        # Claude API

    # UI (web)
    "fastapi>=0.110",
    "uvicorn>=0.29",
    "websockets>=12.0",

    # Testing
    "pytest>=8.0",
    "pytest-asyncio>=0.23",
]

[project.optional-dependencies]
ros2 = ["rclpy"]             # ROS2 bridge (install separately)
qt = ["PyQt6>=6.6"]          # Alternative desktop UI
```

---

## 5. Implementation Order & Milestones

```
Week 1-2:   Phase 1 - Core math + tests          → Can convert coordinates
Week 2-3:   Phase 2 - Comms + event bus           → Can receive GPS data
Week 3-5:   Phase 3 - Guidance algorithms         → Can calculate steering
Week 5-6:   Phase 4 - Field management            → Can manage boundaries/sections
            ─── MILESTONE: Feature parity with basic AgOpenGPS guidance ───
Week 6-8:   Phase 5 - LiDAR integration           → Can detect obstacles
            ─── MILESTONE: LiDAR-assisted guidance working ───
Week 8-10:  Phase 6 - LLM integration             → Can make intelligent decisions
            ─── MILESTONE: Voice commands + obstacle reasoning ───
Week 10-11: Phase 7 - Simulation                  → Can test without hardware
Week 11-14: Phase 8+9 - App + UI                  → Full working application
            ─── MILESTONE: Complete system ready for field testing ───
```

---

## 6. Key Design Principles

1. **Event-driven, not god-object**: Every module communicates via `EventBus`, not direct references. This makes LiDAR and LLM pluggable.

2. **Async-first**: Use `asyncio` for all I/O (GPS, serial, UDP, LiDAR, LLM API calls). Guidance math runs synchronously within async tasks.

3. **NumPy for hot paths**: Vectorize coordinate transforms, point cloud processing, and polygon checks. Keep single-point operations as pure Python for readability.

4. **JSON for data**: Field files, configs, and LLM context all use human-readable JSON/TOML. No binary formats except hardware protocols.

5. **Test from day one**: Every module gets tests. Simulate hardware with recorded data. Validate against C# outputs for ported algorithms.

6. **Cross-platform**: No Windows dependencies. Runs on Linux (Raspberry Pi, Jetson), macOS, Windows.

7. **Safety boundaries for LLM**: LLM can suggest actions but critical safety decisions (emergency stop, steer angle limits) always go through deterministic code with hard limits. The LLM never directly controls hardware.
