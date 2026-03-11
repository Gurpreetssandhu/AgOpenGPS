# CLAUDE.md - AgOpenGPS Development Guide

## Project Overview

AgOpenGPS is agricultural precision mapping, guidance, and section control software. It reads NMEA strings for position mapping, provides up to 16/64 sections of implement control, and outputs Pure Pursuit steer angles for AB line, AB Curve, and Contour guidance. The system consists of two main programs: **AgOpenGPS** (the guidance application) and **AgIO** (the communication hub to external hardware).

## Repository Structure

```
AgOpenGPS/
├── SourceCode/                    # All source code
│   ├── AgOpenGPS.sln              # Visual Studio 2022 solution (13 projects)
│   ├── Directory.Build.props      # Shared build settings (net48, analyzers)
│   ├── .editorconfig              # Code style rules
│   │
│   ├── GPS/                       # Main WinForms guidance application (AgOpenGPS.exe)
│   │   ├── Classes/               # Core logic (CABLine, CBoundary, CNMEA, CGuidance, etc.)
│   │   ├── Forms/                 # UI forms for config, serial, UDP, NMEA
│   │   ├── Controls/              # Custom UI controls
│   │   ├── IO/                    # Input/output handling
│   │   └── Resources/             # Icons, images, buttons
│   │
│   ├── AgOpenGPS.Core/            # Core business logic library
│   │   ├── DrawLib/               # OpenGL rendering (OpenTK 3.3.3 wrapper)
│   │   ├── Models/                # Data models
│   │   ├── ViewModels/            # MVVM view models
│   │   ├── Presenters/            # Application/field presenters
│   │   ├── Streamers/             # Data serialization
│   │   ├── Interfaces/            # Shared interfaces
│   │   ├── Translations/          # i18n (Weblate-managed)
│   │   └── Performance/           # Performance monitoring
│   │
│   ├── AgOpenGPS.WpfApp/          # Modern WPF application
│   ├── AgOpenGPS.WpfViews/        # WPF view components
│   ├── AgIO/Source/               # Communication hub (serial/UDP gateway)
│   ├── AgLibrary/                 # Shared utilities (controls, logging, settings)
│   ├── Keypad/                    # Touch-friendly keypad controls
│   ├── ModSim/Source/             # NMEA/Modbus simulator for testing
│   ├── GPS_Out/Source/            # GPS data output utility
│   ├── AgDiag/                    # Diagnostic tool (.NET Framework 4.8)
│   │
│   ├── AgLibrary.Tests/           # Tests for AgLibrary
│   ├── AgOpenGPS.Core.Tests/      # Tests for Core library
│   └── AgOpenGPS.Tests/           # General application tests
│
├── .github/
│   ├── workflows/build.yml        # CI: develop branch + PRs
│   └── workflows/release.yml      # Release: master + release/* branches
└── global.json                    # .NET SDK 9.0.300
```

## Build & Development Commands

**Prerequisites:** .NET SDK 9.0.300+ (see `global.json`)

```sh
# Restore dependencies
dotnet restore SourceCode/AgOpenGPS.sln

# Build (Debug)
dotnet build SourceCode/AgOpenGPS.sln

# Build (Release) - warnings are errors in Release mode
dotnet build --configuration Release SourceCode/AgOpenGPS.sln

# Run tests
dotnet test SourceCode/AgOpenGPS.sln

# Publish all applications to ./AgOpenGPS/
dotnet publish SourceCode/AgOpenGPS.sln
```

**Important:** The target framework is **.NET Framework 4.8** (`net48`), set in `SourceCode/Directory.Build.props`. This is a Windows-only project.

## Testing

- Framework: **NUnit 4.3.2** with NUnit3TestAdapter
- Test SDK: Microsoft.NET.Test.Sdk 17.12.0
- Run all tests: `dotnet test SourceCode/AgOpenGPS.sln`
- Test projects follow naming convention: `[ProjectName].Tests`

## Code Style & Conventions

### Enforced via .editorconfig and build

- **Naming:** PascalCase for types, properties, methods, events
- **Interfaces:** Prefixed with `I` (e.g., `IFieldPresenter`)
- **Class naming:** Many domain classes use `C` prefix (e.g., `CABLine`, `CBoundary`, `CNMEA`, `CGuidance`, `CSection`)
- **Modifier ordering:** `public static` not `static public` (IDE0036 = warning)
- **Formatting:** IDE0055 = warning
- **.NET Analyzers** are enabled; code style is enforced in build
- **Release builds** treat warnings as errors (`TreatWarningsAsErrors`)

### Observed patterns

- WinForms classes prefixed with `Form` (e.g., `FormGPS`, `FormDialog`)
- Fields often lack explicit prefix convention — follow existing patterns in each file
- Domain-specific `*.Designer.cs` files (ConfigData, NMEA, PGN, etc.) are **not** auto-generated — `generated_code = false` is set so analyzers run on them
- Translations managed via Weblate — do not manually edit translation resource files

## Git Workflow

### Branching model

- **`master`** — stable releases (draft GitHub Releases created on push)
- **`develop`** — active development (CI builds and tests run)
- **`release/*`** — pre-release branches
- **Feature branches** — named descriptively, target `develop` via PR

### Contributing

1. Branch from `develop`
2. Create a descriptively named feature branch
3. Make changes and commit
4. Open a PR targeting `develop`

### Commit message style

Descriptive messages, commonly seen patterns:
- `Add <feature>` — new features
- `Refactor: <description>` — code restructuring
- `Replace <old> with <new>` — updates/replacements
- `Fix <issue>` — bug fixes
- `Enable <analyzer/rule>` — tooling changes

### Versioning

Uses **GitVersion 5.12.x** for automatic semantic versioning from git history.

## CI/CD

- **Build workflow** (`build.yml`): Runs on `develop` pushes and all PRs
  - Restore → Build → Test → Publish → Upload artifact
- **Release workflow** (`release.yml`): Runs on `master` and `release/*` pushes
  - Same build steps + creates ZIP archive + GitHub Release
- Runner: `windows-latest`

## Key Dependencies

| Package | Version | Purpose |
|---------|---------|---------|
| OpenTK | 3.3.3 | OpenGL rendering |
| Newtonsoft.Json | 13.0.4 | JSON serialization |
| Accord.Imaging | 3.8.0 | Image processing |
| Accord.Video.DirectShow | 3.8.0 | Video capture |
| GMap.NET.WinForms | 2.1.7 | Map display |
| Dev4Agriculture.ISO11783.ISOXML | 0.23.1.1 | ISOXML agricultural data |
| System.Data.SQLite | 2.0.1 | Local database |
| NUnit | 4.3.2 | Testing framework |

## Architecture Notes

- **AgOpenGPS** and **AgIO** are the two main executables — either can launch the other
- Communication with hardware uses **UDP** and **serial ports** (NMEA protocol)
- Graphics rendering uses **OpenGL** via OpenTK wrapper in `AgOpenGPS.Core/DrawLib/`
- The project maintains both **WinForms** (legacy, full-featured) and **WPF** (modern) UIs
- `AgOpenGPS.Core` contains shared business logic used by both UI implementations
- `AgLibrary` provides cross-cutting utilities (logging, settings, controls)
- `ModSim` enables development/testing without physical GPS hardware

## Domain Concepts

- **NMEA** — GPS data protocol (sentences like GGA, VTG, RMC)
- **Pure Pursuit** — steering algorithm for guidance line following
- **AB Line / AB Curve / Contour** — types of guidance reference lines
- **Sections** — implement sections for application control (up to 16 unique or 64 same-width)
- **Headland / UTurn** — automatic turning at field boundaries
- **PGN** — Parameter Group Number messages for implement communication
- **ISOBUS** — ISO 11783 agricultural equipment communication standard

## License

GPLv3 — all contributions must maintain this license.
