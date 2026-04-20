# UAM25 Control Hub — System Documentation

> **Project:** Urban Air Mobility (UAM) multi-drone control and streaming platform
> **Stack:** React + Electron (frontend) · Python FastAPI / MAVSDK (backend) · Node.js + FFmpeg (streamer)
> **Network:** LAN — control host at `192.168.1.200`, drones at `192.168.1.31–37`

---

## Table of Contents

1. [System Overview](#1-system-overview)
2. [Component Descriptions](#2-component-descriptions)
3. [Architecture Graph (graph TD)](#3-architecture-graph)
4. [Data Flow Diagram](#4-data-flow-diagram)
5. [Activity Diagrams](#5-activity-diagrams)
6. [Network & Port Reference](#6-network--port-reference)
7. [Environment Configuration](#7-environment-configuration)
8. [Directory Structure](#8-directory-structure)

---

## 1. System Overview

UAM25 is a ground-control system for coordinating and monitoring a fleet of up to 8 autonomous drones. It provides:

- **Real-time telemetry** — position, velocity, attitude, battery, armed state, flight mode
- **Live video streaming** — RTSP feeds from drone cameras converted to WebSocket-delivered MPEG1
- **Mission management** — UAM-specific flight missions, geofencing, formation control
- **DroneManager (DM) integration** — connects to the DroneManager service via UDP external plugin for state updates

The system is composed of three independently deployable services:

| Service | Technology | Purpose |
|---|---|---|
| **Frontend** | React 18 + Electron 41 + Vite | UI — map, drone cards, video feeds |
| **Backend (fromDM)** | Python, FastAPI, MAVSDK, Textual | Drone control, telemetry bridge, mission logic |
| **StreamerMultiple** | Node.js, Express, FFmpeg, WS | RTSP → MPEG1 → WebSocket video pipeline |

---

## 2. Component Descriptions

### 2.1 Frontend (`FRONTEND_UAM/`)

Built with **React 18**, **TypeScript**, **TailwindCSS**, and **Electron 41**. Can run as a desktop app or plain web app.

| Module | Path | Role |
|---|---|---|
| Main Screen | `src/screens/Element/Element.tsx` | Root layout — map + sidebar + panels |
| Map View | `src/components/map/MapView.tsx` | Renders drone positions on map |
| Drone Sidebar | `src/components/drones/DroneCardsSidebar.tsx` | Live drone status cards |
| Drone Card | `src/components/drones/DroneCard.tsx` | Per-drone JSMPEG video + telemetry |
| WebSocket Hook | `src/hooks/useWebSocket.ts` | Connects to backend on `ws://<host>:8765` |
| Connection Store | `src/stores/connection-store.tsx` | Zustand state — drone data, DM status, swap state |
| API Client | `src/api/api.tsx` | Axios HTTP client for REST commands |

**Key env vars:**
```
VITE_WS_URL=ws://192.168.1.200:8765
VITE_LAN_HOST=192.168.1.200
```

---

### 2.2 Backend (`backend/backend/fromDM/`)

Python service with two layers:

**Layer 1 — DroneControl TUI app** (`src/dronecontrol/app.py`)
Textual-based CLI that directly manages drone connections via MAVSDK/MAVLink. Handles arm/disarm, takeoff/land, fly-to, geofence, missions, plugins.

**Layer 2 — UDP Transporter** (`udp_transporter.py`)
Lightweight bridge: subscribes to DroneManager's external plugin (UDP `31659`), receives JSON telemetry, and broadcasts it over WebSocket on port `8765` to the frontend.

**Mock / Test senders:**
- `SAR-UAM-log.py` — sends static mock telemetry on UDP `31659`
- `uam_mock_sender.py` — synchronized multi-drone mock (takeoff → search → POI → land)

---

### 2.3 StreamerMultiple (`STREAMERMULTIPLE_UAM/STREAMERMULTIPLE/`)

Node.js Express server that bridges drone RTSP camera streams to browser-playable WebSocket MPEG1 streams.

| Drone | RTSP Source | WebSocket Out |
|---|---|---|
| green | `rtsp://192.168.1.31:8900/live` | `ws://<host>:9991` |
| yellow | `rtsp://192.168.1.33:8900/live` | `ws://<host>:9992` |
| blue | `rtsp://192.168.1.36:8900/live` | `ws://<host>:9993` |

FFmpeg transcodes each stream to MPEG1 at 1280×720, 30fps, zero-latency. The browser uses JSMpeg to decode and render on a `<canvas>`.

`/api/dm-status` — UDP probe endpoint: sends a subscription packet to DM's external plugin port and returns `{ connected: bool }` within 3 s.

**Build:** `npm run build` → `dist/StreamerMultiple.exe` (pkg-bundled, Windows x64)

---

### 2.4 DroneManager (`C:\Users\ttzhm\Documents\Workspace\DroneManager`)

External application. Exposes a **UDP external plugin server on port `31659`**. Clients subscribe by sending:
```json
{ "frequency": 5, "duration": 60 }
```
DM then pushes JSON telemetry updates back to the subscriber's UDP port for the specified duration.

---

## 3. Architecture Graph

```mermaid
graph TD
    subgraph Drones["Drone Fleet (192.168.1.31–37)"]
        D1[green · .31]
        D2[yellow · .33]
        D3[blue · .36]
        D4[wedge / tycho / gavin / jaina]
    end

    subgraph DM["DroneManager (external)"]
        DMApp[DroneManager App]
        DMExt[External Plugin\nUDP :31659]
        DMApp --> DMExt
    end

    subgraph Backend["Backend — Python"]
        UDPTrans[udp_transporter.py\nUDP subscriber → WS :8765]
        DroneCtrl[dronecontrol/app.py\nTextual TUI + MAVSDK]
        MockSender[uam_mock_sender.py\nMock UDP :31659]
        WebSrv[web_server.py\nFastAPI :8080]
    end

    subgraph Streamer["StreamerMultiple — Node.js"]
        Express[Express :3005\nUI + /api/dm-status]
        FFmpeg1[FFmpeg → MPEG1\nWS :9991]
        FFmpeg2[FFmpeg → MPEG1\nWS :9992]
        FFmpeg3[FFmpeg → MPEG1\nWS :9993]
        Express --> FFmpeg1
        Express --> FFmpeg2
        Express --> FFmpeg3
    end

    subgraph Frontend["Frontend — React + Electron"]
        UI[Main Screen\nMap + Sidebar]
        WsHook[useWebSocket\nws://host:8765]
        DroneCard[DroneCard\nJSMpeg canvas]
        Store[Zustand Store\nconnection-store]
        UI --> WsHook
        UI --> DroneCard
        WsHook --> Store
    end

    %% Drone → Streamer (RTSP camera)
    D1 -->|RTSP :8900| FFmpeg1
    D2 -->|RTSP :8900| FFmpeg2
    D3 -->|RTSP :8900| FFmpeg3

    %% Drone → DroneManager (MAVLink)
    Drones -->|MAVLink UDP| DMApp

    %% DM → Backend (UDP telemetry)
    DMExt -->|UDP telemetry| UDPTrans
    MockSender -->|UDP mock telemetry| UDPTrans

    %% Backend → Frontend (WebSocket telemetry)
    UDPTrans -->|WS :8765 JSON| WsHook

    %% Streamer → Frontend (video)
    FFmpeg1 -->|WS :9991 MPEG1| DroneCard
    FFmpeg2 -->|WS :9992 MPEG1| DroneCard
    FFmpeg3 -->|WS :9993 MPEG1| DroneCard

    %% Streamer DM probe
    Express -->|UDP probe :31659| DMExt

    %% DroneControl → Drones (MAVSDK)
    DroneCtrl -->|MAVSDK / MAVLink| Drones
```

---

## 4. Data Flow Diagram

```mermaid
flowchart LR
    subgraph Source["Data Sources"]
        RTSP["Drone Camera\nRTSP :8900"]
        MAV["Drone Flight Controller\nMAVLink UDP :1456x"]
        DM_OUT["DroneManager\nexternal plugin UDP :31659"]
    end

    subgraph Processing["Processing Layer"]
        FFMPEG["FFmpeg\nRTSP → MPEG1 ts"]
        WS_SRV["WebSocket Server\n:9991 / :9992 / :9993"]
        UDP_BRIDGE["udp_transporter.py\nUDP subscriber"]
        WS_BCAST["WebSocket Broadcast\n:8765"]
    end

    subgraph Display["Display Layer"]
        CANVAS["JSMpeg\n&lt;canvas&gt;"]
        MAP["MapView\ndrone markers"]
        CARDS["DroneCards\ntelemetry overlay"]
        STORE["Zustand Store\ndrone state"]
    end

    RTSP -->|raw H.264 / RTSP| FFMPEG
    FFMPEG -->|MPEG1 binary chunks| WS_SRV
    WS_SRV -->|WebSocket frames| CANVAS

    DM_OUT -->|JSON UDP packets| UDP_BRIDGE
    UDP_BRIDGE -->|JSON string| WS_BCAST
    WS_BCAST -->|WebSocket message| STORE
    STORE --> MAP
    STORE --> CARDS

    MAV -->|MAVLink| DM_OUT
```

---

## 5. Activity Diagrams

### 5.1 Drone Telemetry Startup

```mermaid
stateDiagram-v2
    [*] --> StartBackend : Launch udp_transporter.py
    StartBackend --> BindUDP : Bind UDP socket (dynamic port)
    BindUDP --> SendSubscribe : Send subscription JSON\nto DM :31659
    SendSubscribe --> WaitResponse : Wait for DM packets

    WaitResponse --> DMOnline : Packet received
    WaitResponse --> RetrySubscribe : Timeout (refresh interval)
    RetrySubscribe --> SendSubscribe

    DMOnline --> BroadcastWS : Forward JSON to\nall WS :8765 clients
    BroadcastWS --> WaitResponse : Continue loop

    note right of WaitResponse
        Subscription refreshes every
        max(1, min(30, duration/2)) seconds
        to keep the stream alive
    end note
```

---

### 5.2 RTSP Stream Pipeline (per drone)

```mermaid
stateDiagram-v2
    [*] --> StartStreamer : Launch StreamerMultiple
    StartStreamer --> OpenWS : Create WebSocket server\non port 9991/9992/9993
    OpenWS --> StartFFmpeg : Spawn FFmpeg process\nRTSP → MPEG1

    StartFFmpeg --> StreamingOK : FFmpeg connects to RTSP
    StartFFmpeg --> FFmpegError : RTSP unreachable

    StreamingOK --> PipeData : Pipe MPEG1 chunks to\nall connected WS clients
    PipeData --> PipeData : continuous data loop

    FFmpegError --> ScheduleRestart : wait 5 s
    ScheduleRestart --> StartFFmpeg : reconnect attempt\n(max 10 retries)

    PipeData --> WatchdogTimeout : No reconnect within 45 s
    WatchdogTimeout --> RestartProcess : spawn new server.js\nexit current process

    note right of PipeData
        Each WS client (browser JSMpeg)
        receives raw MPEG1 transport
        stream binary data
    end note
```

---

### 5.3 DM Connection Check (StreamerMultiple UI)

```mermaid
stateDiagram-v2
    [*] --> Idle : Page loaded
    Idle --> Checking : User clicks "Check DM"\nor Auto mode fires (5 s)

    Checking --> ProbeUDP : GET /api/dm-status\n?host=127.0.0.1&port=31659
    ProbeUDP --> SendPacket : Backend sends UDP subscription\npacket to DM :31659
    SendPacket --> WaitReply : Listen for UDP response

    WaitReply --> Connected : Response received\nwithin 3 s
    WaitReply --> Timeout : No response after 3 s
    WaitReply --> Unreachable : ICMP port unreachable\n(DM not running)

    Connected --> ShowGreen : dot = green\n"DroneManager: connected"
    Timeout --> ShowRed : dot = red\n"offline — no response"
    Unreachable --> ShowRed : dot = red\n"offline — unreachable"

    ShowGreen --> Idle
    ShowRed --> Idle
```

---

### 5.4 Frontend Drone Data Update Cycle

```mermaid
stateDiagram-v2
    [*] --> ConnectWS : App loads\nuseWebSocket hook mounts
    ConnectWS --> OpenSocket : ws://192.168.1.200:8765

    OpenSocket --> WaitMessage : Socket open
    OpenSocket --> Reconnect : Connection failed

    Reconnect --> BackoffWait : exponential backoff
    BackoffWait --> ConnectWS

    WaitMessage --> ParseJSON : WebSocket message received
    ParseJSON --> UpdateStore : Zustand store update\ndrones / missions / timestamp

    UpdateStore --> RerenderMap : MapView reacts\n→ drone marker positions
    UpdateStore --> RerenderCards : DroneCards reacts\n→ battery / armed / mode
    UpdateStore --> CheckBattery : Battery alert check

    CheckBattery --> ShowAlert : battery < threshold
    CheckBattery --> WaitMessage : battery OK

    ShowAlert --> WaitMessage
    RerenderMap --> WaitMessage
    RerenderCards --> WaitMessage
```

---

### 5.5 Mission Execution Flow (Backend)

```mermaid
flowchart TD
    A([Operator issues mission command]) --> B{Mission type}

    B -->|UAM| C[uam.py: UAMission]
    B -->|Formation| D[formations.py: FormationPlugin]
    B -->|Script| E[scripts.py: ScriptPlugin]

    C --> F[Stage: TAKEOFF]
    F --> G[Stage: TRANSIT]
    G --> H[Stage: SEARCH]
    H --> I[Stage: POI_NAV]
    I --> J[Stage: LAND]
    J --> K([Mission complete])

    C --> L[GMP3 path planning\ngmp3generator.py]
    L --> M[DirectSetpointFollower\nsend position setpoints]
    M --> N[Drone executes via MAVSDK]
    N -->|telemetry feedback| M

    F & G & H & I & J -->|stage broadcast| O[External plugin\nudp :31659]
    O --> P[udp_transporter.py]
    P --> Q[Frontend WS :8765]
    Q --> R[UI mission stage display]
```

---

## 6. Network & Port Reference

```mermaid
graph LR
    subgraph LAN["LAN 192.168.1.x"]
        subgraph ControlHost["Control Host · 192.168.1.200"]
            FE["Frontend\n:3000 / :5173"]
            BE["Backend WS\n:8765"]
            SM["StreamerMultiple HTTP\n:3005"]
            WS1[":9991"]
            WS2[":9992"]
            WS3[":9993"]
            DM["DroneManager\nUDP :31659"]
        end

        subgraph DroneNet["Drone Fleet"]
            GR["green · .31\nMAVLink :14561\nRTSP :8900"]
            YL["yellow · .33\nMAVLink :14563\nRTSP :8900"]
            BL["blue · .36\nMAVLink :14566\nRTSP :8900"]
            OT["wedge/tycho/gavin/jaina\n.32/.34/.35/.37"]
        end
    end

    FE -->|WS| BE
    FE -->|WS| WS1
    FE -->|WS| WS2
    FE -->|WS| WS3
    SM --- WS1
    SM --- WS2
    SM --- WS3
    SM -->|UDP probe| DM
    BE -->|UDP sub| DM
    DM -->|MAVLink| GR
    DM -->|MAVLink| YL
    DM -->|MAVLink| BL
    DM -->|MAVLink| OT
    GR -->|RTSP| WS1
    YL -->|RTSP| WS2
    BL -->|RTSP| WS3
```

### Full Port Table

| Port | Protocol | Service | Direction | Purpose |
|------|----------|---------|-----------|---------|
| 3005 | HTTP | StreamerMultiple | inbound | Web UI, `/api/dm-status` |
| 8080 | HTTP + WS | web_server.py | inbound | FastAPI REST + WebSocket API |
| 8765 | WebSocket | udp_transporter.py | inbound | Telemetry broadcast to frontend |
| 8900 | RTSP | Drone cameras | outbound | Camera video source |
| 9991 | WebSocket | StreamerMultiple | inbound | green drone MPEG1 video |
| 9992 | WebSocket | StreamerMultiple | inbound | yellow drone MPEG1 video |
| 9993 | WebSocket | StreamerMultiple | inbound | blue drone MPEG1 video |
| 14561–14567 | UDP | MAVLink / MAVSDK | outbound | Drone flight control |
| 31659 | UDP | DroneManager external plugin | bidirectional | Telemetry subscription |
| 31660 | UDP | web_server.py | inbound | Drone data ingress |
| 50051 | gRPC | MAVSDK server | outbound | Drone SDK interface |

---

## 7. Environment Configuration

### Frontend (`.env` / `.env.production`)

```env
VITE_WS_URL=ws://192.168.1.200:8765
VITE_LAN_HOST=192.168.1.200
```

The frontend auto-detects the hostname from `window.location.hostname`. If it starts with `192.`, it uses that directly; otherwise it falls back to `VITE_LAN_HOST`.

### Backend (`udp_transporter.py`)

| Env Var | Default | Description |
|---|---|---|
| `DM_UDP_HOST` | `127.0.0.1` | DroneManager host |
| `DM_UDP_PORT` | `31659` | DroneManager UDP port |
| `DM_SUB_FREQUENCY` | `5` | Subscription frequency (Hz) |
| `DM_SUB_DURATION` | `60` | Subscription window (seconds) |
| `DM_SUBSCRIBE` | `1` | Enable subscription loop |
| `UDP_LISTEN_PORT` | `0` (dynamic) | Local UDP bind port |

### StreamerMultiple (`server.js`)

| Env Var | Default | Description |
|---|---|---|
| `AUTO_RESTART_ON_STREAM_FAILURE` | `1` | Enable watchdog restart |
| `STREAM_FAILURE_TIMEOUT_MS` | `45000` | Time before forced restart |

---

## 8. Directory Structure

```
UAM25/
├── FRONTEND_UAM/
│   ├── src/
│   │   ├── screens/Element/Element.tsx       ← main page
│   │   ├── components/
│   │   │   ├── map/                          ← MapView, markers, controls
│   │   │   ├── drones/                       ← DroneCard, sidebar
│   │   │   └── ui/                           ← Radix UI primitives
│   │   ├── hooks/useWebSocket.ts             ← WS :8765 connection
│   │   ├── stores/connection-store.tsx       ← Zustand global state
│   │   ├── api/                              ← Axios REST + hooks
│   │   └── data/                             ← droneData, mapData
│   ├── electron/main.js                      ← Electron entry
│   └── .env / .env.production
│
├── backend/backend/fromDM/
│   ├── udp_transporter.py                    ← DM → WS :8765 bridge  ★ main service
│   ├── uam_mock_sender.py                    ← mock multi-drone telemetry
│   ├── SAR-UAM-log.py                        ← static mock telemetry
│   ├── web_server.py                         ← FastAPI :8080 + WS
│   └── src/dronecontrol/
│       ├── app.py                            ← Textual TUI drone manager
│       ├── dronemanager.py
│       ├── drone.py
│       ├── navigation/                       ← GMP3, setpoint, geofence
│       ├── missions/uam.py                   ← UAM mission stages
│       └── plugins/                          ← external, mission, formation
│
├── STREAMERMULTIPLE_UAM/STREAMERMULTIPLE/
│   ├── server.js                             ← Express + RTSP bridge  ★ main service
│   ├── rtsp-stream.js                        ← RtspStream class (FFmpeg + WS)
│   ├── index.html                            ← Streamer UI (JSMpeg + DM panel)
│   ├── jsmpeg.min.js                         ← MPEG1 browser decoder
│   ├── ffmpeg/bin/ffmpeg.exe                 ← FFmpeg binary
│   └── dist/
│       ├── StreamerMultiple.exe              ← pkg-bundled exe
│       ├── index.html                        ← copied by build script
│       └── jsmpeg.min.js                     ← copied by build script
│
└── SYSTEM_DOCUMENTATION.md                  ← this file
```

---

## Quick-Start Reference

### Start telemetry bridge (no drones, mock data)
```bash
cd backend/backend/fromDM
python uam_mock_sender.py   # terminal 1 — sends mock UDP telemetry on :31659
python udp_transporter.py   # terminal 2 — bridges to WS :8765
```

### Start video streamer (development)
```bash
cd STREAMERMULTIPLE_UAM/STREAMERMULTIPLE
npm start
# → http://localhost:3005
```

### Build streamer exe
```bash
npm run build
# → dist/StreamerMultiple.exe + dist/index.html + dist/jsmpeg.min.js
```

### Start frontend (development)
```bash
cd FRONTEND_UAM
npm run dev         # web browser at http://localhost:5173
# or
npm run electron:dev  # Electron desktop window
```
