# Web Plugin Interfaces
 
Ground-control web UI for monitoring and coordinating a fleet of autonomous drones. Displays real-time telemetry on a map, live video feeds from drone cameras, and drone status cards.
 
> For full architecture details, data flows, network/port reference, and environment configuration see **[SYSTEM_DOCUMENTATION.md](SYSTEM_DOCUMENTATION.md)**.
 
---
 
## Stack
 
| Component | Technology | Purpose |
|---|---|---|
| **Frontend** (`FRONTEND_CLIENT_UAM/`) | React 18 + TypeScript + Electron | Map UI, drone cards, video feeds |
| **Streamer** (`STREAMERMULTIPLE_UAM/`) | Node.js + FFmpeg + JSMpeg | RTSP → WebSocket MPEG1 video pipeline |
| **Backend / Transporter** (`TRANSPORTER/`) | Python + FastAPI + MAVSDK | Telemetry bridge from DroneManager → WebSocket | REST API Swagger documentation is coming soon |
 
The system connects to an external **DroneManager** application that manages MAVLink communication with the physical drones.
 
---
 
## Prerequisites
 
- **Node.js** 18+
- **Python** 3.10+
- **FFmpeg** (bundled inside `StreamerMultiple.exe` for Windows)
- **DroneManager** running and accessible on UDP port `31659`
 
---
 
## Pre-built Binaries (Windows x64)
 
Large build artifacts are not stored in this repository. Download them from Google Drive:
 
**[Download pre-built binaries (Google Drive)](https://drive.google.com/drive/folders/19vjkobY_BCfNCf_sFnyT8JdSH7wSAM1e?usp=sharing)**
 
| File | Place it at |
|---|---|
| `StreamerMultiple.exe` | `STREAMERMULTIPLE_UAM/dist/StreamerMultiple.exe` |
| `dist-electron/` folder | `FRONTEND_CLIENT_UAM/dist-electron/` |
 
---
 
## Installation & Setup
 
### 1. Frontend
 
**Option A — Run the pre-built Electron app**
 
Download the `dist-electron/` folder from the link above, place it under `FRONTEND_CLIENT_UAM/`, then launch:
 
```
FRONTEND_CLIENT_UAM/dist-electron/win-unpacked/UAM Control Hub.exe
```
 
**Option B — Build from source**
 
```bash
cd FRONTEND_CLIENT_UAM
npm install
```
 
Configure the backend host in `.env`:
 
```env
VITE_WS_URL=ws://<control-host>:8765
VITE_LAN_HOST=<control-host>
```
 
Run in browser:
```bash
npm run dev
# → http://localhost:5173
```
 
Run as Electron desktop app:
```bash
npm run electron:dev
```
 
Build Electron installer:
```bash
npm run electron:build
# → dist-electron/win-unpacked/UAM Control Hub.exe
```
 
---
 
### 2. Video Streamer
 
**Option A — Use the pre-built exe (recommended)**
 
Download `StreamerMultiple.exe` from the link above and place it in `STREAMERMULTIPLE_UAM/dist/`:
 
```
STREAMERMULTIPLE_UAM/
└── dist/
    ├── StreamerMultiple.exe   ← place here
    ├── index.html
    └── jsmpeg.min.js
```
 
Run it:
```
STREAMERMULTIPLE_UAM/dist/StreamerMultiple.exe
# → http://localhost:3005
```
 
**Option B — Run from source**
 
```bash
cd STREAMERMULTIPLE_UAM
npm install
npm start
# → http://localhost:3005
```
 
Each drone's RTSP camera stream is exposed as a WebSocket:
 
| Drone | RTSP source | WebSocket out |
|---|---|---|
| green | `rtsp://192.168.1.31:8900/live` | `ws://<host>:9991` |
| yellow | `rtsp://192.168.1.33:8900/live` | `ws://<host>:9992` |
| blue | `rtsp://192.168.1.36:8900/live` | `ws://<host>:9993` |
 
---
 
### 3. Telemetry Transporter (Backend)
 
```bash
cd TRANSPORTER
python udp_transporter.py
# → WebSocket server on :8765
```
 
To test without real drones, run the mock sender first:
 
```bash
python uam_mock_sender.py   # terminal 1 — sends mock telemetry on UDP :31659
python udp_transporter.py   # terminal 2 — bridges to WS :8765
```
 
---
 
## Quick Port Reference
 
| Port | Service |
|---|---|
| `5173` | Frontend dev server |
| `8765` | Telemetry WebSocket (transporter → frontend) |
| `3005` | Streamer HTTP UI |
| `9991–9993` | Video WebSocket per drone |
| `31659` | DroneManager UDP (external plugin) |
 
---
 
## License
 
See individual component directories for license information. This web plugin developed by HMT TTZ THI. 
 
 
