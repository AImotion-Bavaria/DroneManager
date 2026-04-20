# Web Plugin Interfaces
 
Ground-control web UI for monitoring and coordinating a fleet of autonomous drones. Displays real-time telemetry on a map, live video feeds from drone cameras, and drone status cards.
 
> For full architecture details, data flows, network/port reference, and environment configuration see **[SYSTEM_DOCUMENTATION.md](SYSTEM_DOCUMENTATION.md)**.
 
---
 
## Stack
 
| Component | Technology | Purpose |
|---|---|---|
| **Frontend** (`FRONTEND_CLIENT_UAM/`) | React 18 + TypeScript + Electron | Map UI, drone cards, video feeds |
| **Streamer** (`STREAMERMULTIPLE_UAM/`) | Node.js + FFmpeg + JSMpeg | RTSP → WebSocket MPEG1 video pipeline |
| **Backend / Transporter** (`TRANSPORTER/`) | Python + FastAPI + MAVSDK | Telemetry bridge from DroneManager → WebSocket | Coming soon REST API documentation
 
The system connects to an external **DroneManager** application that manages MAVLink communication with the physical drones.
 
---
 
## Prerequisites
 
- **Node.js** 18+
- **Python** 3.10+
- **FFmpeg** (bundled in `STREAMERMULTIPLE_UAM/` for Windows)
- **DroneManager** running and accessible on UDP port `31659`
 
---
 
## Installation & Setup
 
### 1. Frontend
 
```bash
cd FRONTEND_CLIENT_UAM
npm install
```
 
Configure the backend host by editing `.env` (or `.env.production` for built output):
 
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
 
---
 
### 2. Video Streamer
 
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
pip install -r requirements.txt   # if applicable
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
 
See individual component directories for license information.
 
