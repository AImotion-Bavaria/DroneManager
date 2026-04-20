const express = require('express');
const path = require('path');
const fs = require('fs');
const { spawn } = require('child_process');
const dgram = require('dgram');
const RtspStream = require('./rtsp-stream');

// In pkg, __dirname for the entry script is the exe's real directory.
// All external files (ffmpeg, html, js) must sit next to the exe.
const isPkg = typeof process.pkg !== 'undefined';
const baseDir = isPkg ? path.dirname(process.execPath) : __dirname;

const ffmpegPath = path.join(baseDir, 'ffmpeg', 'bin', 'ffmpeg.exe');

// Read static assets from baseDir:
//   pkg mode → directory containing the exe — files copied there by build script
//   dev mode → __dirname (source folder)
function readAsset(filename) {
    return fs.readFileSync(path.join(baseDir, filename));
}
console.log('Using FFmpeg path:', ffmpegPath);

const app = express();
const port = 3005;
const AUTO_RESTART_ON_STREAM_FAILURE = process.env.AUTO_RESTART_ON_STREAM_FAILURE !== '0';
const STREAM_FAILURE_TIMEOUT_MS = Number(process.env.STREAM_FAILURE_TIMEOUT_MS || 45000);
let isServerRestarting = false;
const streamFailureTimers = new Map();

function restartServer(reason) {
    if (!AUTO_RESTART_ON_STREAM_FAILURE || isServerRestarting) {
        return;
    }
    isServerRestarting = true;
    console.log(`[Watchdog] Restarting server due to: ${reason}`);
    const child = spawn(process.execPath, [path.join(__dirname, 'server.js')], {
        cwd: __dirname,
        detached: true,
        stdio: 'inherit',
    });
    child.unref();
    setTimeout(() => { process.exit(1); }, 1000);
}

// Serve static assets — readAsset works for both pkg exe and plain node
app.get('/', (_req, res) => {
    res.setHeader('Content-Type', 'text/html; charset=utf-8');
    res.end(readAsset('index.html'));
});
app.get('/jsmpeg.min.js', (_req, res) => {
    res.setHeader('Content-Type', 'application/javascript');
    res.end(readAsset('jsmpeg.min.js'));
});

// --- DroneManager connection probe ---
// Sends a UDP subscription request to DM's external plugin (default 127.0.0.1:31659)
// and waits up to 3 seconds for a response. Returns { connected: bool }.
app.get('/api/dm-status', (req, res) => {
    const dmHost = req.query.host || '127.0.0.1';
    const dmPort = parseInt(req.query.port || '31659', 10);
    const timeout = 3000;

    const socket = dgram.createSocket('udp4');
    let done = false;

    function finish(connected, error) {
        if (done) return;
        done = true;
        try { socket.close(); } catch (_) {}
        res.json({ connected, host: dmHost, port: dmPort, ...(error ? { error } : {}) });
    }

    socket.on('message', () => finish(true));
    socket.on('error', (err) => finish(false, err.message));

    const payload = Buffer.from(JSON.stringify({ frequency: 1, duration: 2 }));

    socket.bind(() => {
        socket.send(payload, dmPort, dmHost, (err) => {
            if (err) finish(false, err.message);
        });
    });

    setTimeout(() => finish(false, 'timeout'), timeout);
});

console.log('Flow: [FFmpeg → MPEG1] → [WebSocket] → [JSMPEG in browser]');

// Configure RTSP streams
const streams = [
    new RtspStream({ name: 'green',  streamUrl: 'rtsp://192.168.1.31:8900/live', wsPort: 9991, ffmpegPath }),
    new RtspStream({ name: 'yellow', streamUrl: 'rtsp://192.168.1.33:8900/live', wsPort: 9992, ffmpegPath }),
    new RtspStream({ name: 'blue',   streamUrl: 'rtsp://192.168.1.36:8900/live', wsPort: 9993, ffmpegPath }),
];

// Start all streams
streams.forEach(stream => {
    stream.start();
    let isConnected = false;
    setInterval(() => {
        const current = stream.getStatus();
        if (current !== isConnected) {
            isConnected = current;
            if (isConnected) {
                if (streamFailureTimers.has(stream.name)) {
                    clearTimeout(streamFailureTimers.get(stream.name));
                    streamFailureTimers.delete(stream.name);
                }
                console.log(`✅ ${stream.name}: RTSP stream connected`);
            } else {
                console.log(`❌ ${stream.name}: RTSP stream disconnected`);
                if (!streamFailureTimers.has(stream.name)) {
                    const timer = setTimeout(() => {
                        restartServer(`${stream.name} disconnected for ${STREAM_FAILURE_TIMEOUT_MS}ms`);
                    }, STREAM_FAILURE_TIMEOUT_MS);
                    streamFailureTimers.set(stream.name, timer);
                }
            }
        }
    }, 5000);
});

process.on('SIGINT', () => {
    console.log('Shutting down streams...');
    streamFailureTimers.forEach((timer) => clearTimeout(timer));
    streamFailureTimers.clear();
    streams.forEach(stream => stream.stop());
    process.exit();
});

process.on('uncaughtException', (err) => {
    console.error('Uncaught exception:', err);
});

const os = require('os');
const interfaces = os.networkInterfaces();
const lanIp = interfaces.Ethernet?.find(i => i.family === 'IPv4')?.address || 'localhost';

app.listen(port, () => {
    console.log(`Server running at http://localhost:${port}`);
    console.log('Stream URLs:');
    streams.forEach(stream => {
        console.log(`  ${stream.name}: ws://${lanIp}:${stream.wsPort}`);
    });
    console.log(`DM status API: http://localhost:${port}/api/dm-status`);
});
