const { app, BrowserWindow } = require('electron');
const path = require('path');
const dgram = require('dgram');
const net = require('net');
const { spawn } = require('child_process');

let mainWindow;
let transporterProcess = null;
let transporterStarting = false;

const DM_HOST = process.env.VITE_LAN_HOST || '192.168.1.200';
const DM_PORT = 31659;
const TRANSPORTER_SCRIPT = path.join(
  'C:\\Users\\ttzhm\\Documents\\Workspace\\Control-Hub\\UAM25\\backend\\backend\\fromDM',
  'udp_transporter.py'
);
const WS_PORT = 8765;

function checkDMStatus() {
  return new Promise((resolve) => {
    const socket = dgram.createSocket('udp4');
    let done = false;
    function finish(connected) {
      if (done) return;
      done = true;
      try { socket.close(); } catch (_) {}
      resolve(connected);
    }
    socket.on('message', () => finish(true));
    socket.on('error', () => finish(false));
    const payload = Buffer.from(JSON.stringify({ frequency: 1, duration: 2 }));
    socket.bind(() => {
      socket.send(payload, DM_PORT, DM_HOST, (err) => { if (err) finish(false); });
      setTimeout(() => finish(false), 3000);
    });
  });
}

function isTransporterRunning() {
  return new Promise((resolve) => {
    const socket = new net.Socket();
    socket.setTimeout(500);
    socket.on('connect', () => { socket.destroy(); resolve(true); });
    socket.on('timeout', () => { socket.destroy(); resolve(false); });
    socket.on('error', () => { socket.destroy(); resolve(false); });
    socket.connect(WS_PORT, '127.0.0.1');
  });
}

function startTransporter() {
  if (transporterProcess || transporterStarting) return;
  transporterStarting = true;
  console.log('[Transporter] Starting udp_transporter.py ...');
  transporterProcess = spawn('python', [TRANSPORTER_SCRIPT], {
    env: { ...process.env, DM_UDP_HOST: DM_HOST, DM_UDP_PORT: String(DM_PORT), DM_SUB_FREQUENCY: '5', DM_SUB_DURATION: '60' },
    stdio: ['ignore', 'pipe', 'pipe'],
  });
  transporterProcess.stdout.on('data', (d) => console.log('[Transporter]', d.toString().trim()));
  transporterProcess.stderr.on('data', (d) => console.error('[Transporter ERR]', d.toString().trim()));
  transporterProcess.on('exit', (code) => { console.log(`[Transporter] exited (${code})`); transporterProcess = null; transporterStarting = false; });
  transporterProcess.on('error', (err) => { console.error('[Transporter] Failed:', err.message); transporterProcess = null; transporterStarting = false; });
  transporterStarting = false;
}

function stopTransporter() {
  if (!transporterProcess) return;
  console.log('[Transporter] Stopping...');
  transporterProcess.kill();
  transporterProcess = null;
}

function createWindow() {
  mainWindow = new BrowserWindow({
    width: 1440, height: 900, minWidth: 1024, minHeight: 600,
    webPreferences: {
      preload: path.join(__dirname, 'preload.cjs'),
      contextIsolation: true,
      nodeIntegration: false,
    },
    title: 'UAM Control Hub',
    backgroundColor: '#353535',
  });
  mainWindow.loadFile(path.join(__dirname, '../dist/index.html'));

  async function pollDMStatus() {
    const dmUp = await checkDMStatus();
    if (dmUp) {
      const wsUp = await isTransporterRunning();
      if (!wsUp) startTransporter();
    } else {
      stopTransporter();
    }
    if (mainWindow && !mainWindow.isDestroyed()) {
      mainWindow.webContents.send('dm-status', dmUp);
    }
  }

  mainWindow.webContents.on('did-finish-load', () => pollDMStatus());
  const dmPollInterval = setInterval(pollDMStatus, 5000);
  mainWindow.on('closed', () => { clearInterval(dmPollInterval); stopTransporter(); mainWindow = null; });
}

app.whenReady().then(createWindow);
app.on('window-all-closed', () => { stopTransporter(); if (process.platform !== 'darwin') app.quit(); });
app.on('activate', () => { if (mainWindow === null) createWindow(); });
