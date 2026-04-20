const WebSocket = require('ws');
const ffmpeg = require('fluent-ffmpeg');
const EventEmitter = require('events');

class RtspStream extends EventEmitter {
    constructor(options) {
        super();
        this.name = options.name || 'camera';
        this.url = options.streamUrl;
        this.wsPort = options.wsPort;
        this.ffmpegPath = options.ffmpegPath;
        this.ffmpeg = null;
        this.wsServer = null;
        this.clients = new Set();
        this.isConnected = false;
        this.reconnectAttempts = 0;
        this.maxReconnectAttempts = 10;

        ffmpeg.setFfmpegPath(this.ffmpegPath);
    }

    start() {
        if (this.ffmpeg) {
            this.stop();
        }

        // Create WebSocket server
        this.wsServer = new WebSocket.Server({ port: this.wsPort });
        console.log(`[WebSocket] Server started on port ${this.wsPort}`);

        this.wsServer.on('connection', (ws) => {
            this.clients.add(ws);
            console.log(`[${this.name}] Client connected (${this.clients.size} total)`);

            ws.on('close', () => {
                this.clients.delete(ws);
                console.log(`[${this.name}] Client disconnected (${this.clients.size} remaining)`);
            });
        });

        // Start FFmpeg
        this.ffmpeg = ffmpeg(this.url)
            .addInputOption('-rtsp_transport', 'tcp')
            .addInputOption('-re')
            .addInputOption('-fflags', '+genpts')
            .addInputOption('-avoid_negative_ts', 'make_zero')
            .addInputOption('-max_delay', '0')
            .addInputOption('-buffer_size', '1024000')  // Increased buffer size
            .addInputOption('-analyzeduration', '1000000')  // Increased analyze duration
            .addInputOption('-probesize', '1000000')  // Increased probe size
            .addOutputOption('-c:v', 'mpeg1video')
            .addOutputOption('-b:v', '3000k')  // Increased bitrate
            .addOutputOption('-maxrate', '4000k')  // Maximum bitrate
            .addOutputOption('-bufsize', '8000k')  // Buffer size for rate control
            .addOutputOption('-q:v', '2')  // Lower value means better quality (range: 1-31)
            .addOutputOption('-r', '30')  // Frame rate
            .addOutputOption('-s', '1280x720')  // Resolution
            .addOutputOption('-tune', 'zerolatency')
            .addOutputOption('-preset', 'veryfast')  // Fast encoding
            .addOutputOption('-f', 'mpegts')
            .format('mpegts')
            .on('start', (cmd) => {
                console.log(`[${this.name}] FFmpeg started: ${cmd}`);
                this.isConnected = true;
                this.emit('connect');
            })
            .on('error', (err) => {
                console.error(`[${this.name}] FFmpeg error:`, err.message);
                this.isConnected = false;
                this.emit('error', err);
                this.reconnect();
            })
            .on('end', () => {
                console.log(`[${this.name}] FFmpeg stream ended`);
                this.isConnected = false;
                this.emit('close');
                this.reconnect();
            });

        // Pipe FFmpeg output to WebSocket clients
        const ffmpegStream = this.ffmpeg.pipe();
        ffmpegStream.on('data', (data) => {
            this.clients.forEach((client) => {
                if (client.readyState === WebSocket.OPEN) {
                    client.send(data);
                }
            });
        });
    }

    stop() {
        console.log(`[${this.name}] Stopping stream...`);
        if (this.ffmpeg) {
            this.ffmpeg.kill('SIGKILL');
            this.ffmpeg = null;
        }
        if (this.wsServer) {
            this.wsServer.close();
            this.wsServer = null;
        }
        this.isConnected = false;
        this.clients.clear();
    }

    reconnect() {
        if (this.reconnectAttempts < this.maxReconnectAttempts) {
            this.reconnectAttempts++;
            console.log(`[${this.name}] Attempting to reconnect (${this.reconnectAttempts}/${this.maxReconnectAttempts})...`);
            setTimeout(() => {
                this.start();
            }, 5000);
        } else {
            console.log(`[${this.name}] Max reconnection attempts reached`);
            this.stop();
        }
    }

    getStatus() {
        return this.isConnected;
    }
}

module.exports = RtspStream;
