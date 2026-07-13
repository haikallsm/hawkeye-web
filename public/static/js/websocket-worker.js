// static/js/websocket-worker.js
// SharedWorker untuk menjaga koneksi WebSocket tetap hidup antar halaman

// Impor socket.io client (dari file lokal yang sudah diunduh)
importScripts('/static/js/socket.io.min.js');

let socket = null;
let wsConnected = false;
const clients = [];

// Fungsi untuk broadcast ke semua klien
function broadcast(message) {
    // Hapus port yang sudah tidak valid
    for (let i = clients.length - 1; i >= 0; i--) {
        try {
            clients[i].postMessage(message);
        } catch (e) {
            clients.splice(i, 1);
        }
    }
}

// Fungsi koneksi ke WebSocket
function connect(baseUrl) {
    if (socket && socket.connected) {
        // Kirim status connected ke semua klien
        broadcast({ type: 'status', status: 'connected' });
        return;
    }

    const wsUrl = baseUrl.replace(/^http/, 'ws');

    try {
        socket = io(wsUrl, {
            transports: ['websocket'],
            reconnection: true,
            reconnectionAttempts: 10,
            reconnectionDelay: 1000,
        });

        socket.on('connect', () => {
            wsConnected = true;
            console.log('[Worker] WebSocket connected');
            broadcast({ type: 'status', status: 'connected' });
        });

        socket.on('telemetry_update', (data) => {
            broadcast({ type: 'telemetry', data });
        });

        socket.on('disconnect', () => {
            wsConnected = false;
            console.log('[Worker] WebSocket disconnected');
            broadcast({ type: 'status', status: 'disconnected' });
        });

        socket.on('connect_error', (err) => {
            console.warn('[Worker] Connection error:', err.message);
        });

    } catch (err) {
        console.error('[Worker] Failed to connect:', err);
        broadcast({ type: 'status', status: 'error', message: err.message });
    }
}

// Event saat ada halaman yang terhubung ke worker
self.addEventListener('connect', (event) => {
    const port = event.ports[0];
    clients.push(port);

    port.addEventListener('message', (msg) => {
        const { type, payload } = msg.data;

        if (type === 'connect') {
            // Kirim baseUrl dari main thread
            const baseUrl = payload?.baseUrl || 'http://localhost:5000';
            connect(baseUrl);
        } else if (type === 'disconnect') {
            if (socket) {
                socket.disconnect();
                socket = null;
                wsConnected = false;
                broadcast({ type: 'status', status: 'disconnected' });
            }
        } else if (type === 'getStatus') {
            port.postMessage({
                type: 'status',
                status: wsConnected ? 'connected' : 'disconnected'
            });
        }
    });

    port.start();

    // Kirim status awal ke klien baru
    port.postMessage({
        type: 'status',
        status: wsConnected ? 'connected' : 'disconnected'
    });
});

// Bersihkan saat worker di-terminate
self.addEventListener('close', () => {
    if (socket) {
        socket.disconnect();
        socket = null;
    }
});

console.log('[Worker] SharedWorker started');