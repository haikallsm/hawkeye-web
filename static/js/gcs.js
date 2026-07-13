class GCSTelemetry {
    constructor() {
        this.data = { 
            lat: 0, lon: 0, alt: 0, roll: 0, pitch: 0, yaw: 0, 
            speed: 0, heading: 0, climb: 0, battery_voltage: 0, 
            battery_current: 0, battery_remaining: 0, system_status: 'disconnected', 
            satellites_visible: 0, gps_fix_type: 0 
        };
        this.onUpdate = null;
    }

    updateFromApi(apiData) {
        if (!apiData) return;
        this.data = { ...this.data, ...apiData };
        if (this.onUpdate) this.onUpdate(this.data);
    }
}

function adaptRadioTelemetry(raw) {
    const RAD2DEG = 180 / Math.PI;
    const att = raw.attitude || {};
    return {
        vehicle_type       : raw.vehicle_type ?? 'UNKNOWN',
        roll                : (att.roll  ?? 0) * RAD2DEG,
        pitch               : (att.pitch ?? 0) * RAD2DEG,
        yaw                 : (att.yaw   ?? 0) * RAD2DEG,
        lat                 : raw.latitude ?? 0,
        lon                 : raw.longitude ?? 0,
        alt                 : raw.altitude ?? 0,
        heading             : raw.heading ?? 0,
        speed               : raw.speed ?? 0,
        climb               : raw.climb ?? 0,
        throttle            : raw.throttle ?? 0,
        satellites_visible  : raw.satellites_visible ?? 0,
        gps_fix_type        : raw.gps_fix_type ?? 0,
        battery_voltage     : raw.battery_voltage ?? 0,
        battery_current     : raw.battery_current ?? 0,
        battery_remaining   : raw.battery_remaining ?? -1,
        is_armed            : raw.armed ?? false,
        arm_mode            : raw.armed ? 'ARMED' : 'DISARMED',
        flight_mode         : raw.mode ?? 'UNKNOWN',
        status              : raw.connected ? 'connected' : 'disconnected',
        
        accel_x: raw.accel_x, accel_y: raw.accel_y, accel_z: raw.accel_z,
        gyro_x: raw.gyro_x, gyro_y: raw.gyro_y, gyro_z: raw.gyro_z,
        mag_field: raw.mag_field,
        
        source              : 'radio',
    };
}
window.adaptRadioTelemetry = adaptRadioTelemetry;

// ===== API Connection Manager =====
const DEFAULT_BACKEND_URL = 'http://127.0.0.1:5000'

class GCSApiClient {
    constructor() {
        this.isConnected = false;
        this.currentConnectionName = null;
        this.telemetry = new GCSTelemetry();
        this.ws = null;
        this.statusInterval = null;
        this.baseUrl = DEFAULT_BACKEND_URL;
        this.onConnect = null;
        this.onDisconnect = null;
        this.onTelemetry = null;
        this.onMavlog = null;
        this.onStatusChange = null;
        this.messagesReceived = 0;
        this.parseErrors = 0;
        this.remoteBackendMode = false;
        this.lastLogSeq = 0;
        this.isFetchingLogs = false;
        this.logHistory = [];
        this.maxLogHistory = 200;
        this.localSerialPort = null;
        this.serialReader = null;
        this.serialWriter = null;
    }

    startWebSocket() {
        if (this.ws) this.ws.disconnect();

        this.ws = io(this.baseUrl);

        this.ws.on('connect', () => {
            console.log('Socket.io Telemetry Terhubung');
        });

        this.ws.on('telemetry', (data) => {
            this.telemetry.updateFromApi(data);
            this.messagesReceived++;
            if (this.onTelemetry) this.onTelemetry(data);
        });

        this.ws.on('mavlog', (logItem) => {
            this.handleIncomingLog(logItem);
        });

        // 3. Pengecekan Gap saat baru konek
        this.ws.on('mavlog_meta', (meta) => {
            if (meta.latest_seq > this.lastLogSeq) {
                this.fetchRecentLogs();
            }
        });

        this.ws.on('disconnect', () => {
            console.log('Socket.io Terputus');
        });
    }

    handleIncomingLog(logItem) {
        if (this.lastLogSeq > 0 && logItem.seq > this.lastLogSeq + 1) {
            this.fetchRecentLogs();
            return; 
        }
        
        if (logItem.seq <= this.lastLogSeq) return;

        this.lastLogSeq = logItem.seq;
        this.logHistory.push(logItem);
        if(this.logHistory.length > this.maxLogHistory) this.logHistory.shift();
        if(this.onMavlog) this.onMavlog(logItem);
    }

    // Gap Recovery: Mengambil log yang tertinggal via REST API
    async fetchRecentLogs() {
        if (this.isFetchingLogs) return;
        this.isFetchingLogs = true;

        try {
            const response = await fetch(`${this.baseUrl}/logs/recent?after=${this.lastLogSeq}&limit=100`);
            const data = await response.json();
            
            if (data.status === 'success' && data.items) {
                data.items.forEach(item => {
                    if (item.seq > this.lastLogSeq) {
                        this.lastLogSeq = item.seq;
                        this.logHistory.push(item);
                        if (this.logHistory.length > this.maxLogHistory) this.logHistory.shift();
                        if (this.onMavlog) this.onMavlog(item);
                    }
                });
            }
        } catch (err) {
            console.error("Gagal menarik log yang tertinggal:", err);
        } finally {
            this.isFetchingLogs = false;
        }
    }

    stopWebSocket() {
        if (this.ws) {
            this.ws.disconnect();
            this.ws = null;
        }
    }

    setBaseUrl(url) {
        this.baseUrl = url || '';
    }

    buildBackendUrl(host, port = '5000') {
        const rawHost = (host || '').trim().replace(/\/+$/, '');
        const rawPort = (port || '5000').toString().trim();
        if (!rawHost) return '';

        if (rawHost.startsWith('http://') || rawHost.startsWith('https://')) {
            const url = new URL(rawHost);
            if (!url.port && rawPort) url.port = rawPort;
            return url.origin;
        }

        if (rawHost.includes(':')) {
            return 'http://' + rawHost;
        }

        return 'http://' + rawHost + ':' + rawPort;
    }

    async fetchBackendStatus(baseUrl) {
        try {
            let resp = await fetch(baseUrl + '/ping', { cache: 'no-store' });
            if (resp.status === 404) {
                resp = await fetch(baseUrl + '/status', { cache: 'no-store' });
            }
            const data = await resp.json();
            return { ok: resp.ok, data };
        } catch (err) {
            return {
                ok: false,
                data: {
                    message: `Tidak bisa menghubungi Raspberry Pi di ${baseUrl}. Pastikan server web di Raspy jalan pada host 0.0.0.0 port 5000, IP/port benar, satu jaringan, dan port tidak diblokir firewall. Detail: ${err.message}`
                }
            };
        }
    }
// Load Port 
    // async loadPorts() {
    //     try {
    //         const resp = await fetch(this.baseUrl + '/ports');
    //         const data = await resp.json();
    //         const select = document.getElementById('portSelect');
    //         if (!select) return;
            
    //         select.innerHTML = '';
    //         if (!data.ports || data.ports.length === 0) {
    //             const opt = document.createElement('option');
    //             opt.text = 'No Port Found';
    //             opt.value = '';
    //             select.appendChild(opt);
    //             return;
    //         }
            
    //         data.ports.forEach((p) => {
    //             const opt = document.createElement('option');
    //             opt.text = p.port + ' — ' + (p.description || 'Unknown');
    //             opt.value = p.port;
    //             select.appendChild(opt);
    //         });
    //         if (data.ports.length > 0) select.value = data.ports[0].port;
    //     } catch (err) {
    //         console.error('Load ports error:', err);
    //     }
    // }
    async loadPorts() {
        if (!('serial' in navigator)) {
            alert('Browser Anda tidak mendukung Web Serial API. Gunakan Chrome atau Edge PC.');
            return;
        }

        try {
            // 1. Munculkan popup bawaan Chrome untuk memilih port laptop
            this.localSerialPort = await navigator.serial.requestPort();
            
            // 2. Ambil informasi port (Chrome membatasi info nama spesifik demi privasi)
            const portInfo = this.localSerialPort.getInfo();
            const vendorId = portInfo.usbVendorId ? portInfo.usbVendorId.toString(16) : 'Unknown';
            
            // 3. Masukkan port yang baru saja Anda pilih ke dalam dropdown UI yang sudah ada
            const select = document.getElementById('portSelect');
            if (!select) return;
            
            select.innerHTML = `<option value="webserial" selected>Radio Laptop (VID: ${vendorId})</option>`;
            
        } catch (err) {
            console.log('Scan port dibatalkan oleh pengguna:', err);
        }
    }

    switchMode() {
        const mode = document.getElementById('modeSelect')?.value;
        const ipInput = document.getElementById('ipInput');
        const portSelect = document.getElementById('portSelect');
        const baudSelect = document.getElementById('baudSelect');
        
        if (!ipInput || !portSelect || !baudSelect) return;
        
        if (mode === 'serial') {
            portSelect.style.display = 'block';
            baudSelect.style.display = 'block';
            ipInput.style.display = 'none';
            this.loadPorts();
        } else {
            portSelect.style.display = 'none';
            baudSelect.style.display = 'none';
            ipInput.style.display = 'block';
        }
    }

    async connect() {
        if (this.isConnected) {
            await this.disconnect();
            return;
        }

        const connTypeSelect = document.getElementById('connTypeSelect');
        const mode = connTypeSelect ? connTypeSelect.value : 'ip';

        if (mode === 'serial') {
            console.warn('[gcsConnection.connect] Mode serial tidak lagi ditangani di sini -- gunakan window.radioMavlink.connect() langsung.');
            return;
        }

        try {
            await this.connectIP();
        } catch (err) {
            console.error('Connect error:', err);
            this.updateUI(false, true);
            alert('Error koneksi: ' + err.message);
        }
    }

    // async connectSerial() {
    //     this.remoteBackendMode = false;
    //     const port = document.getElementById('portSelect')?.value;
    //     const baud = document.getElementById('baudSelect')?.value || '115200';
        
    //     if (!port || port === 'No Port Found' || port === 'Error loading ports') {
    //         alert('Silahkan pilih port terlebih dahulu!'); return;
    //     }

    //     try {
    //         const resp = await fetch(this.baseUrl + '/connect', {
    //             method: 'POST',
    //             headers: { 'Content-Type': 'application/json' },
    //             body: JSON.stringify({ type: 'serial', port: port, baud: parseInt(baud, 10) })
    //         });
    //         const data = await resp.json();
            
    //         if (data.status === 'success' || resp.ok) {
    //             this.currentConnectionName = data.connection_name;
    //             this.isConnected = true;
    //             this.updateUI(true);
    //             this.startWebSocket(); // Jalankan WebSocket
    //             if (this.onConnect) this.onConnect();
    //         } else {
    //             this.updateUI(false, true);
    //             alert('Koneksi SERIAL gagal: ' + (data.message || 'Error tidak diketahui'));
    //         }
    //     } catch (err) {
    //         this.updateUI(false, true);
    //     }
    // }

    // ============================================================
    // DEPRECATED -- digantikan oleh radio-mavlink.js (RadioMavlink class)
    // Pendekatan lama ini coba jembatani port serial laptop ke Pi lewat
    // WebSocket (raw_serial_up/raw_serial_down), lalu minta Pi buka UDP
    // 127.0.0.1:14550 -- TIDAK VALID untuk arsitektur Pi+laptop terpisah,
    // karena 127.0.0.1 di request itu merujuk ke Pi itu sendiri, bukan
    // laptop. Parsing MAVLink untuk radio sekarang 100% di browser lewat
    // RadioMavlink, tidak lagi lewat gcs.js/backend Pi sama sekali.
    // Jangan panggil fungsi ini -- navbar sekarang manggil
    // window.radioMavlink.connect() langsung untuk mode 'serial'.
    // ============================================================
    // async connectSerial() {
    //     this.remoteBackendMode = false;
    //     const portValue = document.getElementById('portSelect')?.value;
    //     const baud = document.getElementById('baudSelect')?.value || '115200';
    //
    //     if (portValue === 'webserial') {
    //         if (!this.localSerialPort) {
    //             alert('Silakan tekan Scan dan pilih port terlebih dahulu!');
    //             return;
    //         }
    //         try {
    //             await this.localSerialPort.open({ baudRate: parseInt(baud, 10) });
    //             this.currentConnectionName = "WebSerial_Bridge";
    //             this.isConnected = true;
    //             this.updateUI(true);
    //             await fetch(this.baseUrl + '/connect', {
    //                 method: 'POST',
    //                 headers: { 'Content-Type': 'application/json' },
    //                 body: JSON.stringify({ type: 'ip', host: '127.0.0.1', port: 14550 })
    //             });
    //             this.startWebSocket();
    //             this.serialReader = this.localSerialPort.readable.getReader();
    //             this.serialWriter = this.localSerialPort.writable.getWriter();
    //             this.readLoop();
    //             this.ws.on('raw_serial_down', (data) => {
    //                 if (this.serialWriter) this.serialWriter.write(new Uint8Array(data));
    //             });
    //             if (this.onConnect) this.onConnect();
    //         } catch (err) {
    //             this.updateUI(false, true);
    //             alert('Gagal membuka port radio laptop: ' + err.message);
    //         }
    //     } else {
    //         alert('Silahkan klik tombol Scan terlebih dahulu untuk mencari port laptop!');
    //     }
    // }
    //
    // async readLoop() {
    //     try {
    //         while (true) {
    //             const { value, done } = await this.serialReader.read();
    //             if (done) { this.serialReader.releaseLock(); break; }
    //             if (this.ws && this.ws.connected) this.ws.emit('raw_serial_up', value);
    //         }
    //     } catch (err) {
    //         console.error("Terjadi putus koneksi pada serial baca:", err);
    //     }
    // }

    async connectIP() {
        const hostInput = document.getElementById('ipHostInput');
        const portInput = document.getElementById('ipPortInput');
        if (!hostInput) {
            alert('Elemen IP Host tidak ditemukan!');
            return;
        }
        let host = hostInput.value.trim();
        let ip_port = portInput ? portInput.value.trim() : '14550';
        if (!host) {
            alert('Silahkan masukkan IP MAVLink (contoh: 192.168.1.100)');
            return;
        }
        // Jika user memasukkan host:port di satu input, pakai itu
        if (host.includes(':')) {
            const parts = host.split(':');
            host = parts[0];
            ip_port = parts[1] || ip_port;
        }
        this.baseUrl = this.baseUrl && this.baseUrl.startsWith('http') ? this.baseUrl : window.location.origin;
        this.remoteBackendMode = false;
        try {
            const connectData = {
                type: 'ip',
                host: host,
                port: parseInt(ip_port, 10) || 14550
            };
            console.log('[connectIP] Sending:', connectData);
            const resp = await fetch(this.baseUrl + '/connect', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify(connectData)
            });
            const data = await resp.json();
            console.log('[connectIP] Response:', data);
            if (data.status === 'success' || resp.ok) {
                this.isConnected = true;
                this.updateUI(true);
                this.startWebSocket();
                if (this.onConnect) this.onConnect();
            } else {
                this.updateUI(false, true);
                alert('Koneksi IP gagal: ' + (data.message || 'Error tidak diketahui'));
            }
        } catch (err) {
            this.updateUI(false, true);
            alert('Error koneksi IP: ' + err.message);
        }
    }

    async disconnect() {
        try {
            this.stopWebSocket(); // Hentikan WebSocket
            
            // if (!this.remoteBackendMode) {
            //     try { await fetch(this.baseUrl + '/disconnect', { method: 'POST' }); } catch (e) {}
            // }

            if (this.serialReader) {
                try { await this.serialReader.cancel(); } catch(e){}
                this.serialReader = null;
            }
            if (this.serialWriter) {
                try { this.serialWriter.releaseLock(); } catch(e){}
                this.serialWriter = null;
            }
            if (this.localSerialPort) {
                try { await this.localSerialPort.close(); } catch(e){}
                this.localSerialPort = null;
            }
            // --- AKHIR PENAMBAHAN ---

            if (!this.remoteBackendMode) {
                try { await fetch(this.baseUrl + '/disconnect', { method: 'POST' }); } catch (e) {}
            }

            this.isConnected = false;
            this.currentConnectionName = null;
            this.remoteBackendMode = false;
            this.messagesReceived = 0;
            this.parseErrors = 0;
            if (typeof this.updateUI === 'function') this.updateUI(false);
            if (this.onDisconnect) this.onDisconnect();
            console.log('Disconnected');
        } catch (err) {
            console.error('Disconnect error:', err);
        }
    }

    updateUI(connected, failed = false) {
        const btn = document.getElementById('connectBtn'); // Sesuaikan dengan id di navbar baru
        const statusText = document.getElementById('statusText');
        
        if (btn) {
            if (connected) {
                btn.innerText = 'Disconnect';
            } else {
                btn.innerText = 'Connect';
            }
        }
        
        if (statusText) {
            if (connected) {
                statusText.innerText = 'Connected';
                statusText.style.color = '#4caf50';
            } else {
                statusText.innerText = failed ? 'Failed' : 'Disconnected';
                statusText.style.color = failed ? '#f44336' : '#888';
            }
        }
        
        if (this.onStatusChange) this.onStatusChange(connected);
    }

    restoreNavbarStatus() {
        try {
            const stored = sessionStorage.getItem('hawkeye_navbar_status');
            if (stored) {
                const status = JSON.parse(stored);
                this.updateUI(status.connected, status.failed);
            }
        } catch (e) {
            // Silently ignore
        }
    }

    async sendCommand(cmd, alt = 10) {
        try {
            const resp = await fetch(this.baseUrl + '/command', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ cmd, alt })
            });
            return await resp.json();
        } catch (err) {
            return { status: 'failed', message: err.message };
        }
    }
}

// ===== PERSISTENT CONNECTION MANAGER =====
class PersistentConnectionManager {
    constructor() {
        this.client = null;
        this.reconnectAttempts = 0;
        this.maxReconnectAttempts = 5;
        this.reconnectDelay = 2000;
    }

    init() {
        if (!this.client) {
            this.client = new GCSApiClient();
            
            // Handle automatic reconnection
            this.client.onDisconnect = () => {
                console.log('Connection lost, attempting to reconnect...');
                this.attemptReconnect();
            };
            
            this.client.onConnect = () => {
                this.reconnectAttempts = 0;
                console.log('Successfully reconnected');
            };
        }
        return this.client;
    }

    attemptReconnect() {
        if (this.reconnectAttempts >= this.maxReconnectAttempts) {
            console.error('Max reconnection attempts reached');
            return;
        }

        this.reconnectAttempts++;
        const delay = this.reconnectDelay * this.reconnectAttempts;
        
        console.log(`Reconnection attempt ${this.reconnectAttempts}/${this.maxReconnectAttempts} in ${delay}ms`);
        
        setTimeout(() => {
            if (this.client) {
                this.client.connect().catch(err => {
                    console.error('Reconnect failed:', err);
                    this.attemptReconnect();
                });
            }
        }, delay);
    }
}

// ===== GLOBAL PERSISTENT CONNECTION MANAGER =====
const persistentConnectionMgr = new PersistentConnectionManager();

// ===== Initialize Global GCS Connection =====
window.gcsConnection = new GCSApiClient();

document.addEventListener('DOMContentLoaded', async () => {
    console.log('Initializing HAWKEYE GCS Master...');
    
    // Persiapan awal UI
    window.gcsConnection.loadPorts();
    if (typeof window.gcsConnection.switchMode === 'function') {
        window.gcsConnection.switchMode();
    }
    window.gcsConnection.restoreNavbarStatus();
    
    // PERBAIKAN 1: Routing data telemetri ke dalam Iframe
    window.gcsConnection.onTelemetry = (data) => {
        const frame = document.getElementById('main-frame');
        if (frame && frame.contentWindow && typeof frame.contentWindow.updateTelemetryUI === 'function') {
            frame.contentWindow.updateTelemetryUI(data);
        }
    };

    window.gcsConnection.onMavlog = (logItem) => {
        const frame = document.getElementById('main-frame');
        if (frame && frame.contentWindow && typeof frame.contentWindow.updateMavlogUI === 'function') {
            frame.contentWindow.updateMavlogUI(logItem);
        }
    };
    // CEK STATUS SAAT WEB DIBUKA (TANPA MEMAKSA KONEK)
    try {
        const res = await fetch(window.gcsConnection.baseUrl + '/status');
        const data = await res.json();
        
        if (data.connected) {
            window.gcsConnection.isConnected = true;
            window.gcsConnection.updateUI(true);
            
            // PERBAIKAN 2: Pastikan WebSocket dipanggil saat auto-reconnect
            window.gcsConnection.startWebSocket(); 
            
            console.log("FC terdeteksi sudah terkoneksi sebelumnya. Melanjutkan sesi...");
        } else {
            window.gcsConnection.updateUI(false);
            console.log("Menunggu instruksi koneksi manual dari pengguna...");
        }
    } catch (err) {
        console.error("Gagal memeriksa status awal dari backend:", err);
    }
});