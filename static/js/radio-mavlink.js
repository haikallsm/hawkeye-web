function mavlinkCrc16Update(crc, byte) {
    let tmp = byte ^ (crc & 0xFF);
    tmp = (tmp ^ (tmp << 4)) & 0xFF;
    return ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF;
}

const CRC_EXTRA = {
    0: 50,    // HEARTBEAT
    1: 124,   // SYS_STATUS
    24: 24,   // GPS_RAW_INT
    30: 39,   // ATTITUDE
    33: 104,  // GLOBAL_POSITION_INT
    66: 148,  // REQUEST_DATA_STREAM (dipakai buat KIRIM, bukan cuma terima)
    74: 20,   // VFR_HUD
    253: 83,  // STATUSTEXT
};

const REQUESTED_STREAMS = [1, 2, 3, 6, 10, 11, 12]; // RAW_SENSORS, EXTENDED_STATUS, RC_CHANNELS, POSITION, EXTRA1, EXTRA2, EXTRA3
const STREAM_RATE_HZ = 10; // samain dengan STREAM_RATE_HZ di mavlink_core.py

function buildMavlink1Frame(seq, sysid, compid, msgId, payloadBytes, crcExtra) {
    const len = payloadBytes.length;
    // Urutan sebelum CRC: STX, len, seq, sysid, compid, msgid, payload
    const beforeCrc = [len, seq & 0xFF, sysid, compid, msgId, ...payloadBytes];

    let crc = 0xFFFF;
    for (const b of beforeCrc) crc = mavlinkCrc16Update(crc, b);
    crc = mavlinkCrc16Update(crc, crcExtra);

    return new Uint8Array([0xFE, ...beforeCrc, crc & 0xFF, (crc >> 8) & 0xFF]);
}

const PARSE_STATE = {
    IDLE: 0, GOT_STX: 1, GOT_LENGTH: 2, GOT_INCOMPAT: 3, GOT_COMPAT: 4,
    GOT_SEQ: 5, GOT_SYSID: 6, GOT_COMPID: 7, GOT_MSGID: 8, GOT_PAYLOAD: 9,
};

class MavlinkFrameParser {
    constructor(onFrame) {
        this.onFrame = onFrame; // callback(msgId, payloadBytes)
        this._reset();
    }

    _reset() {
        this.state = PARSE_STATE.IDLE;
        this.isV2 = false;
        this.buf = [];
        this.payloadLen = 0;
        this.msgId = 0;
        this.crcAccum = 0xFFFF;
        this.headerLen = 0; // 6 (v1) atau 10 (v2), dihitung setelah tahu versi
    }

    // Panggil ini per byte yang datang dari port.readable
    feedByte(byte) {
        switch (this.state) {
            case PARSE_STATE.IDLE:
                if (byte === 0xFE) { this.isV2 = false; this._startFrame(); }
                else if (byte === 0xFD) { this.isV2 = true; this._startFrame(); }
                break;

            case PARSE_STATE.GOT_STX:
                this.payloadLen = byte;
                this.buf.push(byte);
                this.state = PARSE_STATE.GOT_LENGTH;
                if (this.isV2) {
                    // v2 punya field incompat_flags & compat_flags sebelum seq
                } else {
                    this.state = PARSE_STATE.GOT_SEQ === undefined ? this.state : PARSE_STATE.GOT_COMPAT;
                    // v1 loncat langsung ke SEQ (tidak ada incompat/compat flags)
                }
                break;

            case PARSE_STATE.GOT_LENGTH:
                if (this.isV2) {
                    this.buf.push(byte); // incompat_flags
                    this.state = PARSE_STATE.GOT_INCOMPAT;
                } else {
                    this.buf.push(byte); // seq (v1)
                    this.state = PARSE_STATE.GOT_SEQ;
                }
                break;

            case PARSE_STATE.GOT_INCOMPAT: // hanya v2
                this.buf.push(byte); // compat_flags
                this.state = PARSE_STATE.GOT_COMPAT;
                break;

            case PARSE_STATE.GOT_COMPAT: // hanya v2
                this.buf.push(byte); // seq
                this.state = PARSE_STATE.GOT_SEQ;
                break;

            case PARSE_STATE.GOT_SEQ:
                this.buf.push(byte); // sysid
                this._sysid = byte;  // simpan terpisah -- dibutuhkan buat REQUEST_DATA_STREAM (target_system)
                this.state = PARSE_STATE.GOT_SYSID;
                break;

            case PARSE_STATE.GOT_SYSID:
                this.buf.push(byte); // compid
                this._compid = byte; // simpan terpisah -- dibutuhkan buat REQUEST_DATA_STREAM (target_component)
                this.state = PARSE_STATE.GOT_COMPID;
                break;

            case PARSE_STATE.GOT_COMPID:
                this.buf.push(byte);
                if (this.isV2) {
                    this.msgIdBytes = [byte];
                    this._msgIdByteCount = 1;
                    this.state = PARSE_STATE.GOT_MSGID; // butuh 3 byte msgid di v2
                } else {
                    this.msgId = byte; // v1 msgid cuma 1 byte
                    this.state = PARSE_STATE.GOT_MSGID;
                    this._payloadStart = this.buf.length;
                }
                break;

            case PARSE_STATE.GOT_MSGID:
                if (this.isV2 && this._msgIdByteCount < 3) {
                    this.buf.push(byte);
                    this.msgIdBytes.push(byte);
                    this._msgIdByteCount++;
                    if (this._msgIdByteCount === 3) {
                        this.msgId = this.msgIdBytes[0] | (this.msgIdBytes[1] << 8) | (this.msgIdBytes[2] << 16);
                        this._payloadStart = this.buf.length;
                    }
                    break;
                }
                // payload byte
                this.buf.push(byte);
                if (this.buf.length - this._payloadStart >= this.payloadLen) {
                    this.state = PARSE_STATE.GOT_PAYLOAD;
                    this._crcBytesNeeded = 2;
                    this._crcBytes = [];
                }
                break;

            case PARSE_STATE.GOT_PAYLOAD:
                this._crcBytes.push(byte);
                if (this._crcBytes.length === 2) {
                    this._finishFrame();
                    this._reset();
                }
                break;
        }
    }

    _startFrame() {
        this.buf = [];
        this.state = PARSE_STATE.GOT_STX;
    }

    _finishFrame() {
        const payload = this.buf.slice(this._payloadStart, this._payloadStart + this.payloadLen);
        const crcExtra = CRC_EXTRA[this.msgId];
        if (crcExtra === undefined) return; // pesan tidak kita dukung, skip diam-diam

        // Hitung CRC atas: length..payload (semua byte SETELAH STX, TANPA STX itu sendiri)
        // lalu tambahkan CRC_EXTRA di akhir -- ini aturan resmi MAVLink.
        let crc = 0xFFFF;
        for (const b of this.buf) crc = mavlinkCrc16Update(crc, b);
        crc = mavlinkCrc16Update(crc, crcExtra);

        const receivedCrc = this._crcBytes[0] | (this._crcBytes[1] << 8);
        if (crc !== receivedCrc) return; // CRC gagal, frame korup/salah sync, buang

        this.onFrame(this.msgId, payload, this._sysid, this._compid);
    }
}

function readF32(dv, off) { return dv.getFloat32(off, true); }
function readU32(dv, off) { return dv.getUint32(off, true); }
function readI32(dv, off) { return dv.getInt32(off, true); }
function readU16(dv, off) { return dv.getUint16(off, true); }
function readI16(dv, off) { return dv.getInt16(off, true); }

const MAV_TYPE_ROVER = 10;
const MAV_TYPE_COPTER_SET = new Set([2, 13, 14]); // QUADROTOR, HEXAROTOR-ish umum ArduCopter
const COPTER_MODE_MAP = { 0: 'STABILIZE', 2: 'ALT_HOLD', 3: 'AUTO', 4: 'GUIDED', 5: 'LOITER', 6: 'RTL', 9: 'LAND', 16: 'POSHOLD', 20: 'GUIDED_NOGPS' };
const ROVER_MODE_MAP = { 0: 'MANUAL', 3: 'STEERING', 4: 'HOLD', 5: 'LOITER', 10: 'AUTO', 11: 'RTL', 15: 'GUIDED' };

const DECODERS = {
    // HEARTBEAT (id=0) wire order: custom_mode(u32), type(u8), autopilot(u8), base_mode(u8), system_status(u8), mavlink_version(u8)
    0: (bytes) => {
        const dv = new DataView(new Uint8Array(bytes).buffer);
        const custom_mode = readU32(dv, 0);
        const type = dv.getUint8(4);
        const base_mode = dv.getUint8(6);
        const armed = (base_mode & 0x80) !== 0; // MAV_MODE_FLAG_SAFETY_ARMED
        const vehicle_type = type === MAV_TYPE_ROVER ? 'ROVER' : (MAV_TYPE_COPTER_SET.has(type) ? 'COPTER' : 'UNKNOWN');
        const modeMap = vehicle_type === 'ROVER' ? ROVER_MODE_MAP : COPTER_MODE_MAP;
        return {
            _kind: 'telemetry',
            armed,
            vehicle_type,
            mode: modeMap[custom_mode] || `MODE_${custom_mode}`,
        };
    },

    // SYS_STATUS (id=1) offset voltage_battery=14(u16 mV), battery_remaining=30(i8 %)
    1: (bytes) => {
        const dv = new DataView(new Uint8Array(bytes).buffer);
        const voltage_mv = readU16(dv, 14);
        const battery_remaining = dv.getInt8(30);
        return {
            _kind: 'telemetry',
            battery_voltage: voltage_mv / 1000.0,
            battery_remaining,
        };
    },

    // ATTITUDE (id=30) order: time_boot_ms(u32), roll(f), pitch(f), yaw(f), ...
    30: (bytes) => {
        const dv = new DataView(new Uint8Array(bytes).buffer);
        return {
            _kind: 'telemetry',
            attitude: {
                roll: readF32(dv, 4),
                pitch: readF32(dv, 8),
                yaw: readF32(dv, 12),
            },
        };
    },

    // GLOBAL_POSITION_INT (id=33) order: time_boot_ms(u32), lat(i32,1e7), lon(i32,1e7), alt(i32,mm), relative_alt(i32,mm), vx,vy,vz(i16), hdg(u16, cdeg)
    33: (bytes) => {
        const dv = new DataView(new Uint8Array(bytes).buffer);
        return {
            _kind: 'telemetry',
            latitude: readI32(dv, 4) / 1e7,
            longitude: readI32(dv, 8) / 1e7,
            altitude: readI32(dv, 16) / 1000.0, // pakai relative_alt (di atas home), bukan alt MSL
            heading: readU16(dv, 26) / 100.0,
        };
    },

    // VFR_HUD (id=74) order: airspeed(f), groundspeed(f), alt(f), climb(f), heading(i16), throttle(u16)
    74: (bytes) => {
        const dv = new DataView(new Uint8Array(bytes).buffer);
        return {
            _kind: 'telemetry',
            speed: readF32(dv, 4), // groundspeed
            climb: readF32(dv, 12),
        };
    },

    // GPS_RAW_INT (id=24) order: time_usec(u64,8), lat,lon,alt(i32 x3), eph,epv,vel,cog(u16 x4), fix_type(u8), satellites_visible(u8)
    24: (bytes) => {
        const dv = new DataView(new Uint8Array(bytes).buffer);
        return {
            _kind: 'telemetry',
            gps_fix_type: dv.getUint8(28),
            satellites_visible: dv.getUint8(29),
        };
    },

    // STATUSTEXT (id=253) order: severity(u8), text(char[50]), [id(u16), chunk_seq(u8)] -- ekstensi v2, opsional
    253: (bytes) => {
        const severity = bytes[0];
        let end = 1;
        while (end < 51 && bytes[end] !== 0) end++;
        const text = new TextDecoder('utf-8').decode(new Uint8Array(bytes.slice(1, end)));
        const msgId = bytes.length >= 53 ? (bytes[51] | (bytes[52] << 8)) : 0;
        const chunkSeq = bytes.length >= 54 ? bytes[53] : 0;
        return { _kind: 'statustext', severity, text, msgId, chunkSeq };
    },
};

class RadioMavlink {
    constructor() {
        this.port = null;
        this.reader = null;
        this.keepReading = false;
        this.parser = new MavlinkFrameParser((msgId, payload) => this._handleFrame(msgId, payload));

        this.telemetry = { connected: false, source: 'radio' };
        this.logSeq = 0;
        this._statustextChunks = {}; // { id: { chunkSeq: text } } -- sama pola dengan mavlink_adapter.py

        // --- untuk kirim command (fix "Passive Listener Syndrome") ---
        this.writer = null;
        this._txSeq = 0;              // sequence number frame yang KITA kirim
        this._gcsSysId = 255;         // konvensi umum: GCS pakai sysid 255
        this._gcsCompId = 190;        // MAV_COMP_ID_MISSIONPLANNER, dipakai banyak GCS
        this.targetSystem = 1;        // sysid FC, di-update otomatis dari HEARTBEAT masuk
        this.targetComponent = 1;
        this._streamsRequested = false;
        this._lastBaudRate = 57600;

        // Callback publik, isi dari luar (mirip gcs.js punya onTelemetry/onMavlog)
        this.onTelemetry = null;
        this.onMavlog = null;
        this.onConnect = null;
        this.onDisconnect = null;

        this._setupAutoReconnectListeners();
    }

    async tryAutoReconnect(baudRate = 57600) {
        if (!('serial' in navigator)) return false;
        try {
            const ports = await navigator.serial.getPorts();
            if (ports.length === 0) return false;
            this.port = ports[0]; // asumsi 1 radio aktif; kalau multi-device, perlu UI pemilihan
            await this.connect(baudRate);
            console.log('[RadioMavlink] Auto-reconnect berhasil ke port yang sudah pernah diizinkan.');
            return true;
        } catch (e) {
            console.warn('[RadioMavlink] Auto-reconnect gagal:', e.message);
            return false;
        }
    }

    _setupAutoReconnectListeners() {
        if (!('serial' in navigator)) return;

        navigator.serial.addEventListener('disconnect', (event) => {
            if (this.port && event.target === this.port) {
                console.warn('[RadioMavlink] Radio ter-unplug secara fisik.');
                this.telemetry.connected = false;
                this._streamsRequested = false;
                if (this.onDisconnect) this.onDisconnect();
            }
        });

        navigator.serial.addEventListener('connect', async (event) => {
            if (!this.keepReading) {
                console.log('[RadioMavlink] Radio ter-plug kembali, mencoba auto-reconnect...');
                this.port = event.target;
                try {
                    await this.connect(this._lastBaudRate);
                } catch (e) {
                    console.warn('[RadioMavlink] Reconnect otomatis gagal:', e.message);
                }
            }
        });
    }

    async requestPort() {
        this.port = await navigator.serial.requestPort();
        return this.port;
    }

    async connect(baudRate = 57600) {
        if (!this.port) {
            throw new Error('Belum ada port dipilih, panggil requestPort() dulu (dari klik tombol).');
        }
        await this.port.open({ baudRate });
        this._lastBaudRate = baudRate;
        this.writer = this.port.writable.getWriter();
        this.keepReading = true;
        this._streamsRequested = false;
        this.telemetry.connected = true;
        if (this.onConnect) this.onConnect();
        this._readLoop(); // jalan di background, tidak di-await
    }

    async disconnect() {
        this.keepReading = false;
        try {
            if (this.writer) {
                await this.writer.close().catch(() => {});
                this.writer = null;
            }
            if (this.reader) {
                await this.reader.cancel();
                this.reader.releaseLock();
            }
            if (this.port) await this.port.close();
        } catch (e) {
            console.warn('[RadioMavlink] Error saat disconnect:', e);
        }
        this.telemetry.connected = false;
        this._streamsRequested = false;
        if (this.onDisconnect) this.onDisconnect();
    }

    async _sendFrame(msgId, payloadBytes, crcExtra) {
        if (!this.writer) return;
        const frame = buildMavlink1Frame(
            this._txSeq++, this._gcsSysId, this._gcsCompId,
            msgId, payloadBytes, crcExtra
        );
        try {
            await this.writer.write(frame);
        } catch (e) {
            console.warn(`[RadioMavlink] Gagal kirim msgId=${msgId}:`, e.message);
        }
    }

    async requestDataStreams(rateHz = STREAM_RATE_HZ) {
        for (const streamId of REQUESTED_STREAMS) {
            // REQUEST_DATA_STREAM payload, urutan WIRE: req_message_rate(u16), target_system(u8),
            // target_component(u8), req_stream_id(u8), start_stop(u8)
            const rateBytes = [rateHz & 0xFF, (rateHz >> 8) & 0xFF];
            const payload = [...rateBytes, this.targetSystem, this.targetComponent, streamId, 1];
            await this._sendFrame(66, payload, CRC_EXTRA[66]);
            await new Promise(r => setTimeout(r, 50)); // samain gap 50ms dengan mavlink_core.py
        }
        console.log(`[RadioMavlink] Streams di-request @ ${rateHz}Hz (${REQUESTED_STREAMS.length} stream) ke sysid=${this.targetSystem}`);
    }

    async _readLoop() {
        while (this.port.readable && this.keepReading) {
            this.reader = this.port.readable.getReader();
            try {
                while (true) {
                    const { value, done } = await this.reader.read();
                    if (done) break;
                    for (const byte of value) this.parser.feedByte(byte);
                }
            } catch (err) {
                console.error('[RadioMavlink] Read error:', err);
            } finally {
                this.reader.releaseLock();
            }
        }
    }

    _handleFrame(msgId, payload, sysid, compid) {
        if (msgId === 0 && !this._streamsRequested) {
            this.targetSystem = sysid;
            this.targetComponent = compid;
            this._streamsRequested = true; // set duluan, cegah request dobel kalau HEARTBEAT numpuk
            this.requestDataStreams().catch(e => console.warn('[RadioMavlink] requestDataStreams gagal:', e));
        }

        const decoder = DECODERS[msgId];
        if (!decoder) return;

        let result;
        try {
            result = decoder(payload);
        } catch (e) {
            console.warn(`[RadioMavlink] Gagal decode msgId=${msgId}:`, e);
            return;
        }

        if (result._kind === 'telemetry') {
            delete result._kind;
            Object.assign(this.telemetry, result);
            this.telemetry.connected = true;
            if (this.onTelemetry) this.onTelemetry({ ...this.telemetry });
        } else if (result._kind === 'statustext') {
            this._handleStatustext(result);
        }
    }

    // Reassembly chunk STATUSTEXT panjang, sama pola dengan mavlink_adapter.py
    _handleStatustext({ severity, text, msgId, chunkSeq }) {
        let fullText = text;

        if (!(msgId === 0 && chunkSeq === 0)) {
            const buf = this._statustextChunks[msgId] || {};
            buf[chunkSeq] = text;
            this._statustextChunks[msgId] = buf;

            if (text.length < 50) {
                // chunk terakhir -- gabungkan semua chunk urut
                const keys = Object.keys(buf).map(Number).sort((a, b) => a - b);
                fullText = keys.map(k => buf[k]).join('');
                delete this._statustextChunks[msgId];
            } else {
                return; // masih nunggu chunk berikutnya
            }
        }

        this.logSeq++;
        const logItem = {
            seq: this.logSeq,
            text: fullText,
            severity,
            server_time: Date.now() / 1000,
            source: 'radio', // penanda asal pesan, beda dari log UDP via Pi
        };
        if (this.onMavlog) this.onMavlog(logItem);
    }
}

window.RadioMavlink = RadioMavlink;