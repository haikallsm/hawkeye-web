// 1. CRC-16/MCRF4XX (X.25) -- algoritma checksum resmi MAVLink
function mavlinkCrc16Update(crc, byte) {
    let tmp = byte ^ (crc & 0xFF);
    tmp = (tmp ^ (tmp << 4)) & 0xFF;
    return ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF;
}

// CRC_EXTRA per message ID -- konstanta resmi dari dialect common.xml.
// WAJIB benar, kalau salah semua frame pesan itu akan selalu gagal CRC
// dan otomatis ke-drop tanpa pernah sampai ke decoder.
const CRC_EXTRA = {
    0: 50,    // HEARTBEAT
    1: 124,   // SYS_STATUS
    24: 24,   // GPS_RAW_INT
    30: 39,   // ATTITUDE
    33: 104,  // GLOBAL_POSITION_INT
    74: 20,   // VFR_HUD
    253: 83,  // STATUSTEXT
};

// 2. MAVLink Frame Parser -- state machine byte-per-byte
//    Mendukung MAVLink v1 (0xFE) dan v2 (0xFD)
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
                this.state = PARSE_STATE.GOT_SYSID;
                break;

            case PARSE_STATE.GOT_SYSID:
                this.buf.push(byte); // compid
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

        this.onFrame(this.msgId, payload);
    }
}

// ============================================================
// 3. Decoder per message -- konversi payload bytes -> objek JS
//    Urutan field WIRE (bukan urutan deklarasi XML!) -- field
//    di-reorder oleh MAVLink compiler dari besar ke kecil.
// ============================================================
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

// 4. RadioMavlink -- wrapper Web Serial + reassembly + emit
class RadioMavlink {
    constructor() {
        this.port = null;
        this.reader = null;
        this.keepReading = false;
        this.parser = new MavlinkFrameParser((msgId, payload) => this._handleFrame(msgId, payload));

        this.telemetry = { connected: false, source: 'radio' };
        this.logSeq = 0;
        this._statustextChunks = {};

        this.onTelemetry = null;
        this.onMavlog = null;
        this.onConnect = null;
        this.onDisconnect = null;
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
        this.keepReading = true;
        this.telemetry.connected = true;
        if (this.onConnect) this.onConnect();
        this._readLoop();
    }

    async disconnect() {
        this.keepReading = false;
        try {
            if (this.reader) {
                await this.reader.cancel();
                this.reader.releaseLock();
            }
            if (this.port) await this.port.close();
        } catch (e) {
            console.warn('[RadioMavlink] Error saat disconnect:', e);
        }
        this.telemetry.connected = false;
        if (this.onDisconnect) this.onDisconnect();
    }

    async _readLoop() {
        while (this.port.readable && this.keepReading) {
            this.reader = this.port.readable.getReader();
            try {
                while (true) {
                    const { value, done } = await this.reader.read();
                    if (done) break;
                    console.log('[RadioMavlink] Received', value.length, 'bytes');
                    for (const byte of value) this.parser.feedByte(byte);
                }
            } catch (err) {
                console.error('[RadioMavlink] Read error:', err);
            } finally {
                this.reader.releaseLock();
            }
        }
    }

    _handleFrame(msgId, payload) {
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
            source: 'radio',
        };
        if (this.onMavlog) this.onMavlog(logItem);
    }
}

window.RadioMavlink = RadioMavlink;