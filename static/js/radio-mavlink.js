/**
 * radio-mavlink.js
 * ============================================================
 * Parser MAVLink v1/v2 murni JavaScript untuk jalur RADIO independen
 * (SiK/RFD900 dongle yang dicolok LANGSUNG ke laptop GCS).
 *
 * Versi ini mencakup:
 *   - Parsing frame dan telemetry dasar (HEARTBEAT, SYS_STATUS, ATTITUDE, GPS, VFR_HUD, STATUSTEXT)
 *   - COMMAND_LONG / COMMAND_INT
 *   - PARAM_REQUEST_READ / PARAM_SET / PARAM_REQUEST_LIST
 *   - MISSION_COUNT / MISSION_ITEM_INT / MISSION_ACK / MISSION_REQUEST_LIST
 *   - Kalibrasi (wrapper command_long)
 *   - PID tuning (getParam/setParam wrapper)
 *
 * Semua method mengembalikan Promise dan menangani timeout.
 * ============================================================
 */

// ============================================================
// 1. CRC-16/MCRF4XX (X.25) -- algoritma checksum resmi MAVLink
// ============================================================
function mavlinkCrc16Update(crc, byte) {
    let tmp = byte ^ (crc & 0xFF);
    tmp = (tmp ^ (tmp << 4)) & 0xFF;
    return ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF;
}

// CRC_EXTRA per message ID -- SEMUA nilai di bawah diverifikasi langsung dari
// pymavlink.dialects.v20.common (mavlink_map[id].crc_extra), bukan dihitung
// manual/ditebak. Jangan ubah nilai ini tanpa verifikasi ulang ke sumber resmi --
// CRC_EXTRA yang salah membuat frame di-drop diam-diam tanpa error apapun.
const CRC_EXTRA = {
    0: 50,     // HEARTBEAT
    1: 124,    // SYS_STATUS
    20: 214,   // PARAM_REQUEST_READ
    21: 159,   // PARAM_REQUEST_LIST
    22: 220,   // PARAM_VALUE
    23: 168,   // PARAM_SET
    24: 24,    // GPS_RAW_INT
    30: 39,    // ATTITUDE
    33: 104,   // GLOBAL_POSITION_INT
    43: 132,   // MISSION_REQUEST_LIST
    44: 221,   // MISSION_COUNT
    47: 153,   // MISSION_ACK
    51: 196,   // MISSION_REQUEST_INT (pengganti modern MISSION_REQUEST lama)
    66: 148,   // REQUEST_DATA_STREAM
    73: 38,    // MISSION_ITEM_INT
    74: 20,    // VFR_HUD
    76: 152,   // COMMAND_LONG
    77: 143,   // COMMAND_ACK
    183: 85,   // MAV_CMD_DO_SET_SERVO -- BUKAN message ID asli, dikirim lewat COMMAND_LONG(76).
               // Entry ini sengaja TIDAK dipakai untuk encode/decode frame langsung.
    253: 83,   // STATUSTEXT
};

// MAV_DATA_STREAM yang di-request
const REQUESTED_STREAMS = [1, 2, 3, 6, 10, 11, 12];
const STREAM_RATE_HZ = 10;

// ============================================================
// 1b. Frame ENCODER
// ============================================================
function buildMavlink1Frame(seq, sysid, compid, msgId, payloadBytes, crcExtra) {
    const len = payloadBytes.length;
    const beforeCrc = [len, seq & 0xFF, sysid, compid, msgId, ...payloadBytes];
    let crc = 0xFFFF;
    for (const b of beforeCrc) crc = mavlinkCrc16Update(crc, b);
    crc = mavlinkCrc16Update(crc, crcExtra);
    return new Uint8Array([0xFE, ...beforeCrc, crc & 0xFF, (crc >> 8) & 0xFF]);
}

// ============================================================
// 2. MAVLink Frame Parser -- state machine
// ============================================================
const PARSE_STATE = {
    IDLE: 0, GOT_STX: 1, GOT_LENGTH: 2, GOT_INCOMPAT: 3, GOT_COMPAT: 4,
    GOT_SEQ: 5, GOT_SYSID: 6, GOT_COMPID: 7, GOT_MSGID: 8, GOT_PAYLOAD: 9,
};

class MavlinkFrameParser {
    constructor(onFrame) {
        this.onFrame = onFrame;
        this._reset();
    }

    _reset() {
        this.state = PARSE_STATE.IDLE;
        this.isV2 = false;
        this.buf = [];
        this.payloadLen = 0;
        this.msgId = 0;
        this.crcAccum = 0xFFFF;
        this.headerLen = 0;
        this._sysid = 0;
        this._compid = 0;
    }

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
                break;
            case PARSE_STATE.GOT_LENGTH:
                if (this.isV2) {
                    this.buf.push(byte);
                    this.state = PARSE_STATE.GOT_INCOMPAT;
                } else {
                    this.buf.push(byte);
                    this.state = PARSE_STATE.GOT_SEQ;
                }
                break;
            case PARSE_STATE.GOT_INCOMPAT:
                this.buf.push(byte);
                this.state = PARSE_STATE.GOT_COMPAT;
                break;
            case PARSE_STATE.GOT_COMPAT:
                this.buf.push(byte);
                this.state = PARSE_STATE.GOT_SEQ;
                break;
            case PARSE_STATE.GOT_SEQ:
                this.buf.push(byte);
                this._sysid = byte;
                this.state = PARSE_STATE.GOT_SYSID;
                break;
            case PARSE_STATE.GOT_SYSID:
                this.buf.push(byte);
                this._compid = byte;
                this.state = PARSE_STATE.GOT_COMPID;
                break;
            case PARSE_STATE.GOT_COMPID:
                this.buf.push(byte);
                if (this.isV2) {
                    this.msgIdBytes = [byte];
                    this._msgIdByteCount = 1;
                    this.state = PARSE_STATE.GOT_MSGID;
                } else {
                    this.msgId = byte;
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
        if (crcExtra === undefined) return;
        let crc = 0xFFFF;
        for (const b of this.buf) crc = mavlinkCrc16Update(crc, b);
        crc = mavlinkCrc16Update(crc, crcExtra);
        const receivedCrc = this._crcBytes[0] | (this._crcBytes[1] << 8);
        if (crc !== receivedCrc) return;
        this.onFrame(this.msgId, payload, this._sysid, this._compid);
    }
}

// ============================================================
// 3. Decoder per message
// ============================================================
function readF32(dv, off) { return dv.getFloat32(off, true); }
function readU32(dv, off) { return dv.getUint32(off, true); }
function readI32(dv, off) { return dv.getInt32(off, true); }
function readU16(dv, off) { return dv.getUint16(off, true); }

const MAV_TYPE_ROVER = 10;
const MAV_TYPE_COPTER_SET = new Set([2, 13, 14]); // QUADROTOR, HEXAROTOR-ish umum ArduCopter
const COPTER_MODE_MAP = { 0:'STABILIZE',2:'ALT_HOLD',3:'AUTO',4:'GUIDED',5:'LOITER',6:'RTL',9:'LAND',16:'POSHOLD',20:'GUIDED_NOGPS' };
const ROVER_MODE_MAP = { 0:'MANUAL',3:'STEERING',4:'HOLD',5:'LOITER',10:'AUTO',11:'RTL',15:'GUIDED' };

const DECODERS = {
    0: (bytes) => {
        const dv = new DataView(new Uint8Array(bytes).buffer);
        const custom_mode = readU32(dv, 0);
        const type = dv.getUint8(4);
        const base_mode = dv.getUint8(6);
        const armed = (base_mode & 0x80) !== 0;
        const vehicle_type = type === MAV_TYPE_ROVER ? 'ROVER' : (MAV_TYPE_COPTER_SET.has(type) ? 'COPTER' : 'UNKNOWN');
        const modeMap = vehicle_type === 'ROVER' ? ROVER_MODE_MAP : COPTER_MODE_MAP;
        return { _kind:'telemetry', armed, vehicle_type, mode: modeMap[custom_mode] || `MODE_${custom_mode}` };
    },
    1: (bytes) => {
        const dv = new DataView(new Uint8Array(bytes).buffer);
        return { _kind:'telemetry', battery_voltage: readU16(dv,14)/1000, battery_remaining: dv.getInt8(30) };
    },
    // PARAM_VALUE (id=22) -- wire order verified: param_value(f,0), param_count(u16,4),
    // param_index(u16,6), param_id(char[16],8), param_type(u8,24)
    22: (bytes) => {
        const dv = new DataView(new Uint8Array(bytes).buffer);
        const value = dv.getFloat32(0, true);
        const paramCount = dv.getUint16(4, true);
        const paramIndex = dv.getUint16(6, true);
        let name = '';
        for (let i = 8; i < 24; i++) { const c = dv.getUint8(i); if (c === 0) break; name += String.fromCharCode(c); }
        const paramType = dv.getUint8(24);
        return { _kind:'param_value', name, value, paramIndex, paramCount, paramType };
    },
    30: (bytes) => {
        const dv = new DataView(new Uint8Array(bytes).buffer);
        return { _kind:'telemetry', attitude: { roll: readF32(dv,4), pitch: readF32(dv,8), yaw: readF32(dv,12) } };
    },
    33: (bytes) => {
        const dv = new DataView(new Uint8Array(bytes).buffer);
        return { _kind:'telemetry', latitude: readI32(dv,4)/1e7, longitude: readI32(dv,8)/1e7, altitude: readI32(dv,16)/1000, heading: readU16(dv,26)/100 };
    },
    74: (bytes) => {
        const dv = new DataView(new Uint8Array(bytes).buffer);
        return { _kind:'telemetry', speed: readF32(dv,4), climb: readF32(dv,12) };
    },
    24: (bytes) => {
        const dv = new DataView(new Uint8Array(bytes).buffer);
        return { _kind:'telemetry', gps_fix_type: dv.getUint8(28), satellites_visible: dv.getUint8(29) };
    },
    // MISSION_REQUEST_INT (id=51) -- FC minta item tertentu ke GCS (dipakai saat UPLOAD,
    // FC yang jadi "penarik"). Wire order: seq(u16,0), target_system(1,2), target_component(1,3)
    51: (bytes) => {
        const dv = new DataView(new Uint8Array(bytes).buffer);
        return { _kind:'mission_request', seq: dv.getUint16(0, true) };
    },
    // MISSION_COUNT (id=44) -- wire order: count(u16,0), target_system(1,2), target_component(1,3)
    44: (bytes) => {
        const dv = new DataView(new Uint8Array(bytes).buffer);
        return { _kind:'mission_count', count: dv.getUint16(0, true) };
    },
    // MISSION_ACK (id=47) -- wire order: target_system(0), target_component(1), type(2)
    47: (bytes) => {
        return { _kind:'mission_ack', result: bytes[2] };
    },
    // MISSION_ITEM_INT (id=73) -- wire order verified lengkap lewat pymavlink:
    // param1-4(f,0-16), x(i32,16), y(i32,20), z(f,24), seq(u16,28), command(u16,30),
    // target_system(32), target_component(33), frame(34), current(35), autocontinue(36)
    73: (bytes) => {
        const dv = new DataView(new Uint8Array(bytes).buffer);
        return {
            _kind: 'mission_item',
            param1: dv.getFloat32(0, true), param2: dv.getFloat32(4, true),
            param3: dv.getFloat32(8, true), param4: dv.getFloat32(12, true),
            x: dv.getInt32(16, true), y: dv.getInt32(20, true), z: dv.getFloat32(24, true),
            seq: dv.getUint16(28, true), command: dv.getUint16(30, true),
            frame: dv.getUint8(34), current: dv.getUint8(35), autocontinue: dv.getUint8(36),
        };
    },
    77: (bytes) => {
        const dv = new DataView(new Uint8Array(bytes).buffer);
        return { _kind:'command_ack', command: readU16(dv,0), result: dv.getUint8(2) };
    },
    253: (bytes) => {
        const severity = bytes[0];
        let end = 1;
        while (end < 51 && bytes[end] !== 0) end++;
        const text = new TextDecoder('utf-8').decode(new Uint8Array(bytes.slice(1, end)));
        const msgId = bytes.length >= 53 ? (bytes[51] | (bytes[52] << 8)) : 0;
        const chunkSeq = bytes.length >= 54 ? bytes[53] : 0;
        return { _kind:'statustext', severity, text, msgId, chunkSeq };
    },
};

// ============================================================
// 4. RadioMavlink -- kelas utama
// ============================================================
class RadioMavlink {
    constructor() {
        this.port = null;
        this.reader = null;
        this.keepReading = false;
        this.parser = new MavlinkFrameParser((msgId, payload, sysid, compid) => this._handleFrame(msgId, payload, sysid, compid));

        this.telemetry = { connected: false, source: 'radio' };
        this.logSeq = 0;
        this._statustextChunks = {};

        this.writer = null;
        this._txSeq = 0;
        this._gcsSysId = 255;
        this._gcsCompId = 190;
        this.targetSystem = 1;
        this.targetComponent = 1;
        this._streamsRequested = false;
        this._lastBaudRate = 57600;

        // Waiters untuk command/param/mission
        this._ackWaiters = new Map();
        this._paramWaiters = new Map();      // untuk getParam/setParam (key: nama param)
        this._paramListWaiters = [];         // untuk getAllParams
        this._missionUpload = null;          // state aktif uploadMission() (FC yang menarik item)
        this._missionCountWaiter = null;     // waiter MISSION_COUNT saat downloadMission()
        this._missionItemWaiter = null;      // waiter MISSION_ITEM_INT per-seq saat downloadMission()

        // Callback publik
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
            this.port = ports[0];
            await this.connect(baudRate);
            console.log('[RadioMavlink] Auto-reconnect berhasil.');
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
                console.warn('[RadioMavlink] Radio ter-unplug.');
                this.keepReading = false;
                this.writer = null;
                this.reader = null;
                this.telemetry.connected = false;
                this._streamsRequested = false;
                if (this.onDisconnect) this.onDisconnect();
            }
        });
        navigator.serial.addEventListener('connect', async (event) => {
            if (!this.keepReading) {
                console.log('[RadioMavlink] Radio ter-plug kembali, auto-reconnect...');
                this.port = event.target;
                try { await this.connect(this._lastBaudRate); } catch (e) {}
            }
        });
    }

    async requestPort() {
        this.port = await navigator.serial.requestPort();
        return this.port;
    }

    async connect(baudRate = 57600) {
        if (!this.port) throw new Error('Belum ada port dipilih.');
        await this.port.open({ baudRate });
        this._lastBaudRate = baudRate;
        this.writer = this.port.writable.getWriter();
        this.keepReading = true;
        this._streamsRequested = false;
        this.telemetry.connected = true;
        if (this.onConnect) this.onConnect();
        this._readLoop();
    }

    async disconnect() {
        this.keepReading = false;
        try {
            if (this.writer) { await this.writer.close().catch(()=>{}); this.writer = null; }
            if (this.reader) { await this.reader.cancel(); this.reader.releaseLock(); }
            if (this.port) await this.port.close();
        } catch (e) {}
        this.telemetry.connected = false;
        this._streamsRequested = false;
        if (this.onDisconnect) this.onDisconnect();
    }

    async _sendFrame(msgId, payloadBytes, crcExtra) {
        if (!this.writer) return;
        const frame = buildMavlink1Frame(this._txSeq++, this._gcsSysId, this._gcsCompId, msgId, payloadBytes, crcExtra);
        try { await this.writer.write(frame); } catch (e) {}
    }

    async requestDataStreams(rateHz = STREAM_RATE_HZ) {
        for (const streamId of REQUESTED_STREAMS) {
            const rateBytes = [rateHz & 0xFF, (rateHz >> 8) & 0xFF];
            const payload = [...rateBytes, this.targetSystem, this.targetComponent, streamId, 1];
            await this._sendFrame(66, payload, CRC_EXTRA[66]);
            await new Promise(r => setTimeout(r, 50));
        }
        console.log(`[RadioMavlink] Streams requested @ ${rateHz}Hz`);
    }

    // ============================================================
    // COMMAND_LONG
    // ============================================================
    async sendCommandLong(command, params = {}, waitAck = true, timeout = 3000) {
        const p = { p1:0, p2:0, p3:0, p4:0, p5:0, p6:0, p7:0, ...params };
        const payload = new Uint8Array(33);
        const dv = new DataView(payload.buffer);
        [p.p1, p.p2, p.p3, p.p4, p.p5, p.p6, p.p7].forEach((v,i) => dv.setFloat32(i*4, v, true));
        dv.setUint16(28, command, true);
        dv.setUint8(30, this.targetSystem);
        dv.setUint8(31, this.targetComponent);
        dv.setUint8(32, 0);

        if (!waitAck) {
            await this._sendFrame(76, Array.from(payload), CRC_EXTRA[76]);
            return { ok: true, result: null };
        }
        const ackPromise = new Promise((resolve) => {
            const timer = setTimeout(() => {
                this._ackWaiters.delete(command);
                resolve({ ok: false, result: 'timeout' });
            }, timeout);
            this._ackWaiters.set(command, { resolve, timer });
        });
        await this._sendFrame(76, Array.from(payload), CRC_EXTRA[76]);
        return ackPromise;
    }

    // ============================================================
    // COMMAND_INT
    // ============================================================
    async sendCommandInt(command, params = {}, waitAck = true, timeout = 3000) {
        const p = { param1:0, param2:0, param3:0, param4:0, x:0, y:0, z:0, frame:0, ...params };
        const buf = new ArrayBuffer(1+1+1+2+1+1+4*4+4+4+4);
        const dv = new DataView(buf);
        let off=0;
        dv.setUint8(off, this.targetSystem); off++;
        dv.setUint8(off, this.targetComponent); off++;
        dv.setUint8(off, p.frame); off++;
        dv.setUint16(off, command, true); off+=2;
        dv.setUint8(off, 0); off++;
        dv.setUint8(off, 0); off++;
        dv.setFloat32(off, p.param1, true); off+=4;
        dv.setFloat32(off, p.param2, true); off+=4;
        dv.setFloat32(off, p.param3, true); off+=4;
        dv.setFloat32(off, p.param4, true); off+=4;
        dv.setInt32(off, p.x, true); off+=4;
        dv.setInt32(off, p.y, true); off+=4;
        dv.setFloat32(off, p.z, true); off+=4;
        const payload = Array.from(new Uint8Array(buf));
        if (!waitAck) {
            await this._sendFrame(75, payload, CRC_EXTRA[75]);
            return { ok: true, result: null };
        }
        const ackPromise = new Promise((resolve) => {
            const timer = setTimeout(() => {
                this._ackWaiters.delete(command);
                resolve({ ok: false, result: 'timeout' });
            }, timeout);
            this._ackWaiters.set(command, { resolve, timer });
        });
        await this._sendFrame(75, payload, CRC_EXTRA[75]);
        return ackPromise;
    }

    // ============================================================
    // PARAMETER
    // ============================================================
    // Encode nama parameter jadi char[16] MAVLink (dipotong, di-pad nol)
    _encodeParamId(name) {
        const bytes = new Uint8Array(16);
        bytes.set(new TextEncoder().encode(name).slice(0, 16));
        return Array.from(bytes);
    }

    // PARAM_REQUEST_READ (id=20) wire order TERVERIFIKASI: param_index(i16,0),
    // target_system(1,2), target_component(1,3), param_id(char[16],4).
    // param_index=-1 (0xFFFF) berarti cari berdasarkan nama, bukan index.
    async getParam(name, timeout = 3000) {
        const payload = [0xFF, 0xFF, this.targetSystem, this.targetComponent, ...this._encodeParamId(name)];
        const promise = new Promise((resolve, reject) => {
            const timer = setTimeout(() => {
                this._paramWaiters.delete(name);
                reject(new Error(`Timeout getParam ${name}`));
            }, timeout);
            this._paramWaiters.set(name, { resolve: (v) => { clearTimeout(timer); resolve(v); }, timer });
        });
        await this._sendFrame(20, payload, CRC_EXTRA[20]);
        return promise;
    }

    // PARAM_SET (id=23) wire order TERVERIFIKASI: param_value(f,0), target_system(1,4),
    // target_component(1,5), param_id(char[16],6), param_type(1,22)
    async setParam(name, value, paramType = 9, timeout = 3000) {
        const buf = new ArrayBuffer(23);
        const dv = new DataView(buf);
        dv.setFloat32(0, value, true);
        dv.setUint8(4, this.targetSystem);
        dv.setUint8(5, this.targetComponent);
        this._encodeParamId(name).forEach((b, i) => dv.setUint8(6 + i, b));
        dv.setUint8(22, paramType);
        const payload = Array.from(new Uint8Array(buf));

        // FC membalas PARAM_SET dengan PARAM_VALUE (echo nilai yang benar-benar
        // tersimpan) -- pakai waiter yang sama dengan getParam supaya satu jalur.
        const promise = new Promise((resolve) => {
            const timer = setTimeout(() => {
                this._paramWaiters.delete(name);
                resolve({ ok: false, reason: 'timeout' });
            }, timeout);
            this._paramWaiters.set(name, {
                resolve: (v) => { clearTimeout(timer); resolve({ ok: true, value: v }); },
                timer
            });
        });
        await this._sendFrame(23, payload, CRC_EXTRA[23]);
        return promise;
    }

    // PARAM_REQUEST_LIST (id=21) -- minta semua parameter. FC (terutama
    // ArduPilot, bisa >1000 parameter) akan membalas dengan banyak PARAM_VALUE
    // satu-satu. Timeout dibuat ROLLING (reset tiap ada progress), bukan flat,
    // supaya FC dengan parameter banyak tidak keburu timeout padahal masih jalan.
    async getAllParams(onProgress, timeout = 20000) {
        return new Promise((resolve, reject) => {
            const waiter = { resolve, reject, params: {}, total: null, count: 0, onProgress, timer: null };
            waiter._resetTimer = () => {
                if (waiter.timer) clearTimeout(waiter.timer);
                waiter.timer = setTimeout(() => {
                    const idx = this._paramListWaiters.indexOf(waiter);
                    if (idx !== -1) this._paramListWaiters.splice(idx, 1);
                    reject(new Error(`Timeout getAllParams (${waiter.count}/${waiter.total ?? '?'} diterima)`));
                }, timeout);
            };
            waiter._resetTimer();
            this._paramListWaiters.push(waiter);
            this._sendFrame(21, [this.targetSystem, this.targetComponent], CRC_EXTRA[21]);
        });
    }

    // ============================================================
    // MISSION
    // Protokol MAVLink itu SEARAH-BERBEDA tergantung upload vs download:
    // - UPLOAD : GCS kirim MISSION_COUNT -> FC MENARIK tiap item lewat
    //            MISSION_REQUEST_INT satu-satu -> FC yang kirim MISSION_ACK
    //            di akhir. GCS TIDAK boleh kirim item sebelum diminta.
    // - DOWNLOAD: GCS kirim MISSION_REQUEST_LIST -> FC balas MISSION_COUNT ->
    //            GCS MENARIK tiap item satu-satu lewat MISSION_REQUEST_INT ->
    //            GCS yang kirim MISSION_ACK di akhir (bukan FC).
    // ============================================================
    async uploadMission(items, timeout = 10000) {
        const count = items.length;
        if (count === 0) throw new Error('Mission kosong, tidak ada yang di-upload');

        return new Promise((resolve, reject) => {
            const state = { items, resolve, reject, timer: null };
            state.resetTimer = () => {
                if (state.timer) clearTimeout(state.timer);
                state.timer = setTimeout(() => {
                    this._missionUpload = null;
                    reject(new Error('Upload mission timeout -- FC berhenti meminta item'));
                }, timeout);
            };
            state.resetTimer();
            this._missionUpload = state;

            // MISSION_COUNT (id=44) wire order: count(u16,0), target_system(1,2), target_component(1,3)
            const countPayload = [count & 0xFF, (count >> 8) & 0xFF, this.targetSystem, this.targetComponent];
            this._sendFrame(44, countPayload, CRC_EXTRA[44]);
            // Setelah ini FC akan kirim MISSION_REQUEST_INT(seq) satu-satu,
            // ditangani _handleMissionRequest() di bawah -- GCS PASIF menunggu diminta.
        });
    }

    // MISSION_ITEM_INT (id=73) wire order TERVERIFIKASI (bukan urutan deklarasi XML):
    // param1-4(f,0-16), x(i32,16), y(i32,20), z(f,24), seq(u16,28), command(u16,30),
    // target_system(32), target_component(33), frame(34), current(35), autocontinue(36)
    _buildMissionItemPayload(seq, item) {
        const buf = new ArrayBuffer(37);
        const dv = new DataView(buf);
        dv.setFloat32(0, item.param1 || 0, true);
        dv.setFloat32(4, item.param2 || 0, true);
        dv.setFloat32(8, item.param3 || 0, true);
        dv.setFloat32(12, item.param4 || 0, true);
        dv.setInt32(16, item.x || 0, true);
        dv.setInt32(20, item.y || 0, true);
        dv.setFloat32(24, item.z || 0, true);
        dv.setUint16(28, seq, true);
        dv.setUint16(30, item.command, true);
        dv.setUint8(32, this.targetSystem);
        dv.setUint8(33, this.targetComponent);
        dv.setUint8(34, item.frame ?? 3); // default MAV_FRAME_GLOBAL_RELATIVE_ALT
        dv.setUint8(35, item.current || 0);
        dv.setUint8(36, item.autocontinue ?? 1);
        return Array.from(new Uint8Array(buf));
    }

    async downloadMission(timeout = 5000, maxRetries = 3) {
        const count = await new Promise((resolve, reject) => {
            const timer = setTimeout(() => reject(new Error('Timeout MISSION_REQUEST_LIST')), timeout);
            this._missionCountWaiter = { resolve: (c) => { clearTimeout(timer); resolve(c); } };
            this._sendFrame(43, [this.targetSystem, this.targetComponent], CRC_EXTRA[43]);
        });

        if (count === 0) return [];

        const items = [];
        for (let seq = 0; seq < count; seq++) {
            let item = null;
            for (let attempt = 0; attempt < maxRetries && !item; attempt++) {
                try {
                    item = await new Promise((resolve, reject) => {
                        const timer = setTimeout(() => reject(new Error('timeout')), timeout);
                        this._missionItemWaiter = { seq, resolve: (it) => { clearTimeout(timer); resolve(it); } };
                        // MISSION_REQUEST_INT (id=51) wire order: seq(u16,0), target_system(1,2), target_component(1,3)
                        const payload = [seq & 0xFF, (seq >> 8) & 0xFF, this.targetSystem, this.targetComponent];
                        this._sendFrame(51, payload, CRC_EXTRA[51]);
                    });
                } catch (e) {
                    console.warn(`[RadioMavlink] Item seq=${seq} timeout, percobaan ${attempt + 1}/${maxRetries}`);
                }
            }
            if (!item) throw new Error(`Gagal ambil mission item seq=${seq} setelah ${maxRetries} percobaan`);
            items.push(item);
        }

        // GCS yang kirim MISSION_ACK di akhir DOWNLOAD (beda dari upload, di mana FC yang kirim)
        // MISSION_ACK (id=47) wire order: target_system(0), target_component(1), type(2)
        await this._sendFrame(47, [this.targetSystem, this.targetComponent, 0], CRC_EXTRA[47]);
        return items;
    }

    // ============================================================
    // WRAPPER: ARM, TAKEOFF, RTL, LAND, MODE, REBOOT, SET_HOME, SET_SERVO
    // ============================================================
    async armDisarm(arm) {
        return this.sendCommandLong(400, { p1: arm ? 1 : 0 });
    }
    async takeoff(alt = 10) {
        return this.sendCommandLong(22, { p7: alt });
    }
    async rtl() {
        return this.sendCommandLong(20);
    }
    async land() {
        return this.sendCommandLong(21);
    }
    async setMode(modeCode) {
        // modeCode: integer custom_mode
        return this.sendCommandLong(176, { p1: modeCode, p2: 0 });
    }
    async reboot() {
        return this.sendCommandLong(246, { p1: 1 });
    }
    async setHome(lat, lon, alt = 0) {
        return this.sendCommandLong(179, { p1: 0, p5: lat, p6: lon, p7: alt });
    }
    async setServo(channel, pwm) {
        return this.sendCommandLong(183, { p1: channel, p2: pwm });
    }

    // ============================================================
    // CALIBRATION WRAPPER
    // ============================================================
    async calibrateMag(compassId = -1) {
        return this.sendCommandLong(241, { p3: 1, p4: compassId >= 0 ? compassId : 0 });
    }
    async calibrateAccel(retries = 0) {
        return this.sendCommandLong(241, { p1: 1, p2: retries });
    }
    async calibrateGyro() {
        return this.sendCommandLong(241, { p5: 1 });
    }
    async calibrateLevel() {
        return this.sendCommandLong(241, { p6: 1 });
    }

    // ============================================================
    // PID TUNING (read/write parameter)
    // ============================================================
    async getPID(axis) {
        const axisMap = { roll:'ROLL', pitch:'PITCH', yaw:'YAW' };
        const prefix = axisMap[axis.toLowerCase()];
        if (!prefix) throw new Error(`Axis ${axis} tidak didukung.`);
        const P = await this.getParam(`ATC_RAT_${prefix}_P`);
        const I = await this.getParam(`ATC_RAT_${prefix}_I`);
        const D = await this.getParam(`ATC_RAT_${prefix}_D`);
        return { P, I, D };
    }
    async setPID(axis, pid) {
        const axisMap = { roll:'ROLL', pitch:'PITCH', yaw:'YAW' };
        const prefix = axisMap[axis.toLowerCase()];
        if (!prefix) throw new Error(`Axis ${axis} tidak didukung.`);
        await this.setParam(`ATC_RAT_${prefix}_P`, pid.P);
        await this.setParam(`ATC_RAT_${prefix}_I`, pid.I);
        await this.setParam(`ATC_RAT_${prefix}_D`, pid.D);
        return true;
    }

    // ============================================================
    // INTERNAL: READ LOOP & FRAME HANDLER
    // ============================================================
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
        // HEARTBEAT -> request streams & update target
        if (msgId === 0 && !this._streamsRequested) {
            this.targetSystem = sysid;
            this.targetComponent = compid;
            this._streamsRequested = true;
            this.requestDataStreams().catch(e => console.warn('[RadioMavlink] requestDataStreams gagal:', e));
        }

        const decoder = DECODERS[msgId];
        if (!decoder) return;

        let result;
        try { result = decoder(payload); } catch (e) { return; }

        if (result._kind === 'telemetry') {
            delete result._kind;
            Object.assign(this.telemetry, result);
            this.telemetry.connected = true;
            if (this.onTelemetry) this.onTelemetry({ ...this.telemetry });
        } else if (result._kind === 'statustext') {
            this._handleStatustext(result);
        } else if (result._kind === 'command_ack') {
            this._handleCommandAck(result.command, result.result);
        } else if (result._kind === 'param_value') {
            this._handleParamValue(result);
        } else if (result._kind === 'mission_request') {
            this._handleMissionRequest(result.seq);
        } else if (result._kind === 'mission_count') {
            this._handleMissionCount(result.count);
        } else if (result._kind === 'mission_item') {
            this._handleMissionItem(result);
        } else if (result._kind === 'mission_ack') {
            this._handleMissionAck(result.result);
        }
    }

    _handleCommandAck(command, result) {
        const waiter = this._ackWaiters.get(command);
        if (waiter) {
            this._ackWaiters.delete(command);
            clearTimeout(waiter.timer);
            waiter.resolve({ ok: result === 0, result });
        }
    }

    _handleParamValue({ name, value, paramIndex, paramCount }) {
        // Waiter tunggal (getParam/setParam)
        const waiter = this._paramWaiters.get(name);
        if (waiter) {
            this._paramWaiters.delete(name);
            waiter.resolve(value);
            return;
        }
        // Waiter list (getAllParams) -- ambil yang paling depan di antrian
        if (this._paramListWaiters.length > 0) {
            const w = this._paramListWaiters[0];
            w.params[name] = value;
            w.count++;
            w.total = paramCount;
            w._resetTimer(); // ada progress -- perpanjang timeout, jangan flat
            if (w.onProgress) w.onProgress(w.count, w.total, name);
            if (w.count >= w.total) {
                clearTimeout(w.timer);
                this._paramListWaiters.shift();
                w.resolve(w.params);
            }
        }
    }

    // Dipanggil saat MISSION_REQUEST_INT diterima -- FC "menarik" item tertentu
    // selama proses upload. GCS PASIF, cuma respon begitu diminta.
    _handleMissionRequest(seq) {
        const state = this._missionUpload;
        if (!state) return; // tidak ada upload aktif, abaikan
        const item = state.items[seq];
        if (!item) {
            console.warn(`[RadioMavlink] FC minta item seq=${seq} tapi mission cuma punya ${state.items.length} item`);
            return;
        }
        state.resetTimer(); // ada progress -- perpanjang timeout
        const payload = this._buildMissionItemPayload(seq, item);
        this._sendFrame(73, payload, CRC_EXTRA[73]);
    }

    // MISSION_COUNT dari FC -- konteksnya cuma satu: balasan atas
    // MISSION_REQUEST_LIST saat downloadMission(). Saat upload, MISSION_COUNT
    // arahnya kebalik (GCS yang kirim ke FC), jadi tidak perlu ditangani di sini.
    _handleMissionCount(count) {
        if (this._missionCountWaiter) {
            const w = this._missionCountWaiter;
            this._missionCountWaiter = null;
            w.resolve(count);
        }
    }

    // MISSION_ITEM_INT dari FC -- balasan atas MISSION_REQUEST_INT saat downloadMission()
    _handleMissionItem(item) {
        if (this._missionItemWaiter && this._missionItemWaiter.seq === item.seq) {
            const w = this._missionItemWaiter;
            this._missionItemWaiter = null;
            w.resolve(item);
        }
    }

    // MISSION_ACK dari FC -- konteksnya cuma satu: konfirmasi UPLOAD selesai
    // (diterima/ditolak). Saat download, GCS sendiri yang kirim ACK di akhir
    // downloadMission(), FC tidak membalas ACK untuk itu.
    _handleMissionAck(result) {
        const state = this._missionUpload;
        if (!state) return;
        clearTimeout(state.timer);
        this._missionUpload = null;
        if (result === 0) { // MAV_MISSION_ACCEPTED
            state.resolve(true);
        } else {
            state.reject(new Error(`Mission ditolak FC, kode error MAV_MISSION_RESULT=${result}`));
        }
    }

    _handleStatustext({ severity, text, msgId, chunkSeq }) {
        let fullText = text;
        if (!(msgId === 0 && chunkSeq === 0)) {
            const buf = this._statustextChunks[msgId] || {};
            buf[chunkSeq] = text;
            this._statustextChunks[msgId] = buf;
            if (text.length < 50) {
                const keys = Object.keys(buf).map(Number).sort((a,b)=>a-b);
                fullText = keys.map(k => buf[k]).join('');
                delete this._statustextChunks[msgId];
            } else {
                return;
            }
        }
        this.logSeq++;
        const logItem = { seq: this.logSeq, text: fullText, severity, server_time: Date.now()/1000, source: 'radio' };
        if (this.onMavlog) this.onMavlog(logItem);
    }
}

// Export global
window.RadioMavlink = RadioMavlink;
console.log('[RadioMavlink] radio-mavlink.js loaded (full version)');