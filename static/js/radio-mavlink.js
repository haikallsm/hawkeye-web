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

// CRC_EXTRA per message ID -- konstanta resmi dari dialect common.xml.
const CRC_EXTRA = {
    0: 50,    // HEARTBEAT
    1: 124,   // SYS_STATUS
    20: 214,  // PARAM_REQUEST_READ
    21: 99,   // PARAM_REQUEST_LIST
    22: 220,  // PARAM_VALUE
    23: 158,  // PARAM_SET
    24: 24,   // GPS_RAW_INT
    30: 39,   // ATTITUDE
    33: 104,  // GLOBAL_POSITION_INT
    39: 49,   // MISSION_ITEM (v1)
    40: 84,   // MISSION_REQUEST
    43: 103,  // MISSION_REQUEST_LIST
    44: 126,  // MISSION_COUNT
    47: 31,   // MISSION_ACK
    66: 148,  // REQUEST_DATA_STREAM
    73: 89,   // MISSION_ITEM_INT
    74: 20,   // VFR_HUD
    75: 143,  // COMMAND_INT
    76: 152,  // COMMAND_LONG
    77: 143,  // COMMAND_ACK
    183: 85,  // SET_SERVO
    253: 83,  // STATUSTEXT
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

const MAV_TYPE_COPTER_SET = new Set([2, 13, 14]);
const COPTER_MODE_MAP = { 0:'STABILIZE',2:'ALT_HOLD',3:'AUTO',4:'GUIDED',5:'LOITER',6:'RTL',9:'LAND',16:'POSHOLD',20:'GUIDED_NOGPS' };

const DECODERS = {
    0: (bytes) => {
        const dv = new DataView(new Uint8Array(bytes).buffer);
        const custom_mode = readU32(dv, 0);
        const type = dv.getUint8(4);
        const base_mode = dv.getUint8(6);
        const armed = (base_mode & 0x80) !== 0;
        const vehicle_type = MAV_TYPE_COPTER_SET.has(type) ? 'COPTER' : 'UNKNOWN';
        return { _kind:'telemetry', armed, vehicle_type, mode: COPTER_MODE_MAP[custom_mode] || `MODE_${custom_mode}` };
    },
    1: (bytes) => {
        const dv = new DataView(new Uint8Array(bytes).buffer);
        return { _kind:'telemetry', battery_voltage: readU16(dv,14)/1000, battery_remaining: dv.getInt8(30) };
    },
    22: (bytes) => { // PARAM_VALUE
        const dv = new DataView(new Uint8Array(bytes).buffer);
        const paramIndex = dv.getInt16(0, true);
        const paramCount = dv.getInt16(2, true);
        const paramType = dv.getUint8(4);
        const value = dv.getFloat32(5, true);
        let name = '';
        for (let i=9; i<25; i++) { const c = dv.getUint8(i); if (c===0) break; name += String.fromCharCode(c); }
        return { _kind:'param_value', paramIndex, paramCount, paramType, value, name };
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
    40: (bytes) => { // MISSION_REQUEST
        const seq = bytes[2] | (bytes[3] << 8);
        return { _kind:'mission_request', seq };
    },
    44: (bytes) => { // MISSION_COUNT (response)
        const count = bytes[2] | (bytes[3] << 8);
        return { _kind:'mission_count', count };
    },
    47: (bytes) => { // MISSION_ACK
        return { _kind:'mission_ack', result: bytes[2] };
    },
    73: (bytes) => { // MISSION_ITEM_INT
        const dv = new DataView(new Uint8Array(bytes).buffer);
        const seq = dv.getUint16(0, true);
        const frame = dv.getUint8(2);
        const command = dv.getUint16(3, true);
        const current = dv.getUint8(5);
        const autocontinue = dv.getUint8(6);
        const p1 = dv.getFloat32(7, true);
        const p2 = dv.getFloat32(11, true);
        const p3 = dv.getFloat32(15, true);
        const p4 = dv.getFloat32(19, true);
        const x = dv.getInt32(23, true);
        const y = dv.getInt32(27, true);
        const z = dv.getFloat32(31, true);
        return { _kind:'mission_item', seq, frame, command, current, autocontinue, p1, p2, p3, p4, x, y, z };
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
        this._paramWaiters = new Map();      // untuk getParam/setParam
        this._paramListWaiters = [];         // untuk getAllParams
        this._missionWaiters = new Map();    // untuk uploadMission
        this._missionReadWaiters = [];       // untuk readMission

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
    async getParam(name, timeout = 3000) {
        const paramIdBytes = new Uint8Array(16);
        const enc = new TextEncoder();
        const nameBytes = enc.encode(name);
        paramIdBytes.set(nameBytes.slice(0, 16));
        const payload = [this.targetSystem, this.targetComponent, ...paramIdBytes, 0xFF, 0xFF];
        const waiter = new Promise((resolve, reject) => {
            const timer = setTimeout(() => {
                this._paramWaiters.delete(name);
                reject(new Error(`Timeout getParam ${name}`));
            }, timeout);
            this._paramWaiters.set(name, { resolve, timer });
        });
        await this._sendFrame(20, payload, CRC_EXTRA[20]);
        return await waiter;
    }

    async setParam(name, value, paramType = 9, timeout = 3000) {
        const paramIdBytes = new Uint8Array(16);
        const enc = new TextEncoder();
        const nameBytes = enc.encode(name);
        paramIdBytes.set(nameBytes.slice(0, 16));
        const buf = new ArrayBuffer(1+1+16+4+1);
        const dv = new DataView(buf);
        let off=0;
        dv.setUint8(off, this.targetSystem); off++;
        dv.setUint8(off, this.targetComponent); off++;
        for (let i=0;i<16;i++) dv.setUint8(off+i, paramIdBytes[i]);
        off += 16;
        dv.setFloat32(off, value, true); off += 4;
        dv.setUint8(off, paramType);
        const payload = Array.from(new Uint8Array(buf));
        const waiter = new Promise((resolve) => {
            const timer = setTimeout(() => {
                this._paramWaiters.delete(name);
                resolve(false);
            }, timeout);
            this._paramWaiters.set(name, { resolve: (v) => { clearTimeout(timer); resolve(true); }, timer });
        });
        await this._sendFrame(23, payload, CRC_EXTRA[23]);
        return await waiter;
    }

    async getAllParams(timeout = 15000) {
        return new Promise((resolve, reject) => {
            const waiter = {
                resolve, reject,
                params: {},
                total: null,
                count: 0,
                timer: setTimeout(() => {
                    const idx = this._paramListWaiters.indexOf(waiter);
                    if (idx !== -1) this._paramListWaiters.splice(idx, 1);
                    reject(new Error(`Timeout getAllParams`));
                }, timeout)
            };
            this._paramListWaiters.push(waiter);
            const payload = [this.targetSystem, this.targetComponent];
            this._sendFrame(21, payload, CRC_EXTRA[21]);
        });
    }

    // ============================================================
    // MISSION
    // ============================================================
    async uploadMission(items, timeout = 5000) {
        const count = items.length;
        // MISSION_COUNT
        const countPayload = [this.targetSystem, this.targetComponent, count & 0xFF, (count >> 8) & 0xFF, 0];
        const countOk = await this._sendMissionCountAndWait(countPayload, timeout);
        if (!countOk) return false;
        for (let i = 0; i < count; i++) {
            const item = items[i];
            const itemPayload = this._buildMissionItemPayload(i, item);
            const req = await this._sendMissionItemAndWait(itemPayload, timeout);
            if (!req) return false;
        }
        // MISSION_ACK final
        const ackPayload = [this.targetSystem, this.targetComponent, 0,0,0];
        await this._sendFrame(47, ackPayload, CRC_EXTRA[47]);
        return true;
    }

    _buildMissionItemPayload(seq, item) {
        const buf = new ArrayBuffer(1+1+2+1+2+1+1+4*4+4+4+4+1);
        const dv = new DataView(buf);
        let off=0;
        dv.setUint8(off, this.targetSystem); off++;
        dv.setUint8(off, this.targetComponent); off++;
        dv.setUint16(off, seq, true); off+=2;
        dv.setUint8(off, item.frame || 0); off++;
        dv.setUint16(off, item.command, true); off+=2;
        dv.setUint8(off, item.current || 0); off++;
        dv.setUint8(off, item.autocontinue || 1); off++;
        dv.setFloat32(off, item.param1 || 0, true); off+=4;
        dv.setFloat32(off, item.param2 || 0, true); off+=4;
        dv.setFloat32(off, item.param3 || 0, true); off+=4;
        dv.setFloat32(off, item.param4 || 0, true); off+=4;
        dv.setInt32(off, item.x || 0, true); off+=4;
        dv.setInt32(off, item.y || 0, true); off+=4;
        dv.setFloat32(off, item.z || 0, true); off+=4;
        dv.setUint8(off, 0);
        return Array.from(new Uint8Array(buf));
    }

    _sendMissionCountAndWait(payload, timeout) {
        return new Promise((resolve) => {
            const timer = setTimeout(() => resolve(false), timeout);
            const key = 'mission_count';
            this._missionWaiters.set(key, { resolve: (ok) => { clearTimeout(timer); resolve(ok); }, timer });
            this._sendFrame(44, payload, CRC_EXTRA[44]);
        });
    }

    _sendMissionItemAndWait(payload, timeout) {
        return new Promise((resolve) => {
            const timer = setTimeout(() => resolve(false), timeout);
            const key = 'mission_item_' + payload[2];
            this._missionWaiters.set(key, { resolve: (ok) => { clearTimeout(timer); resolve(ok); }, timer });
            this._sendFrame(73, payload, CRC_EXTRA[73]); // MISSION_ITEM_INT
        });
    }

    async readMission(timeout = 5000) {
        return new Promise((resolve, reject) => {
            const timer = setTimeout(() => reject(new Error('Read mission timeout')), timeout);
            this._missionReadWaiters.push({
                resolve: (items) => { clearTimeout(timer); resolve(items); },
                reject: reject,
                items: [],
                expected: 0
            });
            const payload = [this.targetSystem, this.targetComponent, 0];
            this._sendFrame(43, payload, CRC_EXTRA[43]); // MISSION_REQUEST_LIST
        });
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
        // Single waiter (getParam/setParam)
        const waiter = this._paramWaiters.get(name);
        if (waiter) {
            this._paramWaiters.delete(name);
            waiter.resolve(value);
            return;
        }
        // List waiter (getAllParams)
        if (this._paramListWaiters.length > 0) {
            const w = this._paramListWaiters[0];
            w.params[name] = value;
            w.count++;
            w.total = paramCount;
            if (w.count >= w.total) {
                clearTimeout(w.timer);
                this._paramListWaiters.shift();
                w.resolve(w.params);
            }
        }
    }

    _handleMissionRequest(seq) {
        const key = 'mission_item_' + seq;
        const waiter = this._missionWaiters.get(key);
        if (waiter) {
            this._missionWaiters.delete(key);
            waiter.resolve(true);
        }
    }

    _handleMissionCount(count) {
        // Saat membaca misi, kita dapat MISSION_COUNT sebagai respon dari MISSION_REQUEST_LIST
        // Kita simpan expected count
        if (this._missionReadWaiters.length > 0) {
            this._missionReadWaiters[0].expected = count;
        }
    }

    _handleMissionItem(item) {
        if (this._missionReadWaiters.length > 0) {
            const w = this._missionReadWaiters[0];
            w.items.push(item);
            // Kirim MISSION_ACK untuk menerima item
            const ackPayload = [this.targetSystem, this.targetComponent, 0,0,0];
            this._sendFrame(47, ackPayload, CRC_EXTRA[47]);
        }
    }

    _handleMissionAck(result) {
        // Untuk upload mission: mission_count waiter
        const key = 'mission_count';
        const waiter = this._missionWaiters.get(key);
        if (waiter) {
            this._missionWaiters.delete(key);
            waiter.resolve(result === 0);
        }
        // Untuk read mission: selesaikan promise
        if (this._missionReadWaiters.length > 0) {
            const w = this._missionReadWaiters.shift();
            // Kita sudah kumpulkan items di _handleMissionItem
            // Setelah MISSION_ACK, kita resolve
            w.resolve(w.items);
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