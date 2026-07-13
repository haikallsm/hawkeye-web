/**
 * command.js
 * ============================================================
 * Jembatan command TERPADU -- satu API yang sama dipanggil dari UI,
 * otomatis diarahkan ke window.radioMavlink (radio, browser murni)
 * atau window.gcsConnection (Pi backend via HTTP) tergantung mana
 * yang sedang connected.
 *
 * PENTING: berbeda dari command.js versi sebelumnya, file ini TIDAK
 * menempel ke class manapun (tidak butuh window.gcsConnection punya
 * field _mode/_radio/commandLong -- field itu tidak pernah ada).
 * Ini murni objek berdiri sendiri yang menampung logic "pilih jalur".
 *
 * Endpoint Pi (/command/reboot, /command/servo, dst) di bawah ini
 * adalah ASUMSI nama route berdasarkan pola /command/reboot dan
 * /command yang sudah ada di appcoba.py. SESUAIKAN path-nya kalau
 * nama route asli di backend kamu berbeda.
 * ============================================================
 */
const commands = {
    // Radio dianggap aktif kalau sudah connect DAN sudah dapat sysid asli
    // dari HEARTBEAT (targetSystem masih default 1 sebelum ada HEARTBEAT
    // pertama, jadi cek telemetry.connected lebih aman daripada cek sysid).
    _radioActive() {
        return !!(window.radioMavlink && window.radioMavlink.telemetry?.connected);
    },
    _piActive() {
        return !!(window.gcsConnection && window.gcsConnection.isConnected);
    },

    async _piPost(path, body = {}) {
        const resp = await fetch(window.gcsConnection.baseUrl + path, {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify(body),
        });
        const data = await resp.json();
        if (data.status && data.status !== 'success' && data.status !== 'ok') {
            throw new Error(data.message || `Request ke ${path} gagal`);
        }
        return data;
    },

    async armDisarm(arm) {
        if (this._radioActive()) return window.radioMavlink.armDisarm(arm);
        if (this._piActive()) return this._piPost('/command', { cmd: arm ? 'ARM' : 'DISARM' });
        throw new Error('Tidak ada koneksi aktif (radio maupun Pi)');
    },

    async reboot() {
        if (this._radioActive()) return window.radioMavlink.reboot();
        if (this._piActive()) return this._piPost('/command/reboot');
        throw new Error('Tidak ada koneksi aktif (radio maupun Pi)');
    },

    async setServo(channel, pwm) {
        if (this._radioActive()) return window.radioMavlink.setServo(channel, pwm);
        if (this._piActive()) return this._piPost('/command/servo', { channel, pwm });
        throw new Error('Tidak ada koneksi aktif (radio maupun Pi)');
    },

    async setHome(lat, lon, alt = 0) {
        if (this._radioActive()) return window.radioMavlink.setHome(lat, lon, alt);
        if (this._piActive()) return this._piPost('/command/set_home', { lat, lon, alt });
        throw new Error('Tidak ada koneksi aktif (radio maupun Pi)');
    },

    async getAllParams(onProgress) {
        if (this._radioActive()) return window.radioMavlink.getAllParams(onProgress);
        if (this._piActive()) {
            const resp = await fetch(window.gcsConnection.baseUrl + '/param/list');
            const data = await resp.json();
            if (data.status !== 'success') throw new Error(data.message);
            return data.params;
        }
        throw new Error('Tidak ada koneksi aktif (radio maupun Pi)');
    },

    async setParam(name, value, paramType = 9) {
        if (this._radioActive()) return window.radioMavlink.setParam(name, value, paramType);
        if (this._piActive()) return this._piPost('/param/set', { name, value });
        throw new Error('Tidak ada koneksi aktif (radio maupun Pi)');
    },

    async getPID(axis) {
        if (this._radioActive()) return window.radioMavlink.getPID(axis);
        if (this._piActive()) {
            const resp = await fetch(window.gcsConnection.baseUrl + `/param/pid/${axis}`);
            const data = await resp.json();
            if (data.status !== 'success') throw new Error(data.message);
            return data.pid;
        }
        throw new Error('Tidak ada koneksi aktif (radio maupun Pi)');
    },

    async setPID(axis, pid) {
        if (this._radioActive()) return window.radioMavlink.setPID(axis, pid);
        if (this._piActive()) return this._piPost(`/param/pid/${axis}`, pid);
        throw new Error('Tidak ada koneksi aktif (radio maupun Pi)');
    },

    async uploadMission(items) {
        if (this._radioActive()) return window.radioMavlink.uploadMission(items);
        if (this._piActive()) return this._piPost('/mission/upload', { items });
        throw new Error('Tidak ada koneksi aktif (radio maupun Pi)');
    },

    async downloadMission() {
        if (this._radioActive()) return window.radioMavlink.downloadMission();
        if (this._piActive()) {
            const resp = await fetch(window.gcsConnection.baseUrl + '/mission/read');
            const data = await resp.json();
            if (data.status !== 'success') throw new Error(data.message);
            return data.items;
        }
        throw new Error('Tidak ada koneksi aktif (radio maupun Pi)');
    },

    async calibrateAccel() {
        if (this._radioActive()) return window.radioMavlink.calibrateAccel();
        if (this._piActive()) return this._piPost('/command/calibrate/accel');
        throw new Error('Tidak ada koneksi aktif (radio maupun Pi)');
    },

    async calibrateMag(compassId = -1) {
        if (this._radioActive()) return window.radioMavlink.calibrateMag(compassId);
        if (this._piActive()) return this._piPost('/command/calibrate/compass', { compassId });
        throw new Error('Tidak ada koneksi aktif (radio maupun Pi)');
    },
};

window.commands = commands;