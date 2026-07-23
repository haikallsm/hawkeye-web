const commands = {
    _radioActive() {
        return !!(window.radioMavlink && window.radioMavlink.isLive());
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
        throw new Error('Tidak ada koneksi aktif');
    },

    async reboot() {
        if (this._radioActive()) return window.radioMavlink.reboot();
        if (this._piActive()) return this._piPost('/command/reboot');
        throw new Error('Tidak ada koneksi aktif');
    },

    async setFlightMode(mode) {
        if (this._radioActive()) return window.radioMavlink.setMode(mode);
        if (this._piActive()) return this._piPost('/command', { cmd: 'MODE:' + mode });
        throw new Error('Tidak ada koneksi aktif');
    },

    async setSafetySwitch(turnOn) {
        // MAV_CMD_DO_SET_SAFETY_SWITCH_STATE (294): 0.0 = on, 1.0 = off
        const p1Value = turnOn ? 0.0 : 1.0; 
        if (this._radioActive()) return window.radioMavlink.sendCommandLong(294, { p1: p1Value });
        if (this._piActive()) return this._piPost('/safety', { state: turnOn ? 'on' : 'off' });
        throw new Error('Tidak ada koneksi aktif');
    },

    async setServo(channel, pwm) {
        if (this._radioActive()) return window.radioMavlink.setServo(channel, pwm);
        if (this._piActive()) return this._piPost('/command/servo', { channel, pwm });
        throw new Error('Tidak ada koneksi aktif');
    },

    async setHome(lat, lon, alt = 0) {
        if (this._radioActive()) return window.radioMavlink.setHome(lat, lon, alt);
        if (this._piActive()) return this._piPost('/command/set_home', { lat, lon, alt });
        throw new Error('Tidak ada koneksi aktif');
    },

    async getAllParams(onProgress) {
        if (this._radioActive()) return window.radioMavlink.getAllParams(onProgress);
        if (this._piActive()) {
            const resp = await fetch(window.gcsConnection.baseUrl + '/param/list');
            const data = await resp.json();
            if (data.status !== 'success') throw new Error(data.message);
            return data.params;
        }
        throw new Error('Tidak ada koneksi aktif');
    },

    async setParam(name, value, paramType = 9) {
        if (this._radioActive()) return window.radioMavlink.setParam(name, value, paramType);
        if (this._piActive()) return this._piPost('/param/set', { name, value });
        throw new Error('Tidak ada koneksi aktif');
    },

    async getPID(axis) {
        if (this._radioActive()) return window.radioMavlink.getPID(axis);
        if (this._piActive()) {
            const resp = await fetch(window.gcsConnection.baseUrl + `/param/pid/${axis}`);
            const data = await resp.json();
            if (data.status !== 'success') throw new Error(data.message);
            return data.pid;
        }
        throw new Error('Tidak ada koneksi aktif');
    },

    async setPID(axis, pid) {
        if (this._radioActive()) return window.radioMavlink.setPID(axis, pid);
        if (this._piActive()) return this._piPost(`/param/pid/${axis}`, pid);
        throw new Error('Tidak ada koneksi aktif');
    },

    async uploadMission(items) {
        if (this._radioActive()) return window.radioMavlink.uploadMission(items);
        if (this._piActive()) return this._piPost('/mission/upload', { items });
        throw new Error('Tidak ada koneksi aktif');
    },

    async downloadMission() {
        if (this._radioActive()) return window.radioMavlink.downloadMission();
        if (this._piActive()) {
            const resp = await fetch(window.gcsConnection.baseUrl + '/mission/read');
            const data = await resp.json();
            if (data.status !== 'success') throw new Error(data.message);
            return data.items;
        }
        throw new Error('Tidak ada koneksi aktif');
    },

    async startAccel3Axis() {
        if (this._radioActive()){
            window.acceptCalStep = 0;
            return window.radioMavlink.sendCommandLong(241, { p5: 1.0 });
        } 
        if (this._piActive()) return this._piPost('/command/calibrate', { type: 'accel_3axis' });
        throw new Error('Tidak ada koneksi aktif');
    },

    async startAccelLevel() {
        if (this._radioActive()) return window.radioMavlink.sendCommandLong(241, { p5: 2.0 });
        if (this._piActive()) return this._piPost('/command/calibrate', { type: 'accel_level' });
        throw new Error('Tidak ada koneksi aktif');
    },

    async startAccelSimple() {
        if (this._radioActive()) return window.radioMavlink.sendCommandLong(241, { p5: 4.0 });
        if (this._piActive()) return this._piPost('/command/calibrate', { type: 'accel_simple' });
        throw new Error('Tidak ada koneksi aktif');
    },

    async nextAccelStep() {
        if (this._radioActive()){
            window.acceptCalStep += 1;
          return window.radioMavlink.sendCommandAck(window.acceptCalStep, 1);  
        } 
        if (this._piActive()) return this._piPost('/command/calibrate/next', { position: 'next' });
        throw new Error('Tidak ada koneksi aktif');
    },

    async startCompassCal() {
        if (this._radioActive()) {
            window.compassSuccessMask = 0; // Reset memori kompas
            return window.radioMavlink.sendCommandLong(42424, { p3: 1.0 }); // 42424 = START_MAG_CAL
        }
        if (this._piActive()) return this._piPost('/command/calibrate/compass/start');
        throw new Error('Tidak ada koneksi aktif');
    },

    async cancelCompassCal() {
        if (this._radioActive()) return window.radioMavlink.sendCommandLong(42426, {}); // 42426 = CANCEL_MAG_CAL
        if (this._piActive()) return this._piPost('/command/calibrate/compass/cancel');
        throw new Error('Tidak ada koneksi aktif');
    },

    async acceptCompassCal() {
        if (this._radioActive()) {
            const mask = window.compassSuccessMask || 0;
            if (mask === 0) throw new Error("Tidak ada compass yang SUCCESS. Silakan kalibrasi ulang.");
            return window.radioMavlink.sendCommandLong(42425, { p1: mask }); // 42425 = ACCEPT_MAG_CAL
        }
        if (this._piActive()) return this._piPost('/command/calibrate/compass/accept');
        throw new Error('Tidak ada koneksi aktif');
    },
};
window.compassSuccessMask = 0;
window.commands = commands;