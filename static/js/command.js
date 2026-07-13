// ===== Method tambahan =====

/**
 * Reboot Flight Controller
 * MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN = 246, param1=1 (reboot)
 */
async reboot() {
    return this.commandLong(246, { p1: 1 });
};

/**
 * Set servo PWM (MAV_CMD_DO_SET_SERVO = 183)
 * @param {number} channel - nomor channel (1-14)
 * @param {number} pwm - nilai PWM dalam microsecond (1000-2000)
 */
async setServo(channel, pwm) {
    return this.commandLong(183, { p1: channel, p2: pwm });
};

/**
 * Set Home position (MAV_CMD_DO_SET_HOME = 179)
 * @param {number} lat - latitude dalam derajat
 * @param {number} lon - longitude dalam derajat
 * @param {number} alt - altitude dalam meter (opsional)
 */
async setHome(lat, lon, alt = 0) {
    return this.commandLong(179, { p1: 0, p5: lat, p6: lon, p7: alt });
}

/**
 * Ambil semua parameter (PARAM_REQUEST_LIST)
 * Untuk radio, kita gunakan implementasi di radio-mavlink-extras.js
 * Untuk backend, panggil endpoint /param/list
 */
async getAllParams() {
    if (this._mode === 'radio') {
        if (!this._radio) throw new Error('Radio tidak tersedia');
        return this._radio.getAllParams();
    }
    const resp = await fetch(`${this.baseUrl}/param/list`);
    const data = await resp.json();
    if (data.status !== 'success') throw new Error(data.message);
    return data.params;
}