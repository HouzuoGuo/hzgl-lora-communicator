function decodeUplink(input) {
    var data = { payload: input.bytes };
    var i = 0;
    var buf = input.bytes;
    if (input.fPort == 119) {
        // Byte 0, 1 - number of seconds since the reception of last downlink message (0 - 65535).
        data.last_rx_sec = buf[i++];
        data.last_rx_sec += buf[i++] << 8;
        // Byte 2, 3, 4, 5 - uptime in seconds.
        data.uptime_sec = buf[i++];
        data.uptime_sec += buf[i++] << 8;
        data.uptime_sec += buf[i++] << 16;
        data.uptime_sec += buf[i++] << 24;
        // Byte 6, 7 - heap usage in KB.
        data.heap_usage_kb = buf[i++];
        data.heap_usage_kb += buf[i++] << 8;
        // Byte 8, 9 - battery voltage in millivolts.
        data.batt_millivolt = buf[i++];
        data.batt_millivolt += buf[i++] << 8;
        // Byte 10 - is battery charging (0 - false, 1 - true).
        data.is_batt_charging = buf[i++];
        // Byte 11, 12, 13, 14 - ambient temperature in celcius.
        data.ambient_temp_celcius = decode_double(buf[i], buf[i + 1], buf[i + 2], buf[i + 3]);
        i += 4;
        // Byte 15 - ambient humidity in percentage.
        data.ambient_humidity_pct = buf[i++];
        // Byte 16, 17, 18, 19 - ambient pressure in hpa.
        data.ambient_pressure_hpa = decode_double(buf[i], buf[i + 1], buf[i + 2], buf[i + 3]);
        i += 4;
        // Byte 20, 21, 22, 23 - pressure altitude in meters.
        data.ambient_altitude_metre = decode_double(buf[i], buf[i + 1], buf[i + 2], buf[i + 3]);
        i += 4;
    } else if (input.fPort == 120) {
        // Byte 0, 1, 2, 3 - GPS latitude.
        data.latitude = decode_double(buf[i], buf[i + 1], buf[i + 2], buf[i + 3]);
        i += 4;
        // Byte 4, 5, 6, 7 - GPS longitude.
        data.longitude = decode_double(buf[i], buf[i + 1], buf[i + 2], buf[i + 3]);
        i += 4;
        // Byte 8, 9 - GPS speed in km/h.
        data.gps_speed_kmh = buf[i++];
        data.gps_speed_kmh += buf[i++] << 8;
        // Byte 10, 11 - GPS heading in degrees.
        data.gps_heading_deg = buf[i++];
        data.gps_heading_deg += buf[i++] << 8;
        // Byte 12, 13, 14, 15 - GPS altitude in metres.
        data.altitude = decode_double(buf[i], buf[i + 1], buf[i + 2], buf[i + 3]);
        i += 4;
        // Byte 16, 17 - the age of last GPS fix in seconds (0 - 65536).
        data.gps_pos_age_sec = buf[i++];
        data.gps_pos_age_sec += buf[i++] << 8;
        // Byte 18 - HDOP in integer (0 - 256).
        data.hdop = buf[i++];
        // Byte 19 - number of GPS satellites in view.
        data.sats = buf[i++];
        // Byte 20 - WiFi monitor - number of inflight packets across all channels.
        data.wifi_loudest_tx_chan = buf[i++];
        // Byte 21 - WiFi monitor - the loudest sender's channel.
        data.wifi_loudest_tx_chan = buf[i++];
        // Byte 22 - WiFi monitor - the loudest sender's RSSI reading above RSSI floor (which is -100).
        data.wifi_loudest_tx_rssi = -100 + buf[i++];
        // Byte 23, 24, 25, 26, 27, 28 - WiFi monitor - the loudest sender's mac.
        data.wifi_loudest_tx_mac = buf[i].toString(16) + ':' + buf[i + 1].toString(16) + ':' + buf[i + 2].toString(16) + ':' + buf[i + 3].toString(16) + ':' + buf[i + 4].toString(16) + ':' + buf[i + 5].toString(16);
        i += 6;
    }
    return {
        data: data,
        warnings: [],
        errors: []
    };
}

function decode_double(b1, b2, b3, b4) {
    var ret = b1 + (b2 << 8) + (b3 << 16) + (b4 << 24);
    if (ret > 2147483647) {
        ret -= 1;
        ret = ~ret;
        ret &= 2147483647;
        ret = -ret;
    }
    ret /= 100000;
    return ret;
}