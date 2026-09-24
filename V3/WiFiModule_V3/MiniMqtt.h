// Minimal MQTT 3.1.1 client: QoS 0 publish/subscribe, keep-alive, last will.
// Built in so the firmware needs no extra libraries. connect() only opens the
// socket and sends CONNECT; loop() finishes the handshake without blocking.
#pragma once
#include <Arduino.h>
#include <Client.h>

class MiniMqtt {
 public:
  enum State : uint8_t { DOWN, WAIT_ACK, UP };
  // lastError(): broker CONNACK code (1..5), ERR_TIMEOUT or ERR_NET
  static const uint8_t ERR_TIMEOUT = 0xFE, ERR_NET = 0xFF;
  using Handler = void (*)(const char* topic, const uint8_t* payload, size_t len);

  void onMessage(Handler h) { _handler = h; }
  State state() const { return _state; }
  uint8_t lastError() const { return _err; }

  bool connect(Client& c, const char* host, uint16_t port, const char* id, const char* user,
               const char* pass, const char* willTopic, const char* willMsg) {
    disconnect();
    _c = &c;
    if (!_c->connect(host, port)) {
      drop(ERR_NET);
      return false;
    }
    const bool hasUser = user && *user, hasPass = pass && *pass;
    static const uint8_t proto[] = {0, 4, 'M', 'Q', 'T', 'T', 4};
    memcpy(body(), proto, sizeof(proto));
    size_t n = sizeof(proto);
    body()[n++] = 0x02 | (willTopic ? 0x24 : 0) | (hasUser ? 0x80 : 0) | (hasPass ? 0x40 : 0);
    body()[n++] = KEEPALIVE_S >> 8;
    body()[n++] = KEEPALIVE_S & 0xFF;
    n = putStr(n, id);
    if (willTopic) n = putStr(putStr(n, willTopic), willMsg);
    if (hasUser) n = putStr(n, user);
    if (hasPass) n = putStr(n, pass);
    if (n == BAD || !send(0x10, n)) {
      drop(ERR_NET);
      return false;
    }
    _state = WAIT_ACK;
    _ps = 0;
    _lastRx = _lastPing = millis();
    return true;
  }

  void disconnect() {
    if (_state == UP) send(0xE0, 0);  // graceful: broker skips the last will
    if (_c) _c->stop();
    _state = DOWN;
  }

  bool publish(const char* topic, const uint8_t* payload, size_t len, bool retain = false) {
    if (_state != UP) return false;
    const size_t n = putStr(0, topic);
    if (n == BAD || n + len > BODY_MAX) return false;
    memcpy(body() + n, payload, len);
    return send(retain ? 0x31 : 0x30, n + len);
  }

  bool publish(const char* topic, const char* s, bool retain = false) {
    return publish(topic, (const uint8_t*)s, strlen(s), retain);
  }

  bool subscribe(const char* topic) {
    if (_state != UP) return false;
    _pid = _pid % 65535 + 1;
    body()[0] = _pid >> 8;
    body()[1] = _pid & 0xFF;
    size_t n = putStr(2, topic);
    if (n == BAD || n >= BODY_MAX) return false;
    body()[n++] = 0;  // requested QoS
    return send(0x82, n);
  }

  void loop() {
    if (!_c || _state == DOWN) return;
    if (!_c->connected() && !_c->available()) {
      drop(ERR_NET);
      return;
    }
    for (int budget = 768; budget > 0 && _c->available(); budget--) {
      const int b = _c->read();
      if (b < 0) break;
      feed((uint8_t)b);
      if (_state == DOWN) return;
    }
    const uint32_t now = millis();
    const uint32_t half = KEEPALIVE_S * 500UL;
    if (_state == WAIT_ACK) {
      if (now - _lastRx > 8000) drop(ERR_TIMEOUT);
    } else if (now - _lastRx > half * 3) {
      drop(ERR_TIMEOUT);
    } else if ((now - _lastTx > half || now - _lastRx > half) && now - _lastPing > half) {
      _lastPing = now;
      send(0xC0, 0);
    }
  }

 private:
  static const uint16_t KEEPALIVE_S = 30;
  static const size_t BODY_MAX = 512, RX_MAX = 512;

  Client* _c = nullptr;
  Handler _handler = nullptr;
  State _state = DOWN;
  uint8_t _err = 0;
  uint16_t _pid = 0;
  uint32_t _lastRx = 0, _lastTx = 0, _lastPing = 0;
  uint8_t _tx[5 + BODY_MAX];  // [0..4] header space, body at 5
  uint8_t _rx[RX_MAX];
  // incoming packet parser
  uint8_t _ps = 0, _hdr = 0;
  uint32_t _rem = 0, _mul = 1, _pos = 0;

  uint8_t* body() { return _tx + 5; }

  // Appends a length-prefixed string at body offset n. Returns the new offset,
  // or BAD (sticky) when it does not fit.
  static const size_t BAD = SIZE_MAX;
  size_t putStr(size_t n, const char* s) {
    if (n == BAD) return BAD;
    const size_t l = strlen(s);
    if (n + 2 + l > BODY_MAX) return BAD;
    body()[n] = l >> 8;
    body()[n + 1] = l & 0xFF;
    memcpy(body() + n + 2, s, l);
    return n + 2 + l;
  }

  bool send(uint8_t type, size_t n) {
    if (!_c || n > BODY_MAX) return false;
    uint8_t len[4];
    uint8_t k = 0;
    size_t v = n;
    do {
      const uint8_t d = v % 128;
      v /= 128;
      len[k++] = v ? (d | 128) : d;
    } while (v && k < 4);
    const size_t start = 4 - k;
    _tx[start] = type;
    memcpy(_tx + start + 1, len, k);
    const size_t total = 1 + k + n;
    if (_c->write(_tx + start, total) != total) {
      drop(ERR_NET);
      return false;
    }
    _lastTx = millis();
    return true;
  }

  void drop(uint8_t err) {
    _err = err;
    if (_c) _c->stop();
    _state = DOWN;
  }

  void feed(uint8_t b) {
    switch (_ps) {
      case 0:
        _hdr = b;
        _rem = 0;
        _mul = 1;
        _ps = 1;
        break;
      case 1:
        _rem += (b & 127) * _mul;
        _mul *= 128;
        if (b & 128) {
          if (_mul > 2097152UL) drop(ERR_NET);
        } else if (_rem == 0) {
          _ps = 0;
          packet();
        } else {
          _pos = 0;
          _ps = 2;
        }
        break;
      default:
        if (_pos < RX_MAX) _rx[_pos] = b;
        if (++_pos == _rem) {
          _ps = 0;
          if (_rem <= RX_MAX) packet();  // oversized packets are skipped
        }
        break;
    }
  }

  void packet() {
    _lastRx = millis();
    switch (_hdr >> 4) {
      case 2:  // CONNACK
        if (_rem >= 2 && _rx[1] == 0) {
          _state = UP;
          _err = 0;
        } else {
          drop(_rem >= 2 ? _rx[1] : ERR_NET);
        }
        break;
      case 3: {  // PUBLISH
        if (_rem < 2 || !_handler) break;
        const size_t tl = ((size_t)_rx[0] << 8) | _rx[1];
        size_t p = 2 + tl + (((_hdr >> 1) & 3) ? 2 : 0);
        char topic[128];
        if (p > _rem || tl >= sizeof(topic)) break;
        memcpy(topic, _rx + 2, tl);
        topic[tl] = 0;
        _handler(topic, _rx + p, _rem - p);
        break;
      }
      default:  // SUBACK, PINGRESP: nothing to do
        break;
    }
  }
};
