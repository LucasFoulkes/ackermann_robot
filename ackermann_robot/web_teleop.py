#!/usr/bin/env python3
"""Web teleop: dual spring-back arc sliders -> /ackermann/cmd_effort."""

from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import threading
from threading import Lock
from urllib.parse import parse_qs, urlparse
import json
import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


HTML = """<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=no, viewport-fit=cover">
<meta name="apple-mobile-web-app-capable" content="yes">
<meta name="mobile-web-app-capable" content="yes">
<meta name="apple-mobile-web-app-status-bar-style" content="black-translucent">
<meta name="theme-color" content="#121212">
<meta http-equiv="Cache-Control" content="no-store">
<title>Ackermann</title>
<style>
  html, body {
    margin: 0; padding: 0;
    background: #121212;
    overflow: hidden;
    touch-action: none;
    overscroll-behavior: none;
    -webkit-user-select: none; user-select: none;
    position: fixed; inset: 0;
    font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif;
  }
  canvas { display: block; width: 100%; height: 100%; }
  #fsBtn {
    position: fixed;
    top: calc(env(safe-area-inset-top, 0px) + 10px);
    left: 50%;
    transform: translateX(-50%);
    background: #2a2a2a;
    color: #fff;
    border: 1px solid #444;
    border-radius: 999px;
    padding: 8px 14px;
    font-size: 14px;
    line-height: 1;
    z-index: 10;
    cursor: pointer;
    -webkit-tap-highlight-color: transparent;
    touch-action: manipulation;
  }
  #fsBtn:active { background: #3a3a3a; }
  #fsBtn.hidden { display: none; }
</style>
</head>
<body>
<canvas id="c"></canvas>
<button id="fsBtn" aria-label="Fullscreen">Fullscreen</button>
<script>
  // Screenfull-compatible cross-browser fullscreen helper (inlined, no CDN).
  const fs = (() => {
    const doc = document;
    const root = doc.documentElement;
    const sets = [
      ['requestFullscreen','exitFullscreen','fullscreenElement','fullscreenchange'],
      ['webkitRequestFullscreen','webkitExitFullscreen','webkitFullscreenElement','webkitfullscreenchange'],
      ['msRequestFullscreen','msExitFullscreen','msFullscreenElement','MSFullscreenChange'],
      ['mozRequestFullScreen','mozCancelFullScreen','mozFullScreenElement','mozfullscreenchange']
    ];
    const k = sets.find(s => s[0] in root) || null;
    return {
      supported: !!k,
      request(target) { return k ? (target || root)[k[0]]() : null; },
      exit()          { return k ? doc[k[1]]() : null; },
      get isFullscreen() { return !!(k && doc[k[2]]); },
      onChange(cb)    { if (k) doc.addEventListener(k[3], cb); },
      toggle(target)  { return this.isFullscreen ? this.exit() : this.request(target); }
    };
  })();

  const canvas = document.getElementById('c');
  const ctx = canvas.getContext('2d');
  const fsBtn = document.getElementById('fsBtn');

  const state = {
    left: 0.5, right: 0.5,
    isLeftHeld: false, isRightHeld: false,
    activeSlider: null
  };
  const config = { thumbRadius: 32, trackWidth: 60, springStrength: 0.2, pad: 12 };

  let W = 0, H = 0, sq = 0;

  function resize() {
    const rect = canvas.getBoundingClientRect();
    const dpr = window.devicePixelRatio || 1;
    W = rect.width;
    H = rect.height;
    canvas.width = Math.round(W * dpr);
    canvas.height = Math.round(H * dpr);
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    sq = Math.max(80, Math.min(W * 0.45, H * 0.9) - config.pad);
  }

  function drawArch(cx, cy, value, side) {
    const r = sq;
    const isLeft = side === 'left';
    const startAngle = isLeft ? 0 : -Math.PI;
    const endAngle = -Math.PI / 2;
    const currentAngle = startAngle + value * (endAngle - startAngle);
    const centerAngle = startAngle + 0.5 * (endAngle - startAngle);

    ctx.beginPath();
    ctx.lineWidth = config.trackWidth;
    ctx.lineCap = 'round';
    ctx.strokeStyle = '#333';
    ctx.arc(cx, cy, r, startAngle, endAngle, isLeft);
    ctx.stroke();

    ctx.beginPath();
    ctx.strokeStyle = '#ccc';
    ctx.arc(cx, cy, r, centerAngle, currentAngle, currentAngle < centerAngle);
    ctx.stroke();

    const tx = cx + Math.cos(currentAngle) * r;
    const ty = cy + Math.sin(currentAngle) * r;
    ctx.beginPath();
    ctx.fillStyle = '#fff';
    ctx.arc(tx, ty, config.thumbRadius, 0, Math.PI * 2);
    ctx.fill();
  }

  function draw() {
    ctx.clearRect(0, 0, W, H);
    drawArch(0, H, state.left, 'left');
    drawArch(W, H, state.right, 'right');
  }

  function handleInteraction(x, y, targetSide) {
    const side = targetSide || (x < W / 2 ? 'left' : 'right');
    const cx = side === 'left' ? 0 : W;
    const cy = H;
    const angle = Math.atan2(y - cy, x - cx);
    let val;
    if (side === 'left') {
      let a = angle;
      if (a > 0) a = 0;
      if (a < -Math.PI / 2) a = -Math.PI / 2;
      val = a / (-Math.PI / 2);
    } else {
      let a = angle;
      if (a > -Math.PI / 2 && a <= 0) a = -Math.PI / 2;
      else if (a > 0 || a < -Math.PI) a = -Math.PI;
      val = (a + Math.PI) / (Math.PI / 2);
    }
    state[side] = Math.max(0, Math.min(1, val));
  }

  function update() {
    if (!state.isLeftHeld) {
      state.left += (0.5 - state.left) * config.springStrength;
      if (Math.abs(state.left - 0.5) < 0.001) state.left = 0.5;
    }
    if (!state.isRightHeld) {
      state.right += (0.5 - state.right) * config.springStrength;
      if (Math.abs(state.right - 0.5) < 0.001) state.right = 0.5;
    }
    draw();
    requestAnimationFrame(update);
  }

  // Fullscreen button
  const clickFs = (e) => {
    e.preventDefault();
    e.stopPropagation();
    try { fs.toggle(); } catch (_) {}
    if (screen.orientation && screen.orientation.lock) {
      screen.orientation.lock('landscape').catch(() => {});
    }
  };
  fsBtn.addEventListener('click', clickFs);
  fsBtn.addEventListener('touchend', (e) => { clickFs(e); }, { passive: false });
  fs.onChange(() => {
    fsBtn.textContent = fs.isFullscreen ? 'Exit' : 'Fullscreen';
    setTimeout(resize, 50);
  });
  if (!fs.supported) {
    // iOS Safari path: no API. Hint user to add to home screen.
    fsBtn.textContent = 'Add to Home Screen';
    fsBtn.addEventListener('click', () => {
      fsBtn.textContent = 'Share → Add to Home Screen';
    });
  }

  // Sliders (mouse)
  canvas.addEventListener('mousedown', (e) => {
    const side = e.clientX < W / 2 ? 'left' : 'right';
    state.activeSlider = side;
    if (side === 'left') state.isLeftHeld = true; else state.isRightHeld = true;
    handleInteraction(e.clientX, e.clientY, side);
  });
  window.addEventListener('mousemove', (e) => {
    if (state.activeSlider) handleInteraction(e.clientX, e.clientY, state.activeSlider);
  });
  window.addEventListener('mouseup', () => {
    state.isLeftHeld = false; state.isRightHeld = false; state.activeSlider = null;
  });

  // Sliders (touch)
  canvas.addEventListener('touchstart', (e) => {
    e.preventDefault();
    for (const t of e.touches) {
      const side = t.clientX < W / 2 ? 'left' : 'right';
      if (side === 'left') state.isLeftHeld = true; else state.isRightHeld = true;
      handleInteraction(t.clientX, t.clientY, side);
    }
  }, { passive: false });
  canvas.addEventListener('touchmove', (e) => {
    e.preventDefault();
    let l = false, r = false;
    for (const t of e.touches) {
      const side = t.clientX < W / 2 ? 'left' : 'right';
      if (side === 'left') l = true; else r = true;
      handleInteraction(t.clientX, t.clientY, side);
    }
    state.isLeftHeld = l; state.isRightHeld = r;
  }, { passive: false });
  const endTouch = (e) => {
    let l = false, r = false;
    for (const t of e.touches) {
      const side = t.clientX < W / 2 ? 'left' : 'right';
      if (side === 'left') l = true; else r = true;
    }
    state.isLeftHeld = l; state.isRightHeld = r;
  };
  canvas.addEventListener('touchend', endTouch);
  canvas.addEventListener('touchcancel', endTouch);

  window.addEventListener('resize', resize);
  window.addEventListener('orientationchange', () => setTimeout(resize, 100));
  if (window.visualViewport) {
    window.visualViewport.addEventListener('resize', resize);
  }

  resize();
  update();

  // Publish: left=throttle, right=steer, 20 Hz (always — keeps driver watchdog alive).
  setInterval(() => {
    const throttle = Math.round((state.left - 0.5) * 200);
    const steer = Math.round((state.right - 0.5) * 200);
    fetch('/cmd?steer=' + steer + '&throttle=' + throttle).catch(() => {});
  }, 50);
</script>
</body>
</html>
"""


def clamp(value, lo=-1.0, hi=1.0):
    try:
        value = float(value)
    except (TypeError, ValueError):
        return 0.0
    if not math.isfinite(value):
        return 0.0
    return max(lo, min(hi, value))


class WebTeleopNode(Node):
    def __init__(self):
        super().__init__('web_teleop')
        self.cmd_topic = str(self.declare_parameter('cmd_topic', '/ackermann/cmd_effort').value)
        self.host = str(self.declare_parameter('host', '0.0.0.0').value)
        self.port = int(self.declare_parameter('port', 8080).value)
        self._pub = self.create_publisher(Float32MultiArray, self.cmd_topic, 10)
        self._lock = Lock()

    def publish_cmd(self, steer_pct, throttle_pct):
        steer = clamp(steer_pct / 100.0)
        throttle = clamp(throttle_pct / 100.0)
        msg = Float32MultiArray()
        msg.data = [steer, throttle]
        with self._lock:
            self._pub.publish(msg)
        return steer, throttle


class TeleopServer(ThreadingHTTPServer):
    daemon_threads = True

    def __init__(self, address, handler_cls, node):
        super().__init__(address, handler_cls)
        self.node = node


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *a, **k):
        pass

    def _send(self, code, body, ctype):
        self.send_response(code)
        self.send_header('Content-Type', ctype)
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        url = urlparse(self.path)
        if url.path in ('/', '/index.html'):
            self._send(200, HTML.encode(), 'text/html; charset=utf-8')
            return
        if url.path == '/cmd':
            q = parse_qs(url.query)
            try:
                steer = float(q.get('steer', ['0'])[0])
                throttle = float(q.get('throttle', ['0'])[0])
            except ValueError:
                self.send_error(400, 'Invalid steer/throttle')
                return
            s, t = self.server.node.publish_cmd(steer, throttle)
            self._send(200, json.dumps({'steer': s, 'throttle': t}).encode(), 'application/json')
            return
        self.send_error(404)


def main(args=None):
    rclpy.init(args=args)
    node = WebTeleopNode()
    server = TeleopServer((node.host, node.port), Handler, node)
    node.get_logger().info(f'web_teleop serving on http://{node.host}:{node.port}/')
    spin = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin.start()
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
