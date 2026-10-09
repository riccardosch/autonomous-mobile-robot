#!/usr/bin/env python3
"""Live web viewer for the LiDAR scans sent by the firmware (esp32c3 environment).

Reads the binary frames from the serial port and serves a page that draws each
scan in real time. The Pi has no screen, so the page is opened from another
computer's browser.

Usage:
    python3 scan_viewer.py [--port /dev/ttyACM0] [--host 127.0.0.1] [--http-port 8000]

With the default host the page is only reachable from the Pi itself: open it
through VS Code's port forwarding (Ports tab) or `ssh -L 8000:localhost:8000`.
Use --host 0.0.0.0 to reach it directly at http://<pi-address>:8000 from the LAN.
"""

import argparse
import json
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import serial

from read_scan import read_frames


class ScanState:
    """Latest scan and link statistics, shared between the serial and HTTP threads."""

    def __init__(self):
        self.cond = threading.Condition()
        self.version = 0
        self.data = {"connected": False, "port": None, "error": None}
        self.crc_errors = 0
        self.lost = 0
        self.last_seq = None
        self.frame_times = []

    def update(self, **fields):
        with self.cond:
            self.data.update(fields)
            self.version += 1
            self.cond.notify_all()

    def wait_newer(self, version, timeout):
        with self.cond:
            self.cond.wait_for(lambda: self.version != version, timeout)
            return self.version, json.dumps(self.data)


def serial_loop(state, port_name):
    def on_crc_error():
        state.crc_errors += 1

    while True:
        try:
            with serial.Serial(port_name, 115200, timeout=1) as port:
                state.update(connected=True, port=port_name, error=None)
                state.last_seq = None
                for seq, duration_us, distances in read_frames(port, on_crc_error):
                    if state.last_seq is not None:
                        state.lost += (seq - state.last_seq - 1) & 0xFFFF
                    state.last_seq = seq
                    now = time.monotonic()
                    state.frame_times = [t for t in state.frame_times if now - t < 2] + [now]
                    rate = (len(state.frame_times) - 1) / (now - state.frame_times[0]) if len(state.frame_times) > 1 else 0
                    state.update(
                        seq=seq,
                        scan_hz=1e6 / duration_us if duration_us else 0,
                        frames_per_s=rate,
                        distances=distances,
                        crc_errors=state.crc_errors,
                        lost=state.lost,
                    )
        except (serial.SerialException, OSError) as exc:
            state.update(connected=False, error=str(exc))
            time.sleep(2)


def make_handler(state):
    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            if self.path == "/":
                body = PAGE.encode()
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
            elif self.path == "/events":
                self.send_response(200)
                self.send_header("Content-Type", "text/event-stream")
                self.send_header("Cache-Control", "no-cache")
                self.end_headers()
                version = -1
                try:
                    while True:
                        version, payload = state.wait_newer(version, timeout=5)
                        self.wfile.write(f"data: {payload}\n\n".encode())
                        self.wfile.flush()
                except (BrokenPipeError, ConnectionResetError):
                    pass
            else:
                self.send_error(404)

        def log_message(self, *args):
            pass  # keep the terminal quiet

    return Handler


PAGE = r"""<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>LiDAR Scan Viewer</title>
<style>
  :root {
    --bg: #f6f7f9; --panel: #ffffff; --text: #1d2330; --muted: #6b7280;
    --grid: #dde1e7; --grid-label: #9aa1ad; --point: #0f7bd8; --line: rgba(15,123,216,.25);
    --robot: #1d2330; --nearest: #d93a3a; --ok: #1f9d55; --bad: #d93a3a; --border: #e3e6eb;
  }
  @media (prefers-color-scheme: dark) {
    :root {
      --bg: #0f1218; --panel: #171b23; --text: #e6e8ec; --muted: #8b93a1;
      --grid: #262c37; --grid-label: #5d6573; --point: #4fb3ff; --line: rgba(79,179,255,.25);
      --robot: #e6e8ec; --nearest: #ff6b6b; --ok: #3ccf7a; --bad: #ff6b6b; --border: #262c37;
    }
  }
  * { box-sizing: border-box; }
  body { margin: 0; background: var(--bg); color: var(--text);
         font: 14px/1.4 system-ui, -apple-system, "Segoe UI", sans-serif; }
  main { display: flex; gap: 16px; padding: 16px; height: 100vh; }
  #plot { flex: 1; min-width: 0; background: var(--panel); border: 1px solid var(--border);
          border-radius: 10px; position: relative; }
  canvas { width: 100%; height: 100%; display: block; cursor: crosshair; }
  #hover { position: absolute; top: 12px; left: 12px; font-variant-numeric: tabular-nums;
           color: var(--muted); pointer-events: none; }
  aside { width: 260px; flex-shrink: 0; display: flex; flex-direction: column; gap: 12px; }
  .card { background: var(--panel); border: 1px solid var(--border); border-radius: 10px; padding: 14px; }
  h1 { font-size: 16px; margin: 0 0 4px; }
  .status { display: flex; align-items: center; gap: 8px; color: var(--muted); }
  .dot { width: 9px; height: 9px; border-radius: 50%; background: var(--bad); }
  .dot.on { background: var(--ok); }
  dl { margin: 0; display: grid; grid-template-columns: 1fr auto; gap: 6px 12px; }
  dt { color: var(--muted); }
  dd { margin: 0; text-align: right; font-variant-numeric: tabular-nums; font-weight: 600; }
  label { display: flex; justify-content: space-between; align-items: center; gap: 8px; margin: 6px 0; }
  select, button { font: inherit; color: var(--text); background: var(--bg);
                   border: 1px solid var(--border); border-radius: 6px; padding: 4px 8px; }
  button { width: 100%; margin-top: 6px; cursor: pointer; }
  .hint { color: var(--muted); font-size: 12px; margin: 8px 0 0; }
  @media (max-width: 700px) {
    main { flex-direction: column; height: auto; }
    #plot { height: 80vw; }
    aside { width: auto; }
  }
</style>
</head>
<body>
<main>
  <div id="plot"><canvas id="c"></canvas><div id="hover"></div></div>
  <aside>
    <div class="card">
      <h1>LiDAR Scan Viewer</h1>
      <div class="status"><span class="dot" id="dot"></span><span id="status">connecting…</span></div>
    </div>
    <div class="card">
      <dl>
        <dt>Scan rate</dt><dd id="hz">–</dd>
        <dt>Frames received</dt><dd id="fps">–</dd>
        <dt>Valid bins</dt><dd id="valid">–</dd>
        <dt>Nearest</dt><dd id="nearest">–</dd>
        <dt>Sequence</dt><dd id="seq">–</dd>
        <dt>Lost frames</dt><dd id="lost">–</dd>
        <dt>CRC errors</dt><dd id="crc">–</dd>
      </dl>
    </div>
    <div class="card">
      <label>Range
        <select id="range">
          <option value="0">auto</option><option value="500">0.5 m</option>
          <option value="1000">1 m</option><option value="2000">2 m</option>
          <option value="4000">4 m</option><option value="8000">8 m</option>
        </select>
      </label>
      <label>Connect points <input type="checkbox" id="lines"></label>
      <label>Show nearest <input type="checkbox" id="shownearest" checked></label>
      <button id="pause">Pause</button>
      <p class="hint">0° points up, angles grow clockwise (YDLIDAR convention, to be checked on the robot). Mouse wheel zooms.</p>
    </div>
  </aside>
</main>
<script>
const $ = id => document.getElementById(id);
const canvas = $("c"), ctx = canvas.getContext("2d");
let scan = null, paused = false, rangeMm = 0, autoRange = 4000, mouse = null;

function css(name) { return getComputedStyle(document.documentElement).getPropertyValue(name).trim(); }

function niceStep(range) {
  const raw = range / 4, p = Math.pow(10, Math.floor(Math.log10(raw)));
  return [1, 2, 5, 10].map(m => m * p).find(s => s >= raw);
}

// Screen position of a point: 0 deg up, clockwise.
function toXY(angleDeg, d, cx, cy, scale) {
  const a = angleDeg * Math.PI / 180;
  return [cx + Math.sin(a) * d * scale, cy - Math.cos(a) * d * scale];
}

function draw() {
  const dpr = window.devicePixelRatio || 1;
  const w = canvas.clientWidth, h = canvas.clientHeight;
  if (canvas.width !== w * dpr || canvas.height !== h * dpr) { canvas.width = w * dpr; canvas.height = h * dpr; }
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  ctx.clearRect(0, 0, w, h);

  const cx = w / 2, cy = h / 2, radius = Math.min(w, h) / 2 - 24;
  const range = rangeMm || autoRange, scale = radius / range;

  // Range rings and spokes
  const step = niceStep(range);
  ctx.strokeStyle = css("--grid"); ctx.fillStyle = css("--grid-label");
  ctx.lineWidth = 1; ctx.font = "11px system-ui"; ctx.textAlign = "left";
  for (let r = step; r <= range + 1e-6; r += step) {
    ctx.beginPath(); ctx.arc(cx, cy, r * scale, 0, 2 * Math.PI); ctx.stroke();
    ctx.fillText(r >= 1000 ? (r / 1000) + " m" : r + " mm", cx + 4, cy - r * scale - 4);
  }
  ctx.textAlign = "center";
  for (let a = 0; a < 360; a += 30) {
    const [x, y] = toXY(a, range, cx, cy, scale);
    ctx.beginPath(); ctx.moveTo(cx, cy); ctx.lineTo(x, y); ctx.stroke();
    const [lx, ly] = toXY(a, range + 14 / scale, cx, cy, scale);
    ctx.fillText(a + "°", lx, ly + 4);
  }

  // Robot marker, pointing to 0 deg
  ctx.fillStyle = css("--robot");
  ctx.beginPath(); ctx.moveTo(cx, cy - 10); ctx.lineTo(cx - 6, cy + 6); ctx.lineTo(cx + 6, cy + 6); ctx.closePath(); ctx.fill();

  if (!scan || !scan.distances) return;
  const d = scan.distances, n = d.length, degPerBin = 360 / n;

  if ($("lines").checked) {
    ctx.strokeStyle = css("--line"); ctx.lineWidth = 1.5; ctx.beginPath();
    let pen = false;
    for (let i = 0; i <= n; i++) {
      const v = d[i % n];
      if (!v) { pen = false; continue; }
      const [x, y] = toXY(i * degPerBin, v, cx, cy, scale);
      pen ? ctx.lineTo(x, y) : ctx.moveTo(x, y); pen = true;
    }
    ctx.stroke();
  }

  ctx.fillStyle = css("--point");
  for (let i = 0; i < n; i++) {
    if (!d[i] || d[i] > range) continue;
    const [x, y] = toXY(i * degPerBin, d[i], cx, cy, scale);
    ctx.fillRect(x - 2, y - 2, 4, 4);
  }

  if ($("shownearest").checked && scan.nearestIdx >= 0) {
    const i = scan.nearestIdx;
    const [x, y] = toXY(i * degPerBin, d[i], cx, cy, scale);
    ctx.strokeStyle = css("--nearest"); ctx.lineWidth = 2;
    ctx.beginPath(); ctx.arc(x, y, 7, 0, 2 * Math.PI); ctx.stroke();
  }

  // Hover readout: the bin under the mouse
  if (mouse) {
    const dx = mouse.x - cx, dy = mouse.y - cy;
    let ang = Math.atan2(dx, -dy) * 180 / Math.PI; if (ang < 0) ang += 360;
    const i = Math.round(ang / degPerBin) % n;
    $("hover").textContent = `${(i * degPerBin).toFixed(0)}°  ${d[i] ? d[i] + " mm" : "no echo"}`;
  } else {
    $("hover").textContent = "";
  }
}

function onScan(msg) {
  $("dot").classList.toggle("on", msg.connected);
  $("status").textContent = msg.connected ? msg.port : ("no serial: " + (msg.error || "waiting"));
  if (!msg.distances || paused) return;

  const d = msg.distances;
  let valid = 0, nearestIdx = -1, maxD = 0;
  const sorted = [];
  d.forEach((v, i) => {
    if (!v) return;
    valid++; sorted.push(v);
    if (nearestIdx < 0 || v < d[nearestIdx]) nearestIdx = i;
  });
  // Auto range: 95th percentile, so a few far points do not shrink everything
  if (sorted.length) {
    sorted.sort((a, b) => a - b);
    const p95 = sorted[Math.floor(sorted.length * 0.95)];
    autoRange = Math.max(500, Math.ceil(p95 * 1.1 / 500) * 500);
  }
  scan = { distances: d, nearestIdx };

  $("hz").textContent = msg.scan_hz.toFixed(1) + " Hz";
  $("fps").textContent = msg.frames_per_s.toFixed(1) + " /s";
  $("valid").textContent = `${valid}/${d.length}`;
  $("nearest").textContent = nearestIdx >= 0 ? `${d[nearestIdx]} mm @ ${Math.round(nearestIdx * 360 / d.length)}°` : "–";
  $("seq").textContent = msg.seq;
  $("lost").textContent = msg.lost;
  $("crc").textContent = msg.crc_errors;
}

const es = new EventSource("/events");
es.onmessage = e => onScan(JSON.parse(e.data));
es.onerror = () => { $("dot").classList.remove("on"); $("status").textContent = "viewer server not reachable"; };

$("range").onchange = e => rangeMm = +e.target.value;
$("pause").onclick = () => { paused = !paused; $("pause").textContent = paused ? "Resume" : "Pause"; };
canvas.addEventListener("mousemove", e => { const r = canvas.getBoundingClientRect(); mouse = { x: e.clientX - r.left, y: e.clientY - r.top }; });
canvas.addEventListener("mouseleave", () => mouse = null);
canvas.addEventListener("wheel", e => {
  e.preventDefault();
  rangeMm = Math.min(12000, Math.max(200, Math.round((rangeMm || autoRange) * (e.deltaY > 0 ? 1.15 : 1 / 1.15))));
  $("range").value = "";
}, { passive: false });

(function loop() { draw(); requestAnimationFrame(loop); })();
</script>
</body>
</html>
"""


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--port", default="/dev/ttyACM0", help="serial port of the ESP32")
    parser.add_argument("--host", default="127.0.0.1", help="address to listen on (0.0.0.0 = whole LAN)")
    parser.add_argument("--http-port", type=int, default=8000)
    args = parser.parse_args()

    state = ScanState()
    threading.Thread(target=serial_loop, args=(state, args.port), daemon=True).start()

    server = ThreadingHTTPServer((args.host, args.http_port), make_handler(state))
    server.daemon_threads = True
    print(f"Scan viewer on http://{args.host}:{args.http_port}  (Ctrl+C to stop)")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
