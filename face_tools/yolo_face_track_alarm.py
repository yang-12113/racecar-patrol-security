import argparse
import copy
import errno
import hashlib
import json
import os
import shutil
import subprocess
import tempfile
import threading
import time
import http.client
import urllib.error
import urllib.parse
import urllib.request
from collections import Counter, deque
from datetime import datetime
from http import server
from socketserver import ThreadingMixIn

import cv2
import numpy as np


FOLLOW_STATE_SCHEMA_VERSION = 1


def monotonic_ms():
    return int(time.monotonic_ns() // 1_000_000)


def canonical_json_bytes(data):
    return json.dumps(data, ensure_ascii=False, sort_keys=True, separators=(",", ":")).encode("utf-8")


def apply_checksum(data):
    payload = copy.deepcopy(data)
    payload.setdefault("meta", {})
    payload["meta"]["checksum_sha256"] = ""
    payload["meta"]["checksum_sha256"] = hashlib.sha256(canonical_json_bytes(payload)).hexdigest()
    return payload


def letterbox(im, new_shape=(640, 640), color=(114, 114, 114)):
    h, w = im.shape[:2]
    r = min(new_shape[0] / h, new_shape[1] / w)
    new_unpad = (int(round(w * r)), int(round(h * r)))
    dw, dh = new_shape[1] - new_unpad[0], new_shape[0] - new_unpad[1]
    dw /= 2
    dh /= 2
    if (w, h) != new_unpad:
        im = cv2.resize(im, new_unpad, interpolation=cv2.INTER_LINEAR)
    top, bottom = int(round(dh - 0.1)), int(round(dh + 0.1))
    left, right = int(round(dw - 0.1)), int(round(dw + 0.1))
    im = cv2.copyMakeBorder(im, top, bottom, left, right, cv2.BORDER_CONSTANT, value=color)
    return im, r, (dw, dh)


def l2_normalize(v):
    n = np.linalg.norm(v)
    if n < 1e-12:
        return v
    return v / n


def load_backend(model, device_id):
    errs = []
    try:
        from ais_bench.infer.interface import InferSession

        sess = InferSession(device_id, model)
        return "ais_bench", sess
    except Exception as e:
        errs.append(f"ais_bench: {e}")

    try:
        import aclruntime

        try:
            sess = aclruntime.InferenceSession(model, device_id)
        except TypeError:
            sess = aclruntime.InferenceSession(model, device_id, 0)
        return "aclruntime", sess
    except Exception as e:
        errs.append(f"aclruntime: {e}")

    raise RuntimeError("No inference backend available. " + " | ".join(errs))


def detect_model_input_size(sess):
    """Return the fixed square input size when the backend exposes one."""
    try:
        inputs = sess.get_inputs()
    except Exception:
        return None
    if not inputs:
        return None
    shape = getattr(inputs[0], "shape", None)
    if not isinstance(shape, (list, tuple)) or len(shape) != 4:
        return None
    try:
        n, c, h, w = [int(v) for v in shape]
    except Exception:
        return None
    if n != 1 or c not in (1, 3) or h <= 0 or w <= 0 or h != w:
        return None
    return h


def resolve_v4l2_device(camera):
    if isinstance(camera, int):
        return f"/dev/video{camera}"
    text = str(camera).strip()
    if text.isdigit():
        return f"/dev/video{text}"
    if text.startswith("/dev/video"):
        return text
    if text.startswith("/dev/") and os.path.exists(text):
        try:
            return os.path.realpath(text)
        except Exception:
            return text
    return None


def apply_camera_runtime_controls(device, power_line_frequency=None, exposure_auto_priority=None):
    if not device or not shutil.which("v4l2-ctl"):
        return

    ctrls = []
    if power_line_frequency is not None:
        ctrls.append(f"power_line_frequency={int(power_line_frequency)}")
    if exposure_auto_priority is not None:
        ctrls.append(f"exposure_auto_priority={int(exposure_auto_priority)}")
    if not ctrls:
        return

    cmd = ["v4l2-ctl", "-d", device, "--set-ctrl=" + ",".join(ctrls)]
    try:
        subprocess.run(cmd, check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        print(f"[CAM] applied runtime controls on {device}: {', '.join(ctrls)}")
    except Exception as e:
        print(f"[CAM] failed to apply runtime controls on {device}: {e}")


class LatestFrameStore:
    def __init__(self):
        self._cv = threading.Condition()
        self._frame_id = -1
        self._jpeg = None
        self._meta = {}
        self._closed = False

    def update(self, frame_id, jpeg_bytes, metadata=None):
        with self._cv:
            self._frame_id = int(frame_id)
            self._jpeg = jpeg_bytes
            self._meta = dict(metadata or {})
            self._cv.notify_all()

    def latest(self):
        with self._cv:
            return self._frame_id, self._jpeg, self._closed, dict(self._meta)

    def wait_next(self, last_frame_id=-1, timeout=1.0):
        with self._cv:
            end = time.time() + max(0.0, float(timeout))
            while not self._closed and (self._jpeg is None or self._frame_id == last_frame_id):
                remaining = end - time.time()
                if remaining <= 0:
                    break
                self._cv.wait(remaining)
            return self._frame_id, self._jpeg, self._closed, dict(self._meta)

    def close(self):
        with self._cv:
            self._closed = True
            self._cv.notify_all()


class ThreadedHTTPServer(ThreadingMixIn, server.HTTPServer):
    daemon_threads = True
    allow_reuse_address = True


class MjpegHandler(server.BaseHTTPRequestHandler):
    server_version = "FaceTrackMJPEG/1.0"

    def do_GET(self):
        if self.path in ("/", "/index.html"):
            body = (
                "<html><head><meta charset='utf-8'><title>Racecar Camera Stream</title>"
                "<meta name='viewport' content='width=device-width, initial-scale=1' />"
                "<style>"
                "body{margin:0;background:linear-gradient(180deg,#0f1115 0%,#171b22 100%);color:#eef2f6;"
                "font-family:'Segoe UI',sans-serif;}"
                ".wrap{max-width:1180px;margin:0 auto;padding:18px;}"
                ".hero{display:flex;justify-content:space-between;align-items:center;gap:16px;flex-wrap:wrap;"
                "margin-bottom:16px;}"
                ".title{font-size:28px;font-weight:700;letter-spacing:.3px;}"
                ".sub{font-size:14px;opacity:.78;margin-top:6px;}"
                ".badge{display:inline-block;padding:6px 10px;border-radius:999px;background:#1f2937;"
                "color:#8de1ff;font-size:12px;font-weight:600;letter-spacing:.4px;}"
                ".grid{display:grid;grid-template-columns:2fr 1fr;gap:16px;align-items:start;}"
                ".card{background:rgba(255,255,255,.04);border:1px solid rgba(255,255,255,.08);"
                "border-radius:18px;overflow:hidden;box-shadow:0 24px 50px rgba(0,0,0,.28);}"
                ".card img{display:block;width:100%;height:auto;background:#000;}"
                ".links{display:flex;gap:10px;flex-wrap:wrap;padding:14px 16px;border-top:1px solid rgba(255,255,255,.08);"
                "background:rgba(0,0,0,.15);}"
                ".links a{color:#d8f3ff;text-decoration:none;background:#243041;padding:8px 12px;border-radius:10px;"
                "font-size:13px;}"
                ".links a:hover{background:#304157;}"
                ".panel{padding:16px 18px;}"
                ".panel-title{font-size:16px;font-weight:700;margin-bottom:12px;color:#f2f6fb;}"
                ".stats{display:grid;grid-template-columns:1fr 1fr;gap:10px;}"
                ".stat{background:rgba(255,255,255,.04);border:1px solid rgba(255,255,255,.06);border-radius:14px;padding:12px;}"
                ".stat-label{font-size:12px;opacity:.7;letter-spacing:.3px;}"
                ".stat-value{font-size:22px;font-weight:700;margin-top:6px;}"
                ".meta{margin-top:12px;display:grid;gap:8px;}"
                ".meta-row{display:flex;justify-content:space-between;gap:12px;font-size:13px;padding:8px 0;border-bottom:1px solid rgba(255,255,255,.06);}"
                ".meta-row:last-child{border-bottom:none;}"
                ".meta-key{opacity:.72;}"
                ".meta-val{font-weight:600;color:#f0f5fb;text-align:right;}"
                ".ok-owner{color:#70f0a0;}"
                ".ok-unknown{color:#ff7d7d;}"
                "@media (max-width: 980px){.grid{grid-template-columns:1fr;}}"
                "</style></head>"
                "<body><div class='wrap'>"
                "<div class='hero'><div>"
                "<div class='title'>Racecar Live Camera</div>"
                "<div class='sub'>Annotated stream with identity labels, runtime HUD and remote detector status. This page uses low-latency latest-frame refresh instead of buffered MJPEG.</div>"
                "</div><div class='badge'>LIVE</div></div>"
                "<div class='grid'>"
                "<div class='card'>"
                "<img id='live-frame' src='/latest.jpg?ts=0' alt='racecar stream' />"
                "<div class='links'>"
                "<a href='/stream.mjpg' target='_blank'>Open Raw Stream</a>"
                "<a href='/latest.jpg' target='_blank'>Open Latest Frame</a>"
                "<a href='/status.json' target='_blank'>Open Status JSON</a>"
                "</div></div>"
                "<div class='card panel'>"
                "<div class='panel-title'>Live Status</div>"
                "<div class='stats'>"
                "<div class='stat'><div class='stat-label'>Mode</div><div class='stat-value' id='mode'>--</div></div>"
                "<div class='stat'><div class='stat-label'>FPS</div><div class='stat-value' id='fps'>--</div></div>"
                "<div class='stat'><div class='stat-label'>Tracks</div><div class='stat-value' id='tracks'>--</div></div>"
                "<div class='stat'><div class='stat-label'>Follow</div><div class='stat-value' id='follow'>--</div></div>"
                "</div>"
                "<div class='meta'>"
                "<div class='meta-row'><div class='meta-key'>Owner Count</div><div class='meta-val ok-owner' id='owner_count'>--</div></div>"
                "<div class='meta-row'><div class='meta-key'>Unknown Count</div><div class='meta-val ok-unknown' id='unknown_count'>--</div></div>"
                "<div class='meta-row'><div class='meta-key'>Loop FPS</div><div class='meta-val' id='loop_fps'>--</div></div>"
                "<div class='meta-row'><div class='meta-key'>Route</div><div class='meta-val' id='remote_route'>--</div></div>"
                "<div class='meta-row'><div class='meta-key'>Infer</div><div class='meta-val' id='infer_ms'>--</div></div>"
                "<div class='meta-row'><div class='meta-key'>RPC</div><div class='meta-val' id='rpc_ms'>--</div></div>"
                "<div class='meta-row'><div class='meta-key'>Server</div><div class='meta-val' id='remote_server_ms'>--</div></div>"
                "<div class='meta-row'><div class='meta-key'>Last Frame</div><div class='meta-val' id='frame_id'>--</div></div>"
                "</div>"
                "</div></div>"
                "<script>"
                "let frameInflight=false;"
                "function pollFrame(){"
                "const img=document.getElementById('live-frame');"
                "if(!img || frameInflight){return;}"
                "frameInflight=true;"
                "img.onload=()=>{frameInflight=false;};"
                "img.onerror=()=>{frameInflight=false;};"
                "img.src='/latest.jpg?ts='+Date.now();"
                "}"
                "async function pollStatus(){"
                "try{const r=await fetch('/status.json',{cache:'no-store'});"
                "if(!r.ok){throw new Error('status');}"
                "const s=await r.json();"
                "const set=(id,v)=>{const el=document.getElementById(id); if(el){el.textContent=v;}};"
                "set('mode',(s.detector_mode||'--').toUpperCase());"
                "set('fps', s.stream_fps==null ? '--' : Number(s.stream_fps).toFixed(1));"
                "set('tracks', s.tracks_total==null ? '--' : s.tracks_total);"
                "set('follow', s.follow_target || 'none');"
                "set('owner_count', s.owner_count==null ? '--' : s.owner_count);"
                "set('unknown_count', s.unknown_count==null ? '--' : s.unknown_count);"
                "set('loop_fps', s.loop_fps==null ? '--' : Number(s.loop_fps).toFixed(1));"
                "set('remote_route', s.remote_route || '--');"
                "set('infer_ms', s.infer_ms==null ? '--' : Number(s.infer_ms).toFixed(1) + ' ms');"
                "set('rpc_ms', s.rpc_ms==null ? '--' : Number(s.rpc_ms).toFixed(1) + ' ms');"
                "set('remote_server_ms', s.remote_server_ms==null ? '--' : Number(s.remote_server_ms).toFixed(1) + ' ms');"
                "set('frame_id', s.frame_id==null ? '--' : s.frame_id);"
                "}catch(e){}}"
                "pollFrame(); setInterval(pollFrame, 60);"
                "pollStatus(); setInterval(pollStatus, 1000);"
                "</script>"
                "</div></body></html>"
            ).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return

        if self.path == "/latest.jpg":
            _, jpeg, _, _ = self.server.frame_store.latest()
            if not jpeg:
                self.send_error(503, "No frame available yet")
                return
            self.send_response(200)
            self.send_header("Content-Type", "image/jpeg")
            self.send_header("Content-Length", str(len(jpeg)))
            self.send_header("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0")
            self.end_headers()
            self.wfile.write(jpeg)
            return

        if self.path == "/status.json":
            frame_id, _, closed, meta = self.server.frame_store.latest()
            body = json.dumps(
                {
                    "ok": bool(frame_id >= 0 and not closed),
                    "frame_id": int(frame_id),
                    **meta,
                },
                ensure_ascii=False,
            ).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return

        if self.path not in ("/stream", "/stream.mjpg"):
            self.send_error(404)
            return

        self.send_response(200)
        self.send_header("Age", "0")
        self.send_header("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0")
        self.send_header("Pragma", "no-cache")
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.end_headers()

        last_frame_id = -1
        try:
            while True:
                frame_id, jpeg, closed, _ = self.server.frame_store.wait_next(last_frame_id, timeout=1.0)
                if closed:
                    break
                if not jpeg or frame_id == last_frame_id:
                    continue
                last_frame_id = frame_id
                self.wfile.write(b"--frame\r\n")
                self.wfile.write(b"Content-Type: image/jpeg\r\n")
                self.wfile.write(f"Content-Length: {len(jpeg)}\r\n\r\n".encode("ascii"))
                self.wfile.write(jpeg)
                self.wfile.write(b"\r\n")
        except (BrokenPipeError, ConnectionResetError):
            return

    def log_message(self, fmt, *args):
        return


def start_mjpeg_server(port, frame_store):
    httpd = ThreadedHTTPServer(("0.0.0.0", int(port)), MjpegHandler)
    httpd.frame_store = frame_store
    thread = threading.Thread(target=httpd.serve_forever, daemon=True)
    thread.start()
    return httpd


def derive_remote_health_url(detect_url):
    parsed = urllib.parse.urlparse(detect_url)
    path = parsed.path or ""
    if path.endswith("/detect"):
        path = path[: -len("/detect")] + "/health"
    elif path.endswith("/"):
        path = path + "health"
    elif not path:
        path = "/health"
    else:
        path = path + "/health"
    return urllib.parse.urlunparse(parsed._replace(path=path, params="", query="", fragment=""))


def _remote_build_path(path, base_query, extra_query=None):
    pairs = []
    if base_query:
        pairs.extend(urllib.parse.parse_qsl(base_query, keep_blank_values=True))
    if extra_query:
        pairs.extend(list(extra_query.items()))
    query = urllib.parse.urlencode(pairs)
    return f"{path}?{query}" if query else path


class RemoteDetectorClient:
    def __init__(self, detect_url, timeout):
        parsed = urllib.parse.urlparse(detect_url)
        scheme = (parsed.scheme or "http").lower()
        if scheme not in ("http", "https"):
            raise ValueError(f"Unsupported remote detector scheme: {scheme}")
        self.scheme = scheme
        self.host = parsed.hostname or "127.0.0.1"
        self.port = parsed.port or (443 if scheme == "https" else 80)
        self.detect_path = parsed.path or "/detect"
        self.base_query = parsed.query or ""
        self.timeout = float(timeout)
        self._conn = None

    def _connect(self):
        if self._conn is not None:
            return self._conn
        if self.scheme == "https":
            self._conn = http.client.HTTPSConnection(self.host, self.port, timeout=self.timeout)
        else:
            self._conn = http.client.HTTPConnection(self.host, self.port, timeout=self.timeout)
        return self._conn

    def close(self):
        conn = self._conn
        self._conn = None
        if conn is not None:
            try:
                conn.close()
            except Exception:
                pass

    def _request_bytes(self, method, path, body=None, headers=None):
        payload = body if body is not None else None
        hdrs = {"Connection": "keep-alive"}
        if headers:
            hdrs.update(headers)
        if payload is not None and "Content-Length" not in hdrs:
            hdrs["Content-Length"] = str(len(payload))

        last_exc = None
        for attempt in range(2):
            conn = self._connect()
            try:
                conn.request(method, path, body=payload, headers=hdrs)
                resp = conn.getresponse()
                raw = resp.read()
                status = int(resp.status)
                if status >= 400:
                    raise RuntimeError(f"remote detector HTTP {status}: {raw[:200]!r}")
                return raw
            except Exception as e:
                last_exc = e
                self.close()
                if attempt == 1:
                    raise
        raise RuntimeError(f"remote detector request failed: {last_exc}")

    def healthcheck(self):
        health_url = derive_remote_health_url(
            urllib.parse.urlunparse(
                (
                    self.scheme,
                    f"{self.host}:{self.port}",
                    self.detect_path,
                    "",
                    self.base_query,
                    "",
                )
            )
        )
        parsed = urllib.parse.urlparse(health_url)
        raw = self._request_bytes("GET", _remote_build_path(parsed.path or "/health", parsed.query))
        return json.loads(raw.decode("utf-8"))

    def detect(self, jpeg_bytes, conf, imgsz):
        path = _remote_build_path(
            self.detect_path,
            self.base_query,
            {"conf": float(conf), "imgsz": int(imgsz)},
        )
        raw = self._request_bytes(
            "POST",
            path,
            body=jpeg_bytes,
            headers={"Content-Type": "image/jpeg"},
        )
        return json.loads(raw.decode("utf-8"))


class AsyncRemoteDetector:
    def __init__(
        self,
        remote_client,
        discovery_conf,
        roi_conf,
        imgsz,
        infer_interval,
        discovery_interval,
        discovery_jpeg_quality,
        discovery_scale,
        roi_max_tracks,
        roi_expand,
        roi_jpeg_quality,
        roi_scale,
        track_low_conf,
        target_cls,
        iou_thres,
    ):
        self.remote_client = remote_client
        self.discovery_conf = float(discovery_conf)
        self.roi_conf = float(roi_conf)
        self.imgsz = int(imgsz)
        self.infer_interval = max(1, int(infer_interval))
        self.discovery_interval = max(1, int(discovery_interval))
        self.discovery_jpeg_quality = int(discovery_jpeg_quality)
        self.discovery_scale = float(discovery_scale)
        self.roi_max_tracks = max(0, int(roi_max_tracks))
        self.roi_expand = float(roi_expand)
        self.roi_jpeg_quality = int(roi_jpeg_quality)
        self.roi_scale = float(roi_scale)
        self.track_low_conf = float(track_low_conf)
        self.target_cls = int(target_cls)
        self.iou_thres = float(iou_thres)
        self._cv = threading.Condition()
        self._pending = None
        self._latest = None
        self._job_id = 0
        self._closed = False
        self._thread = threading.Thread(target=self._worker, daemon=True, name="async_remote_detector")
        self._thread.start()

    def submit(self, frame_id, frame, tracks_snapshot=None):
        with self._cv:
            self._job_id += 1
            self._pending = {
                "job_id": int(self._job_id),
                "frame_id": int(frame_id),
                "frame": frame.copy(),
                "tracks": list(tracks_snapshot or []),
            }
            self._cv.notify_all()
            return int(self._job_id)

    def latest(self):
        with self._cv:
            if self._latest is None:
                return None
            return dict(self._latest)

    def close(self):
        with self._cv:
            self._closed = True
            self._cv.notify_all()
        self._thread.join(timeout=1.0)

    def _worker(self):
        while True:
            with self._cv:
                while not self._closed and self._pending is None:
                    self._cv.wait(timeout=0.5)
                if self._closed:
                    return
                job = self._pending
                self._pending = None

            result = {
                "job_id": int(job["job_id"]),
                "frame_id": int(job["frame_id"]),
                "completed_ts": time.time(),
                "ok": False,
                "dets": [],
                "encode_ms": 0.0,
                "rpc_ms": 0.0,
                "server_ms": 0.0,
                "route": "idle",
                "error": "",
            }
            try:
                dets, encode_ms, rpc_ms, server_ms, route = remote_detect_hybrid(
                    job["frame"],
                    self.remote_client,
                    self.discovery_conf,
                    self.roi_conf,
                    self.imgsz,
                    job["frame_id"],
                    self.infer_interval,
                    self.discovery_interval,
                    self.discovery_jpeg_quality,
                    self.discovery_scale,
                    job.get("tracks", []),
                    self.roi_max_tracks,
                    self.roi_expand,
                    self.roi_jpeg_quality,
                    self.roi_scale,
                    self.track_low_conf,
                    self.target_cls,
                    self.iou_thres,
                )
                result.update(
                    {
                        "completed_ts": time.time(),
                        "ok": True,
                        "dets": dets,
                        "encode_ms": float(encode_ms),
                        "rpc_ms": float(rpc_ms),
                        "server_ms": float(server_ms),
                        "route": str(route),
                    }
                )
            except Exception as e:
                result.update(
                    {
                        "completed_ts": time.time(),
                        "ok": False,
                        "error": str(e),
                    }
                )

            with self._cv:
                self._latest = result
                self._cv.notify_all()


def should_run_remote_discovery(frame_id, infer_interval, discovery_interval):
    infer_interval = max(1, int(infer_interval))
    discovery_interval = max(1, int(discovery_interval))
    if int(frame_id) <= 0:
        return True
    infer_cycle = int(frame_id) // infer_interval
    return (infer_cycle % discovery_interval) == 0


def expand_bbox_xyxy(bbox, frame_shape, expand_ratio=0.2, min_pad=12):
    h, w = frame_shape[:2]
    x1, y1, x2, y2 = [float(v) for v in bbox]
    bw = max(1.0, x2 - x1)
    bh = max(1.0, y2 - y1)
    pad_x = max(float(min_pad), bw * float(expand_ratio))
    pad_y = max(float(min_pad), bh * float(expand_ratio))
    rx1 = max(0, int(round(x1 - pad_x)))
    ry1 = max(0, int(round(y1 - pad_y)))
    rx2 = min(w - 1, int(round(x2 + pad_x)))
    ry2 = min(h - 1, int(round(y2 + pad_y)))
    if rx2 <= rx1:
        rx2 = min(w - 1, rx1 + 1)
    if ry2 <= ry1:
        ry2 = min(h - 1, ry1 + 1)
    return [rx1, ry1, rx2, ry2]


def snapshot_tracks_for_remote_roi(tracks):
    out = []
    for tr in tracks:
        bbox = [int(v) for v in getattr(tr, "bbox", [0, 0, 0, 0])]
        out.append(
            {
                "id": int(getattr(tr, "id", -1)),
                "bbox": bbox,
                "conf": float(getattr(tr, "conf", 0.0)),
                "lost": int(getattr(tr, "lost", 0)),
            }
        )
    return out


def bbox_touches_lr_edge(bbox, frame_shape, edge_margin_ratio=0.02):
    if frame_shape is None:
        return False
    h, w = frame_shape[:2]
    if w <= 0:
        return False
    x1, y1, x2, y2 = [int(v) for v in bbox]
    edge_margin = max(2, int(float(edge_margin_ratio) * float(w)))
    return x1 <= edge_margin or x2 >= (w - edge_margin)


def select_remote_roi_tracks(tracks_snapshot, max_tracks):
    limit = max(0, int(max_tracks))
    if limit <= 0:
        return []
    candidates = []
    for item in tracks_snapshot:
        bbox = [int(v) for v in item.get("bbox", [0, 0, 0, 0])]
        x1, y1, x2, y2 = bbox
        bw = max(0, x2 - x1)
        bh = max(0, y2 - y1)
        area = bw * bh
        if area <= 0:
            continue
        lost = int(item.get("lost", 0))
        if lost > 1:
            continue
        candidates.append(
            {
                "id": int(item.get("id", -1)),
                "bbox": bbox,
                "conf": float(item.get("conf", 0.0)),
                "lost": lost,
                "area": area,
            }
        )
    candidates.sort(
        key=lambda item: (
            0 if int(item["lost"]) <= 0 else 1,
            -float(item["conf"]),
            -int(item["area"]),
        )
    )
    return candidates[:limit]


def remap_detections_from_roi(detections, roi_bbox):
    rx1, ry1, _, _ = roi_bbox
    out = []
    for det in detections:
        bbox = det.get("bbox", [0, 0, 0, 0])
        out.append(
            {
                **det,
                "bbox": [
                    int(bbox[0] + rx1),
                    int(bbox[1] + ry1),
                    int(bbox[2] + rx1),
                    int(bbox[3] + ry1),
                ],
            }
        )
    return out


def choose_best_roi_detection(detections, source_bbox):
    if not detections:
        return None
    best = None
    best_key = None
    source_bbox = [int(v) for v in source_bbox]
    source_area = max(1.0, float(max(0, source_bbox[2] - source_bbox[0]) * max(0, source_bbox[3] - source_bbox[1])))
    for det in detections:
        bbox = [int(v) for v in det.get("bbox", [0, 0, 0, 0])]
        det_area = max(1.0, float(max(0, bbox[2] - bbox[0]) * max(0, bbox[3] - bbox[1])))
        overlap = iou_xyxy(bbox, source_bbox)
        area_ratio = min(det_area, source_area) / max(det_area, source_area)
        key = (float(overlap), float(area_ratio), float(det.get("conf", 0.0)))
        if best is None or key > best_key:
            best = det
            best_key = key
    return best


def dedupe_detection_dicts(detections, iou_thres=0.45):
    if len(detections) <= 1:
        return list(detections)
    valid_detections = []
    boxes = []
    scores = []
    for det in detections:
        x1, y1, x2, y2 = [float(v) for v in det.get("bbox", [0, 0, 0, 0])]
        w = max(0.0, x2 - x1)
        h = max(0.0, y2 - y1)
        if w <= 0 or h <= 0:
            continue
        valid_detections.append(det)
        boxes.append([x1, y1, w, h])
        scores.append(float(det.get("conf", 0.0)))
    if not boxes:
        return []
    idxs = cv2.dnn.NMSBoxes(boxes, scores, 0.0, float(iou_thres))
    if len(idxs) == 0:
        return []
    keep = np.array(idxs).reshape(-1).tolist()
    return [valid_detections[i] for i in keep if 0 <= i < len(valid_detections)]


def make_cached_track_detection(track_item, track_low_conf, target_cls):
    return {
        "bbox": [int(v) for v in track_item.get("bbox", [0, 0, 0, 0])],
        "conf": float(max(track_low_conf, min(float(track_item.get("conf", 0.0)), max(track_low_conf, 0.15)))),
        "cls": int(target_cls),
        "source": "track-cache",
        "track_id": int(track_item.get("id", -1)),
    }


def should_emit_cached_track(track_item, min_conf_required, max_lost=0):
    if int(track_item.get("lost", 0)) > int(max_lost):
        return False
    bbox = [int(v) for v in track_item.get("bbox", [0, 0, 0, 0])]
    x1, y1, x2, y2 = bbox
    if x2 <= x1 or y2 <= y1:
        return False
    conf = float(track_item.get("conf", 0.0))
    if conf < float(min_conf_required):
        return False
    return True


def remote_detect_hybrid(
    frame,
    remote_client,
    discovery_conf,
    roi_conf,
    imgsz,
    frame_id,
    infer_interval,
    discovery_interval,
    discovery_jpeg_quality,
    discovery_scale,
    roi_tracks_snapshot,
    roi_max_tracks,
    roi_expand,
    roi_jpeg_quality,
    roi_scale,
    track_low_conf,
    target_cls,
    iou_thres,
):
    total_encode_ms = 0.0
    total_rpc_ms = 0.0
    total_server_ms = 0.0
    route = "discovery"
    merged = []
    roi_tracks = select_remote_roi_tracks(roi_tracks_snapshot, max_tracks=roi_max_tracks)
    run_discovery = should_run_remote_discovery(frame_id, infer_interval, discovery_interval)

    if run_discovery or not roi_tracks:
        dets, encode_ms, rpc_ms, server_ms = remote_detect(
            frame,
            remote_client,
            discovery_conf,
            imgsz,
            discovery_jpeg_quality,
            discovery_scale,
        )
        total_encode_ms += float(encode_ms)
        total_rpc_ms += float(rpc_ms)
        total_server_ms += float(server_ms)
        dets = [{**det, "source": "discovery"} for det in dets]
        merged.extend(dets)
        route = "discovery"
    else:
        route = "roi"
        selected_ids = set()
        for track_item in roi_tracks:
            selected_ids.add(int(track_item.get("id", -1)))
            if bbox_touches_lr_edge(track_item.get("bbox", [0, 0, 0, 0]), frame.shape):
                continue
            roi_bbox = expand_bbox_xyxy(track_item.get("bbox", [0, 0, 0, 0]), frame.shape, roi_expand)
            rx1, ry1, rx2, ry2 = roi_bbox
            roi_frame = frame[ry1:ry2, rx1:rx2]
            if roi_frame.size == 0 or roi_frame.shape[0] < 8 or roi_frame.shape[1] < 8:
                continue
            dets, encode_ms, rpc_ms, server_ms = remote_detect(
                roi_frame,
                remote_client,
                roi_conf,
                imgsz,
                roi_jpeg_quality,
                roi_scale,
            )
            total_encode_ms += float(encode_ms)
            total_rpc_ms += float(rpc_ms)
            total_server_ms += float(server_ms)
            remapped = remap_detections_from_roi(dets, roi_bbox)
            best = choose_best_roi_detection(remapped, track_item.get("bbox", [0, 0, 0, 0]))
            if best is None:
                continue
            best = {
                **best,
                "source": "roi",
                "track_id": int(track_item.get("id", -1)),
            }
            merged.append(best)

    merged = dedupe_detection_dicts(merged, iou_thres=iou_thres)
    return merged, total_encode_ms, total_rpc_ms, total_server_ms, route


class FFmpegVideoStreamer:
    def __init__(self, output_url, width, height, fps=25.0, crf=20, preset="ultrafast"):
        self.output_url = str(output_url)
        self.width = int(width)
        self.height = int(height)
        self.fps = max(1.0, float(fps))
        self.crf = int(crf)
        self.preset = str(preset)
        self.proc = None
        self._cv = threading.Condition()
        self._latest_frame = None
        self._closed = False
        self._start_process()
        self._thread = threading.Thread(target=self._worker, daemon=True, name="ffmpeg_video_streamer")
        self._thread.start()

    def _start_process(self):
        cmd = [
            "ffmpeg",
            "-loglevel",
            "error",
            "-nostdin",
            "-fflags",
            "nobuffer",
            "-f",
            "rawvideo",
            "-pix_fmt",
            "bgr24",
            "-s",
            f"{self.width}x{self.height}",
            "-r",
            f"{self.fps:.3f}",
            "-i",
            "-",
            "-an",
            "-c:v",
            "libx264",
            "-preset",
            self.preset,
            "-tune",
            "zerolatency",
            "-flags",
            "low_delay",
            "-pix_fmt",
            "yuv420p",
            "-g",
            str(max(1, int(round(self.fps)))),
            "-keyint_min",
            str(max(1, int(round(self.fps)))),
            "-crf",
            str(self.crf),
            "-flush_packets",
            "1",
            "-muxdelay",
            "0",
            "-muxpreload",
            "0",
            "-f",
            "mpegts",
            self.output_url,
        ]
        self.proc = subprocess.Popen(
            cmd,
            stdin=subprocess.PIPE,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )

    def write(self, frame):
        if frame is None or frame.size == 0:
            return False
        with self._cv:
            if self._closed:
                return False
            self._latest_frame = frame.copy()
            self._cv.notify_all()
            return True

    def _close_process(self):
        if self.proc is None:
            return
        try:
            if self.proc.stdin is not None:
                try:
                    self.proc.stdin.close()
                except Exception:
                    pass
            self.proc.terminate()
            self.proc.wait(timeout=1.5)
        except Exception:
            try:
                self.proc.kill()
            except Exception:
                pass
        finally:
            self.proc = None

    def _worker(self):
        while True:
            with self._cv:
                while not self._closed and self._latest_frame is None:
                    self._cv.wait(timeout=0.5)
                if self._closed:
                    return
                frame = self._latest_frame
                self._latest_frame = None

            if frame.shape[1] != self.width or frame.shape[0] != self.height:
                frame = cv2.resize(frame, (self.width, self.height), interpolation=cv2.INTER_LINEAR)

            if self.proc is None:
                try:
                    self._start_process()
                except Exception:
                    time.sleep(0.2)
                    continue

            try:
                self.proc.stdin.write(frame.tobytes())
            except Exception:
                self._close_process()
                time.sleep(0.1)

    def close(self):
        with self._cv:
            self._closed = True
            self._latest_frame = None
            self._cv.notify_all()
        try:
            self._thread.join(timeout=1.0)
        except Exception:
            pass
        self._close_process()


def remote_healthcheck(remote_client):
    return remote_client.healthcheck()


def remote_detect(frame, remote_client, conf, imgsz, jpeg_quality, scale=1.0):
    scale = min(1.0, max(0.25, float(scale)))
    send_frame = frame
    scale_x = 1.0
    scale_y = 1.0
    if scale < 0.999:
        src_h, src_w = frame.shape[:2]
        dst_w = max(1, int(round(src_w * scale)))
        dst_h = max(1, int(round(src_h * scale)))
        if dst_w != src_w or dst_h != src_h:
            send_frame = cv2.resize(frame, (dst_w, dst_h), interpolation=cv2.INTER_AREA)
            scale_x = float(src_w) / float(dst_w)
            scale_y = float(src_h) / float(dst_h)

    t_encode = time.time()
    ok, enc = cv2.imencode(".jpg", send_frame, [int(cv2.IMWRITE_JPEG_QUALITY), int(jpeg_quality)])
    if not ok:
        raise RuntimeError("Failed to encode frame as JPEG for remote detector")
    encode_ms = (time.time() - t_encode) * 1000.0

    t_rpc = time.time()
    payload = remote_client.detect(enc.tobytes(), conf, imgsz)
    rpc_ms = (time.time() - t_rpc) * 1000.0

    dets = []
    for item in payload.get("detections", []):
        bbox = item.get("bbox_xyxy") or item.get("bbox")
        if not isinstance(bbox, (list, tuple)) or len(bbox) != 4:
            continue
        if scale_x != 1.0 or scale_y != 1.0:
            bbox = [
                float(bbox[0]) * scale_x,
                float(bbox[1]) * scale_y,
                float(bbox[2]) * scale_x,
                float(bbox[3]) * scale_y,
            ]
        dets.append(
            {
                "bbox": [int(round(float(v))) for v in bbox],
                "conf": float(item.get("conf", 0.0)),
                "cls": int(item.get("cls", 0)),
            }
        )
    return dets, encode_ms, rpc_ms, float(payload.get("server_ms", 0.0))


def run_infer(backend, sess, inp):
    if backend == "ais_bench":
        return sess.infer([inp])

    trials = []
    if hasattr(sess, "infer"):
        trials += [lambda: sess.infer([inp]), lambda: sess.infer(inp)]
    if hasattr(sess, "run"):
        trials += [lambda: sess.run([inp]), lambda: sess.run(None, [inp]), lambda: sess.run([inp], None)]

    last = None
    for f in trials:
        try:
            return f()
        except Exception as e:
            last = e
    raise RuntimeError(f"aclruntime infer failed: {last}")


def normalize_output(outputs):
    if isinstance(outputs, tuple):
        outputs = list(outputs)
    elif not isinstance(outputs, list):
        outputs = [outputs]

    mats = []
    for o in outputs:
        a = np.array(o)
        if a.size == 0:
            continue
        a = np.squeeze(a)
        if a.ndim == 2 and a.shape[1] >= 6:
            mats.append(a)
        elif a.ndim in (3, 4) and a.shape[-1] >= 6:
            mats.append(a.reshape(-1, a.shape[-1]))

    if not mats:
        raise RuntimeError("Cannot parse model output tensor shape.")
    return np.concatenate(mats, axis=0).astype(np.float32, copy=False)


def postprocess(
    pred,
    ratio,
    dwdh,
    orig_shape,
    conf_thres=0.25,
    iou_thres=0.45,
    target_cls=0,
    max_candidates=300,
    low_conf_thres=None,
):
    if low_conf_thres is None:
        low_conf_thres = conf_thres
    low_conf_thres = min(float(conf_thres), float(low_conf_thres))
    h0, w0 = orig_shape[:2]
    dw, dh = dwdh

    if pred.shape[1] >= 7:
        boxes = pred[:, :4]
        obj = pred[:, 4]
        cls_prob = pred[:, 5:]
        cls_id = np.argmax(cls_prob, axis=1)
        cls_conf = cls_prob[np.arange(cls_prob.shape[0]), cls_id]
        scores = obj * cls_conf

        keep = scores >= low_conf_thres
        boxes, scores, cls_id = boxes[keep], scores[keep], cls_id[keep]
        if len(boxes) == 0:
            return []

        if target_cls >= 0:
            keep = cls_id == int(target_cls)
            boxes, scores, cls_id = boxes[keep], scores[keep], cls_id[keep]
            if len(boxes) == 0:
                return []

        xyxy = np.zeros_like(boxes)
        xyxy[:, 0] = boxes[:, 0] - boxes[:, 2] / 2
        xyxy[:, 1] = boxes[:, 1] - boxes[:, 3] / 2
        xyxy[:, 2] = boxes[:, 0] + boxes[:, 2] / 2
        xyxy[:, 3] = boxes[:, 1] + boxes[:, 3] / 2
    else:
        xyxy = pred[:, :4]
        scores = pred[:, 4]
        cls_id = pred[:, 5].astype(np.int32)
        keep = scores >= low_conf_thres
        xyxy, scores, cls_id = xyxy[keep], scores[keep], cls_id[keep]
        if len(xyxy) == 0:
            return []

        if target_cls >= 0:
            keep = cls_id == int(target_cls)
            xyxy, scores, cls_id = xyxy[keep], scores[keep], cls_id[keep]
            if len(xyxy) == 0:
                return []

    if max_candidates > 0 and len(scores) > max_candidates:
        top_idx = np.argpartition(scores, -max_candidates)[-max_candidates:]
        top_idx = top_idx[np.argsort(scores[top_idx])[::-1]]
        xyxy = xyxy[top_idx]
        scores = scores[top_idx]
        cls_id = cls_id[top_idx]

    xyxy[:, [0, 2]] = (xyxy[:, [0, 2]] - dw) / ratio
    xyxy[:, [1, 3]] = (xyxy[:, [1, 3]] - dh) / ratio
    xyxy[:, 0] = np.clip(xyxy[:, 0], 0, w0 - 1)
    xyxy[:, 1] = np.clip(xyxy[:, 1], 0, h0 - 1)
    xyxy[:, 2] = np.clip(xyxy[:, 2], 0, w0 - 1)
    xyxy[:, 3] = np.clip(xyxy[:, 3], 0, h0 - 1)

    if len(xyxy) == 1:
        idxs = np.array([0], dtype=np.int32)
    else:
        wh = np.maximum(0.0, xyxy[:, 2:4] - xyxy[:, 0:2])
        nms_boxes = np.concatenate([xyxy[:, 0:2], wh], axis=1).tolist()

        idxs = cv2.dnn.NMSBoxes(nms_boxes, scores.tolist(), conf_thres, iou_thres)
        if len(idxs) == 0:
            return []

    idxs = np.array(idxs).reshape(-1)
    out = []
    for i in idxs:
        cls = int(cls_id[i])
        x1, y1, x2, y2 = xyxy[i].tolist()
        out.append(
            {
                "cls": cls,
                "conf": float(scores[i]),
                "bbox": [int(x1), int(y1), int(x2), int(y2)],
            }
        )
    return out


def filter_person_detections(detections, frame_shape, args):
    if not detections:
        return []

    h0, w0 = frame_shape[:2]
    edge_margin = max(2, int(float(args.person_edge_margin) * float(w0)))
    min_w = max(1, int(args.min_person_width))
    min_h = max(1, int(args.min_person_height))
    min_area = max(1, int(args.min_person_area))
    edge_min_conf = float(args.person_edge_min_conf)

    kept = []
    for det in detections:
        x1, y1, x2, y2 = [int(v) for v in det.get("bbox", [0, 0, 0, 0])]
        bw = max(0, x2 - x1)
        bh = max(0, y2 - y1)
        area = bw * bh
        conf = float(det.get("conf", 0.0))
        if bw < min_w or bh < min_h or area < min_area:
            continue
        touches_lr_edge = x1 <= edge_margin or x2 >= (w0 - edge_margin)
        if touches_lr_edge and conf < edge_min_conf:
            continue
        kept.append(det)
    return kept


def iou_xyxy(a, b):
    ax1, ay1, ax2, ay2 = a
    bx1, by1, bx2, by2 = b
    inter_x1 = max(ax1, bx1)
    inter_y1 = max(ay1, by1)
    inter_x2 = min(ax2, bx2)
    inter_y2 = min(ay2, by2)
    iw = max(0.0, inter_x2 - inter_x1)
    ih = max(0.0, inter_y2 - inter_y1)
    inter = iw * ih
    if inter <= 0:
        return 0.0
    a_area = max(0.0, (ax2 - ax1) * (ay2 - ay1))
    b_area = max(0.0, (bx2 - bx1) * (by2 - by1))
    union = a_area + b_area - inter
    if union <= 0:
        return 0.0
    return inter / union


class Track:
    def __init__(self, track_id, bbox, conf, frame_id):
        self.id = track_id
        self.bbox = bbox
        self.conf = conf
        self.created_frame = frame_id
        self.last_seen = frame_id
        self.last_detection_frame = frame_id
        self.matched_this_frame = True
        self.lost = 0
        self.name_history = deque(maxlen=8)
        self.name = "unknown"
        self.score = 0.0
        self.unknown_streak = 0
        self.known_miss_count = 0
        self.last_face_frame = -999999
        self.last_face_area = 0.0
        self.last_face_raw_name = "none"
        self.last_face_raw_score = 0.0
        self.last_face_top1_score = 0.0
        self.last_face_topk_score = 0.0
        self.last_face_match_count = 0
        self.face_evidence_state = "none"
        self.identity_transition_reason = "init"
        self.alarmed_once = False
        self.provisional_name = ""
        self.provisional_score = 0.0
        self.provisional_expires_frame = -1
        self.provisional_source_track_id = None

    def center(self):
        x1, y1, x2, y2 = self.bbox
        return (0.5 * (x1 + x2), 0.5 * (y1 + y2))

    def size(self):
        x1, y1, x2, y2 = self.bbox
        return (max(0, x2 - x1), max(0, y2 - y1))

    def area(self):
        w, h = self.size()
        return float(w * h)

    def clear_provisional_identity(self):
        self.provisional_name = ""
        self.provisional_score = 0.0
        self.provisional_expires_frame = -1
        self.provisional_source_track_id = None

    def set_provisional_identity(self, name, score, expires_frame, source_track_id=None):
        self.provisional_name = str(name)
        self.provisional_score = float(score)
        self.provisional_expires_frame = int(expires_frame)
        self.provisional_source_track_id = None if source_track_id is None else int(source_track_id)

    def update_name(
        self,
        new_name,
        new_score,
        known_hold_misses=2,
        known_hold_score=0.42,
        min_accept_score=0.0,
        known_accept_score=0.68,
        known_confirm_hits=3,
    ):
        prev_name = self.name
        smoothed_prev_score = float(self.score)
        self.score = max(self.score * 0.8, float(new_score))
        self.identity_transition_reason = "unchanged"

        candidate_accept = max(0.0, float(min_accept_score))
        direct_accept = max(candidate_accept, float(known_accept_score))
        if new_name != "unknown" and float(new_score) < candidate_accept:
            new_name = "unknown"

        if new_name != "unknown" and float(new_score) >= direct_accept:
            self.known_miss_count = 0
            self.name_history.append(new_name)
            self.name = str(new_name)
            self.score = max(float(self.score), float(new_score))
            self.unknown_streak = 0
            self.clear_provisional_identity()
            self.identity_transition_reason = "direct_accept"
            return self.name != prev_name

        if new_name != "unknown":
            self.known_miss_count = 0
            self.name_history.append(new_name)
            cnt = Counter(self.name_history)
            if cnt:
                best_name, best_count = cnt.most_common(1)[0]
                if best_name != "unknown" and best_count >= max(2, int(known_confirm_hits)):
                    self.name = str(best_name)
                    self.score = max(float(self.score), float(new_score))
                    self.unknown_streak = 0
                    self.clear_provisional_identity()
                    self.identity_transition_reason = "confirm_hits"
                    return self.name != prev_name
            self.name = "unknown"
            self.clear_provisional_identity()
            self.identity_transition_reason = "known_pending"
            return self.name != prev_name

        if new_name == "unknown":
            if (
                prev_name != "unknown"
                and self.known_miss_count < int(known_hold_misses)
                and smoothed_prev_score >= float(known_hold_score)
            ):
                self.known_miss_count += 1
                self.name_history.append(prev_name)
                self.name = prev_name
                self.unknown_streak = 0
                self.clear_provisional_identity()
                self.identity_transition_reason = "known_hold"
                return self.name != prev_name
            self.known_miss_count = 0
            self.name_history.append("unknown")
            self.name = "unknown"
            self.clear_provisional_identity()
            self.identity_transition_reason = "fallback_unknown"
            return self.name != prev_name
        self.name = "unknown"
        self.clear_provisional_identity()
        self.identity_transition_reason = "forced_unknown"
        return self.name != prev_name


class IOUTracker:
    def __init__(
        self,
        iou_thres=0.35,
        max_lost=20,
        high_conf_thres=0.35,
        low_conf_thres=0.1,
        new_track_iou_gate=0.25,
    ):
        self.iou_thres = iou_thres
        self.max_lost = max_lost
        self.high_conf_thres = float(high_conf_thres)
        self.low_conf_thres = float(low_conf_thres)
        self.new_track_iou_gate = float(new_track_iou_gate)
        self.next_id = 1
        self.tracks = {}

    def _associate(self, track_ids, detections, frame_id):
        pairs = []
        for ti, tid in enumerate(track_ids):
            tb = self.tracks[tid].bbox
            for di, det in enumerate(detections):
                iou = iou_xyxy(tb, det["bbox"])
                if iou >= self.iou_thres:
                    pairs.append((iou, ti, di))
        pairs.sort(reverse=True, key=lambda x: x[0])

        used_tracks = set()
        used_dets = set()
        for _, ti, di in pairs:
            tid = track_ids[ti]
            if tid in used_tracks or di in used_dets:
                continue
            tr = self.tracks[tid]
            det = detections[di]
            tr.bbox = det["bbox"]
            tr.conf = det["conf"]
            tr.last_seen = frame_id
            tr.last_detection_frame = frame_id
            tr.matched_this_frame = True
            tr.lost = 0
            used_tracks.add(tid)
            used_dets.add(di)

        return used_tracks, used_dets

    def update(self, detections, frame_id, detections_fresh=True):
        for tr in self.tracks.values():
            tr.matched_this_frame = False
        if not detections_fresh:
            return list(self.tracks.values())
        detections = list(detections or [])
        high_dets = [
            det
            for det in detections
            if float(det.get("conf", 0.0)) >= self.high_conf_thres
        ]
        low_dets = [
            det
            for det in detections
            if self.low_conf_thres <= float(det.get("conf", 0.0)) < self.high_conf_thres
        ]

        track_ids = list(self.tracks.keys())
        matched_tracks, matched_high = self._associate(track_ids, high_dets, frame_id)

        unmatched_tracks = [tid for tid in track_ids if tid not in matched_tracks]
        if unmatched_tracks and low_dets:
            recovered_tracks, matched_low = self._associate(unmatched_tracks, low_dets, frame_id)
            matched_tracks.update(recovered_tracks)
        else:
            matched_low = set()

        for tid in track_ids:
            if tid not in matched_tracks:
                self.tracks[tid].lost += 1

        for di, det in enumerate(high_dets):
            if di in matched_high:
                continue
            if str(det.get("source", "")).lower() in {"roi", "track-cache"}:
                continue
            overlaps_existing = False
            for tid, tr in self.tracks.items():
                if iou_xyxy(tr.bbox, det["bbox"]) >= self.new_track_iou_gate:
                    overlaps_existing = True
                    break
            if overlaps_existing:
                continue
            tid = self.next_id
            self.next_id += 1
            self.tracks[tid] = Track(tid, det["bbox"], det["conf"], frame_id)

        dead = [tid for tid, tr in self.tracks.items() if tr.lost > self.max_lost]
        for tid in dead:
            del self.tracks[tid]

        return list(self.tracks.values())


def build_haar_hog():
    cascade = cv2.CascadeClassifier(cv2.data.haarcascades + "haarcascade_frontalface_default.xml")
    hog = cv2.HOGDescriptor(
        _winSize=(64, 64),
        _blockSize=(16, 16),
        _blockStride=(8, 8),
        _cellSize=(8, 8),
        _nbins=9,
    )
    return cascade, hog


def choose_face_backend(det_model, rec_model, det_thres=0.85):
    if (
        det_model
        and rec_model
        and os.path.exists(det_model)
        and os.path.exists(rec_model)
        and os.path.getsize(det_model) > 0
        and os.path.getsize(rec_model) > 0
        and hasattr(cv2, "FaceDetectorYN_create")
        and hasattr(cv2, "FaceRecognizerSF_create")
    ):
        detector = cv2.FaceDetectorYN_create(det_model, "", (320, 320), det_thres, 0.3, 5000)
        recognizer = cv2.FaceRecognizerSF_create(rec_model, "")
        return "sface", detector, recognizer
    cascade, hog = build_haar_hog()
    return "haar", cascade, hog


def has_recent_face_evidence(track, frame_id, max_gap, min_score=0.0, required_raw_name=None):
    last_face_frame = int(getattr(track, "last_face_frame", -999999))
    if last_face_frame < 0:
        return False
    if frame_id is not None and int(frame_id) - last_face_frame > max(0, int(max_gap)):
        return False
    raw_score = float(getattr(track, "last_face_raw_score", 0.0))
    if raw_score < float(min_score):
        return False
    if required_raw_name is not None:
        raw_name = str(getattr(track, "last_face_raw_name", "none"))
        if raw_name != str(required_raw_name):
            return False
    return True


def has_active_provisional_identity(track, frame_id):
    if str(getattr(track, "name", "unknown")) != "unknown":
        return False
    provisional_name = str(getattr(track, "provisional_name", "") or "")
    if not provisional_name:
        return False
    return int(frame_id) <= int(getattr(track, "provisional_expires_frame", -1))


def provisional_display_name(track, frame_id):
    if has_active_provisional_identity(track, frame_id):
        return str(getattr(track, "provisional_name", ""))
    return ""


def normalized_center_distance(a, b, frame_shape):
    ah, aw = frame_shape[:2]
    scale = max(1.0, float(max(aw, ah)))
    ax, ay = a.center()
    bx, by = b.center()
    return float(np.hypot(float(ax) - float(bx), float(ay) - float(by)) / scale)


def area_ratio_between(a, b):
    a_area = max(1.0, float(a.area()))
    b_area = max(1.0, float(b.area()))
    hi = max(a_area, b_area)
    lo = max(1.0, min(a_area, b_area))
    return hi / lo


def apply_identity_inheritance(tracks, frame_id, frame_shape, args):
    fresh_unknown_tracks = []
    for tr in tracks:
        if int(getattr(tr, "lost", 0)) > 0 or str(getattr(tr, "name", "unknown")) != "unknown":
            tr.clear_provisional_identity()
            continue
        if int(frame_id) > int(getattr(tr, "provisional_expires_frame", -1)):
            tr.clear_provisional_identity()
        if int(frame_id) - int(getattr(tr, "created_frame", frame_id)) <= int(args.identity_inherit_new_track_window):
            fresh_unknown_tracks.append(tr)

    if not fresh_unknown_tracks:
        return

    candidates = [
        tr
        for tr in tracks
        if str(getattr(tr, "name", "unknown")) != "unknown"
        and float(getattr(tr, "score", 0.0)) >= float(args.identity_inherit_min_score)
        and 0 < int(getattr(tr, "lost", 0)) <= int(args.identity_inherit_max_lost)
    ]
    if not candidates:
        return

    for target in fresh_unknown_tracks:
        best = None
        best_key = None
        for source in candidates:
            center_dist = normalized_center_distance(source, target, frame_shape)
            if center_dist > float(args.identity_inherit_center_dist):
                continue
            area_ratio = area_ratio_between(source, target)
            if area_ratio > float(args.identity_inherit_area_ratio):
                continue
            key = (float(source.score), -center_dist, -area_ratio)
            if best_key is None or key > best_key:
                best = source
                best_key = key
        if best is None:
            continue
        target.set_provisional_identity(
            best.name,
            float(best.score),
            int(frame_id) + int(args.identity_inherit_hold_frames),
            source_track_id=best.id,
        )


def load_face_db(db_path):
    if not os.path.exists(db_path):
        return [], np.zeros((0, 1024), dtype=np.float32), {"method": "unknown", "aggregate": "unknown", "build_version": "unknown", "quality_scores": np.zeros((0,), dtype=np.float32)}
    data = np.load(db_path, allow_pickle=True)
    names = data["names"].tolist()
    feats = data["feats"].astype(np.float32)
    feats = np.asarray([l2_normalize(f) for f in feats], dtype=np.float32)
    method = str(data["method"][0]) if "method" in data else "unknown"
    aggregate = str(data["aggregate"][0]) if "aggregate" in data else "unknown"
    build_version = str(data["build_version"][0]) if "build_version" in data else "unknown"
    quality_scores = data["quality_scores"].astype(np.float32) if "quality_scores" in data else np.ones((len(names),), dtype=np.float32)
    if quality_scores.shape[0] != len(names):
        quality_scores = np.ones((len(names),), dtype=np.float32)
    meta = {
        "method": method,
        "aggregate": aggregate,
        "build_version": build_version,
        "quality_scores": quality_scores,
    }
    return names, feats, meta


def summarize_identity_scores(db_names, scores, db_quality_scores, topk=3, match_thres=0.36):
    grouped = {}
    for idx, name in enumerate(db_names):
        grouped.setdefault(str(name), []).append(
            (
                float(scores[idx]),
                float(db_quality_scores[idx]) if idx < len(db_quality_scores) else 1.0,
            )
        )

    summary = {}
    for name, items in grouped.items():
        items.sort(reverse=True, key=lambda item: item[0])
        keep = items[: max(1, min(int(topk), len(items)))]
        vals = np.asarray([float(score) for score, _ in keep], dtype=np.float32)
        weights = np.asarray([max(0.2, float(weight)) for _, weight in keep], dtype=np.float32)
        topk_mean = float(np.dot(vals, weights) / max(1e-6, float(weights.sum())))
        top1 = float(items[0][0]) if items else 0.0
        match_count = int(sum(1 for score, _ in items if float(score) >= float(match_thres)))
        summary[name] = {
            "top1": top1,
            "topk_mean": topk_mean,
            "match_count": match_count,
        }
    return summary


def classify_face_evidence(raw_name, top1_score, topk_score, face_detected, match_thres, high_accept):
    if not face_detected:
        return "none"
    if raw_name != "unknown":
        if float(topk_score) >= float(high_accept):
            return "confirmed_owner"
        return "weak_owner"
    if max(float(top1_score), float(topk_score)) >= max(0.1, float(match_thres) * 0.5):
        return "weak_unknown"
    return "confirmed_unknown"


def detect_largest_face_sface(detector, img):
    h, w = img.shape[:2]
    if h < 20 or w < 20:
        return None
    detector.setInputSize((w, h))
    _, faces = detector.detect(img)
    if faces is None or len(faces) == 0:
        return None
    areas = faces[:, 2] * faces[:, 3]
    return faces[int(np.argmax(areas))]


def detect_all_faces_sface(detector, img):
    h, w = img.shape[:2]
    if h < 20 or w < 20:
        return []
    detector.setInputSize((w, h))
    _, faces = detector.detect(img)
    if faces is None or len(faces) == 0:
        return []
    return [faces[i] for i in range(len(faces))]


def detect_all_faces_haar(cascade, img):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    gray = cv2.equalizeHist(gray)
    faces = cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5, minSize=(40, 40))
    return list(faces)


def extract_face_feature_haar(cascade, hog, img):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    gray = cv2.equalizeHist(gray)
    faces = cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5, minSize=(40, 40))
    if len(faces) == 0:
        return None
    x, y, w, h = max(faces, key=lambda r: r[2] * r[3])
    face = gray[y : y + h, x : x + w]
    if face.size == 0:
        return None
    s1 = cv2.resize(face, (32, 32), interpolation=cv2.INTER_AREA).astype(np.float32).flatten() / 255.0
    s2_in = cv2.resize(face, (64, 64), interpolation=cv2.INTER_AREA)
    s2 = hog.compute(s2_in).flatten().astype(np.float32)
    s2 = l2_normalize(s2)
    feat = np.concatenate([s1, s2], axis=0).astype(np.float32)
    return l2_normalize(feat)


def recognize_face_in_person(
    face_mode,
    detector,
    recognizer,
    db_names,
    db_feats,
    db_meta,
    frame,
    person_bbox,
    match_thres,
    high_accept,
):
    if len(db_names) == 0 or len(db_feats) == 0:
        return {
            "name": "unknown",
            "score": 0.0,
            "raw_name": "none",
            "top1_score": 0.0,
            "topk_score": 0.0,
            "match_count_above_threshold": 0,
            "face_detected": False,
            "face_evidence_state": "none",
        }

    x1, y1, x2, y2 = person_bbox
    h, w = frame.shape[:2]
    x1 = max(0, min(w - 1, x1))
    y1 = max(0, min(h - 1, y1))
    x2 = max(0, min(w - 1, x2))
    y2 = max(0, min(h - 1, y2))
    if x2 <= x1 + 20 or y2 <= y1 + 20:
        return {
            "name": "unknown",
            "score": 0.0,
            "raw_name": "none",
            "top1_score": 0.0,
            "topk_score": 0.0,
            "match_count_above_threshold": 0,
            "face_detected": False,
            "face_evidence_state": "none",
        }

    person = frame[y1:y2, x1:x2]
    ph, pw = person.shape[:2]
    if ph < 80 or pw < 60:
        return {
            "name": "unknown",
            "score": 0.0,
            "raw_name": "none",
            "top1_score": 0.0,
            "topk_score": 0.0,
            "match_count_above_threshold": 0,
            "face_detected": False,
            "face_evidence_state": "none",
        }

    person_top = person[: int(ph * 0.80), :]
    db_quality_scores = np.asarray(db_meta.get("quality_scores", np.ones((len(db_names),), dtype=np.float32)), dtype=np.float32)
    if face_mode == "sface":
        scales = [1.0]
        # Small hallway faces are often missed unless we upscale the head crop.
        if pw < 160 or ph < 260:
            scales.extend([1.5, 2.0, 2.5])

        best_result = None
        for scale in scales:
            if scale == 1.0:
                roi = person_top
            else:
                roi = cv2.resize(person_top, None, fx=scale, fy=scale, interpolation=cv2.INTER_CUBIC)
            face = detect_largest_face_sface(detector, roi)
            if face is None:
                continue
            aligned = recognizer.alignCrop(roi, face)
            feat = recognizer.feature(aligned).flatten().astype(np.float32)
            feat = l2_normalize(feat)
            scores = db_feats @ feat
            summary = summarize_identity_scores(db_names, scores, db_quality_scores, topk=3, match_thres=match_thres)
            if not summary:
                continue
            best_name = max(summary.keys(), key=lambda name: (summary[name]["topk_mean"], summary[name]["top1"], summary[name]["match_count"]))
            top1_score = float(summary[best_name]["top1"])
            topk_score = float(summary[best_name]["topk_mean"])
            match_count = int(summary[best_name]["match_count"])
            accepted_name = best_name if top1_score >= float(match_thres) and topk_score >= float(match_thres) else "unknown"
            evidence_state = classify_face_evidence(accepted_name, top1_score, topk_score, True, match_thres, high_accept)
            candidate = {
                "name": accepted_name,
                "score": topk_score,
                "raw_name": accepted_name if accepted_name != "unknown" else "unknown",
                "top1_score": top1_score,
                "topk_score": topk_score,
                "match_count_above_threshold": match_count,
                "face_detected": True,
                "face_evidence_state": evidence_state,
            }
            if best_result is None or (candidate["topk_score"], candidate["top1_score"]) > (
                best_result["topk_score"],
                best_result["top1_score"],
            ):
                best_result = candidate
        if best_result is not None:
            return best_result
        return {
            "name": "unknown",
            "score": 0.0,
            "raw_name": "none",
            "top1_score": 0.0,
            "topk_score": 0.0,
            "match_count_above_threshold": 0,
            "face_detected": False,
            "face_evidence_state": "none",
        }
    else:
        feat = extract_face_feature_haar(detector, recognizer, person_top)
        if feat is None:
            return {
                "name": "unknown",
                "score": 0.0,
                "raw_name": "none",
                "top1_score": 0.0,
                "topk_score": 0.0,
                "match_count_above_threshold": 0,
                "face_detected": False,
                "face_evidence_state": "none",
            }

    scores = db_feats @ feat
    summary = summarize_identity_scores(db_names, scores, db_quality_scores, topk=3, match_thres=match_thres)
    if not summary:
        return {
            "name": "unknown",
            "score": 0.0,
            "raw_name": "none",
            "top1_score": 0.0,
            "topk_score": 0.0,
            "match_count_above_threshold": 0,
            "face_detected": False,
            "face_evidence_state": "none",
        }
    best_name = max(summary.keys(), key=lambda name: (summary[name]["topk_mean"], summary[name]["top1"], summary[name]["match_count"]))
    top1_score = float(summary[best_name]["top1"])
    topk_score = float(summary[best_name]["topk_mean"])
    match_count = int(summary[best_name]["match_count"])
    accepted_name = best_name if top1_score >= float(match_thres) and topk_score >= float(match_thres) else "unknown"
    return {
        "name": accepted_name,
        "score": topk_score,
        "raw_name": accepted_name if accepted_name != "unknown" else "unknown",
        "top1_score": top1_score,
        "topk_score": topk_score,
        "match_count_above_threshold": match_count,
        "face_detected": True,
        "face_evidence_state": classify_face_evidence(accepted_name, top1_score, topk_score, True, match_thres, high_accept),
    }


def recognize_faces_in_frame(face_mode, detector, recognizer, db_names, db_feats, frame, match_thres):
    if len(db_names) == 0 or len(db_feats) == 0:
        return []

    out = []
    if face_mode == "sface":
        faces = detect_all_faces_sface(detector, frame)
        for face in faces:
            x, y, w, h = face[:4].astype(np.int32).tolist()
            if w < 20 or h < 20:
                continue
            aligned = recognizer.alignCrop(frame, face)
            feat = recognizer.feature(aligned).flatten().astype(np.float32)
            feat = l2_normalize(feat)
            scores = db_feats @ feat
            best_idx = int(np.argmax(scores))
            best_score = float(scores[best_idx])
            name = db_names[best_idx] if best_score >= match_thres else "unknown"
            out.append(
                {
                    "bbox": [x, y, x + w, y + h],
                    "name": name,
                    "score": best_score,
                }
            )
        return out

    faces = detect_all_faces_haar(detector, frame)
    for (x, y, w, h) in faces:
        roi = frame[y : y + h, x : x + w]
        feat = extract_face_feature_haar(detector, recognizer, roi)
        if feat is None:
            continue
        scores = db_feats @ feat
        best_idx = int(np.argmax(scores))
        best_score = float(scores[best_idx])
        name = db_names[best_idx] if best_score >= match_thres else "unknown"
        out.append(
            {
                "bbox": [int(x), int(y), int(x + w), int(y + h)],
                "name": name,
                "score": best_score,
            }
        )
    return out


def match_face_to_track(face_rows, person_bbox):
    if not face_rows:
        return None
    x1, y1, x2, y2 = person_bbox
    best = None
    best_score = -1.0
    for row in face_rows:
        fx1, fy1, fx2, fy2 = row["bbox"]
        cx = 0.5 * (fx1 + fx2)
        cy = 0.5 * (fy1 + fy2)
        if x1 <= cx <= x2 and y1 <= cy <= y2:
            face_area = max(0, fx2 - fx1) * max(0, fy2 - fy1)
            rank = (float(row["score"]), float(face_area))
            if rank > (best_score, -1):
                best = row
                best_score = float(row["score"])
    return best


def make_temp_writer(width, height, fps, record_step=1):
    base = datetime.now().strftime("tmp_rec_%Y%m%d_%H%M%S")
    mp4_path = f"/tmp/{base}.mp4"
    avi_path = f"/tmp/{base}.avi"

    save_fps = fps / max(1, record_step)
    if save_fps < 1:
        save_fps = 1

    writer = cv2.VideoWriter(mp4_path, cv2.VideoWriter_fourcc(*"mp4v"), save_fps, (width, height))
    if writer.isOpened():
        return writer, mp4_path

    writer.release()
    writer = cv2.VideoWriter(avi_path, cv2.VideoWriter_fourcc(*"XVID"), save_fps, (width, height))
    if writer.isOpened():
        return writer, avi_path

    writer.release()
    raise RuntimeError("Failed to create temporary video writer.")


def unique_path(path):
    if not os.path.exists(path):
        return path
    stem, ext = os.path.splitext(path)
    idx = 1
    while True:
        p = f"{stem}_{idx}{ext}"
        if not os.path.exists(p):
            return p
        idx += 1


def ask_save_recording(tmp_video):
    if not tmp_video or not os.path.exists(tmp_video):
        print("[Recorder] No temporary video found.")
        return

    try:
        choice = input("\n是否保存本次录像? [y/N]: ").strip().lower()
    except EOFError:
        choice = "n"

    if choice not in ("y", "yes"):
        try:
            os.remove(tmp_video)
            print(f"[Recorder] 已丢弃临时录像: {tmp_video}")
        except OSError:
            pass
        return

    default_dir = "/root/runs/videos"
    try:
        save_dir = input(f"保存目录(默认 {default_dir}): ").strip()
    except EOFError:
        save_dir = ""
    if not save_dir:
        save_dir = default_dir

    os.makedirs(save_dir, exist_ok=True)

    ts = datetime.now().strftime("cam_%Y%m%d_%H%M%S")
    ext = os.path.splitext(tmp_video)[1] or ".mp4"
    try:
        name = input(f"文件名(不含扩展, 默认 {ts}): ").strip()
    except EOFError:
        name = ""
    if not name:
        name = ts

    if name.lower().endswith(ext.lower()):
        final_path = os.path.join(save_dir, name)
    else:
        final_path = os.path.join(save_dir, name + ext)
    final_path = unique_path(final_path)

    shutil.move(tmp_video, final_path)
    print(f"[Recorder] 录像已保存: {final_path}")


def trigger_alarm(alarm_cmd):
    try:
        subprocess.Popen(alarm_cmd, shell=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        print("[ALARM] triggered:", alarm_cmd)
    except Exception as e:
        print("[ALARM] failed:", e)


def atomic_write_json(path, data):
    if not path:
        return
    tmp_dir = os.path.dirname(path) or "."
    os.makedirs(tmp_dir, exist_ok=True)
    fd, tmp_path = tempfile.mkstemp(prefix=".face_follow_", suffix=".json", dir=tmp_dir)
    try:
        payload = apply_checksum(data)
        with os.fdopen(fd, "wb") as f:
            f.write(canonical_json_bytes(payload))
            f.flush()
            os.fsync(f.fileno())
        try:
            os.replace(tmp_path, path)
        except OSError as exc:
            if exc.errno == errno.EXDEV:
                raise RuntimeError(f"rename across filesystems is not allowed: tmp={tmp_path} dst={path}") from exc
            raise
    finally:
        if os.path.exists(tmp_path):
            try:
                os.remove(tmp_path)
            except OSError:
                pass


def append_jsonl(path, data):
    if not path:
        return
    out_dir = os.path.dirname(path) or "."
    os.makedirs(out_dir, exist_ok=True)
    with open(path, "a", encoding="utf-8") as f:
        f.write(json.dumps(data, ensure_ascii=False) + "\n")


def track_color(track):
    name = str(getattr(track, "name", "")).lower()
    if name == "unknown":
        return (48, 64, 255)
    if name == "owner":
        return (76, 220, 92)
    return (255, 200, 0)


def draw_label_box(frame, x1, y1, label, color):
    font = cv2.FONT_HERSHEY_SIMPLEX
    scale = 0.58
    thickness = 2
    (tw, th), baseline = cv2.getTextSize(label, font, scale, thickness)
    pad_x = 8
    pad_y = 6
    box_left = max(0, int(x1))
    box_top = max(0, int(y1 - th - baseline - pad_y * 2 - 2))
    box_right = min(frame.shape[1] - 1, box_left + tw + pad_x * 2)
    box_bottom = min(frame.shape[0] - 1, box_top + th + baseline + pad_y * 2)

    overlay = frame.copy()
    cv2.rectangle(overlay, (box_left, box_top), (box_right, box_bottom), color, -1)
    cv2.addWeighted(overlay, 0.82, frame, 0.18, 0.0, frame)
    cv2.putText(
        frame,
        label,
        (box_left + pad_x, box_bottom - baseline - pad_y + 1),
        font,
        scale,
        (255, 255, 255),
        thickness,
        cv2.LINE_AA,
    )


def draw_runtime_hud(
    frame,
    detector_mode,
    stream_fps,
    loop_fps,
    infer_ms,
    rpc_ms,
    remote_server_ms,
    tracks,
    follow_target,
    remote_route="",
):
    h, w = frame.shape[:2]
    owner_count = sum(1 for tr in tracks if str(getattr(tr, "name", "")).lower() == "owner")
    unknown_count = sum(1 for tr in tracks if str(getattr(tr, "name", "")).lower() == "unknown")
    mode_text = "REMOTE" if detector_mode == "remote" else "LOCAL"
    accent = (255, 168, 0) if detector_mode == "remote" else (90, 210, 90)
    follow_text = "none"
    if follow_target is not None:
        follow_text = f"id{int(follow_target.id)}:{follow_target.name}"

    line1 = f"{mode_text}  STREAM {stream_fps:4.1f} FPS  tracks {len(tracks)}  owner {owner_count}  unknown {unknown_count}"
    if detector_mode == "remote":
        route_suffix = f"   route {str(remote_route)}" if remote_route else ""
        line2 = (
            f"loop {loop_fps:4.1f} fps   infer {infer_ms:5.1f} ms   rpc {rpc_ms:5.1f} ms   "
            f"server {remote_server_ms:5.1f} ms{route_suffix}"
        )
    else:
        line2 = f"loop {loop_fps:4.1f} fps   infer {infer_ms:5.1f} ms"
    line3 = f"follow {follow_text}"

    panel_left = 12
    panel_top = 12
    panel_right = min(w - 12, panel_left + 540)
    panel_bottom = min(h - 12, panel_top + 90)

    overlay = frame.copy()
    cv2.rectangle(overlay, (panel_left, panel_top), (panel_right, panel_bottom), (18, 18, 18), -1)
    cv2.rectangle(overlay, (panel_left, panel_top), (panel_right, panel_top + 6), accent, -1)
    cv2.addWeighted(overlay, 0.72, frame, 0.28, 0.0, frame)

    font = cv2.FONT_HERSHEY_SIMPLEX
    cv2.putText(frame, line1, (panel_left + 12, panel_top + 28), font, 0.62, (245, 245, 245), 2, cv2.LINE_AA)
    cv2.putText(frame, line2, (panel_left + 12, panel_top + 54), font, 0.55, (210, 210, 210), 1, cv2.LINE_AA)
    cv2.putText(frame, line3, (panel_left + 12, panel_top + 78), font, 0.55, accent, 1, cv2.LINE_AA)


def make_stream_metadata(
    detector_mode,
    detector_backend,
    active_model,
    active_device,
    stream_fps,
    loop_fps,
    infer_ms,
    rpc_ms,
    remote_server_ms,
    tracks,
    follow_target,
    remote_route="",
    raw_det_count=0,
    filtered_det_count=0,
    track_debug=None,
    db_build_version="unknown",
    db_aggregate_mode="unknown",
):
    owner_count = sum(1 for tr in tracks if str(getattr(tr, "name", "")).lower() == "owner")
    unknown_count = sum(1 for tr in tracks if str(getattr(tr, "name", "")).lower() == "unknown")
    follow_text = "none"
    if follow_target is not None:
        follow_text = f"id{int(follow_target.id)}:{follow_target.name}"
    return {
        "detector_mode": str(detector_mode),
        "detector_backend": str(detector_backend),
        "active_model": str(active_model),
        "active_device": str(active_device),
        "stream_fps": round(float(stream_fps), 2),
        "loop_fps": round(float(loop_fps), 2),
        "infer_ms": round(float(infer_ms), 2),
        "rpc_ms": round(float(rpc_ms), 2),
        "remote_server_ms": round(float(remote_server_ms), 2),
        "remote_route": str(remote_route),
        "raw_det_count": int(raw_det_count),
        "filtered_det_count": int(filtered_det_count),
        "db_build_version": str(db_build_version),
        "db_aggregate_mode": str(db_aggregate_mode),
        "tracks_total": int(len(tracks)),
        "owner_count": int(owner_count),
        "unknown_count": int(unknown_count),
        "follow_target": follow_text,
        "track_debug": list(track_debug or []),
        "ts": round(time.time(), 3),
    }


def choose_follow_target(
    tracks,
    follow_policy,
    follow_name,
    min_unknown_streak=3,
    min_unknown_conf=0.0,
    preferred_track_id=None,
    max_lost=0,
    frame_id=None,
    require_unknown_face_evidence=False,
    unknown_face_max_gap=0,
    unknown_face_min_score=0.0,
):
    if not tracks:
        return None

    fresh_tracks = [tr for tr in tracks if getattr(tr, "lost", 999999) <= 0]
    if not fresh_tracks:
        return None

    def rank_key(tr):
        x1, y1, x2, y2 = tr.bbox
        area = max(0, x2 - x1) * max(0, y2 - y1)
        return (float(tr.score), float(tr.conf), area)

    if follow_policy == "none":
        return None

    def prefer_existing(candidates):
        if preferred_track_id is None:
            return None
        for tr in candidates:
            if int(tr.id) == int(preferred_track_id):
                return tr
        return None

    if follow_policy == "owner" and follow_name:
        exact = [tr for tr in fresh_tracks if tr.name == follow_name and tr.score >= 0.35]
        if exact:
            keep = prefer_existing(exact)
            if keep is not None:
                return keep
            return max(exact, key=rank_key)
        return None

    if follow_policy == "unknown":
        if preferred_track_id is not None:
            for tr in fresh_tracks:
                if int(tr.id) == int(preferred_track_id) and float(tr.conf) >= float(min_unknown_conf):
                    return tr
        unknowns = [
            tr
            for tr in fresh_tracks
            if tr.name == "unknown"
            and tr.unknown_streak >= min_unknown_streak
            and float(tr.conf) >= float(min_unknown_conf)
            and (
                not require_unknown_face_evidence
                or has_recent_face_evidence(
                    tr,
                    frame_id,
                    unknown_face_max_gap,
                    min_score=unknown_face_min_score,
                    required_raw_name="unknown",
                )
            )
        ]
        if unknowns:
            keep = prefer_existing(unknowns)
            if keep is not None:
                return keep
            return max(unknowns, key=rank_key)
        return None

    named = [tr for tr in fresh_tracks if tr.name != "unknown" and tr.score >= 0.35]
    if named:
        keep = prefer_existing(named)
        if keep is not None:
            return keep
        return max(named, key=rank_key)
    return None


def face_refresh_stats(track, frame_id):
    frames_since_face = int(frame_id - track.last_face_frame) if track.last_face_frame >= 0 else 10**9
    current_area = track.area()
    area_growth = 0.0
    if track.last_face_area > 1.0:
        area_growth = current_area / track.last_face_area
    return frames_since_face, current_area, area_growth


def should_refresh_face_for_track(track, frame_id, args):
    if getattr(track, "lost", 999999) > 0:
        return False

    tw, th = track.size()
    if tw < args.min_face_track_width or th < args.min_face_track_height:
        return False

    if track.last_face_frame < 0:
        return True

    frames_since_face, _, area_growth = face_refresh_stats(track, frame_id)
    refresh_interval = args.face_interval if track.name == "unknown" else args.face_known_interval
    refresh_interval = max(1, int(refresh_interval))

    if track.score < args.face_keep_score:
        refresh_interval = min(refresh_interval, max(1, int(args.face_interval)))

    # Let larger / clearer targets cut in line even before the normal refresh period.
    if area_growth >= args.face_growth_trigger and frames_since_face >= args.face_recheck_min_gap:
        return True

    return frames_since_face >= refresh_interval


def select_tracks_for_face_refresh(tracks, frame_id, args):
    due = [tr for tr in tracks if should_refresh_face_for_track(tr, frame_id, args)]
    if not due:
        return []

    def priority(track):
        frames_since_face, current_area, area_growth = face_refresh_stats(track, frame_id)
        return (
            1 if track.name == "unknown" else 0,
            1 if track.score < args.face_keep_score else 0,
            float(area_growth),
            int(track.unknown_streak),
            float(current_area),
            float(track.conf),
            int(frames_since_face),
        )

    due.sort(reverse=True, key=priority)
    limit = max(1, int(args.max_face_tracks_per_cycle))
    return due[:limit]


def apply_fast_mode_defaults(args):
    if args.imgsz == 640:
        args.imgsz = 512
    if args.face_interval == 6:
        args.face_interval = 10
    if args.face_known_interval == 18:
        args.face_known_interval = 30
    if args.max_face_tracks_per_cycle == 2:
        args.max_face_tracks_per_cycle = 1
    if args.max_det_candidates == 300:
        args.max_det_candidates = 120
    if args.person_edge_min_conf == 0.55:
        args.person_edge_min_conf = 0.6
    if args.unknown_min_conf == 0.45:
        args.unknown_min_conf = 0.5
    if getattr(args, "detector_mode", "local") == "remote":
        if getattr(args, "remote_jpeg_quality", 85) == 85:
            args.remote_jpeg_quality = 70
        if getattr(args, "remote_scale", 1.0) == 1.0:
            args.remote_scale = 0.75
        if getattr(args, "remote_discovery_jpeg_quality", 68) == 68:
            args.remote_discovery_jpeg_quality = 60
        if getattr(args, "remote_discovery_scale", 0.60) == 0.60:
            args.remote_discovery_scale = 0.50
        if getattr(args, "remote_discovery_interval", 3) == 3:
            args.remote_discovery_interval = 4
        if getattr(args, "remote_roi_max_tracks", 2) == 2:
            args.remote_roi_max_tracks = 1


def track_display_name(track, unknown_label_frames=0, frame_id=None):
    provisional_name = provisional_display_name(track, frame_id if frame_id is not None else int(getattr(track, "last_seen", 0)))
    if provisional_name:
        return f"{provisional_name}?"
    name = str(getattr(track, "name", "unknown"))
    if name == "unknown" and int(getattr(track, "unknown_streak", 0)) < max(0, int(unknown_label_frames)):
        return "candidate"
    return name


def choose_render_target(tracks, control_track, preferred_track_id=None):
    if preferred_track_id is not None:
        for tr in tracks:
            if int(tr.id) == int(preferred_track_id):
                return tr
    if control_track is not None:
        return control_track
    if not tracks:
        return None
    return max(
        tracks,
        key=lambda tr: (
            int(getattr(tr, "lost", 999999) <= 0),
            bool(getattr(tr, "matched_this_frame", False)),
            float(getattr(tr, "score", 0.0)),
            float(getattr(tr, "conf", 0.0)),
        ),
    )


def write_follow_state(
    state_file,
    render_track,
    control_track,
    frame_id,
    frame_shape,
    follow_name,
    seq,
    unknown_face_max_gap=0,
    unknown_face_min_score=0.0,
):
    h, w = frame_shape[:2]
    now_ms = monotonic_ms()

    def bbox_norm(x1, y1, x2, y2):
        return [
            round(float(x1) / max(1.0, float(w)), 6),
            round(float(y1) / max(1.0, float(h)), 6),
            round(float(x2) / max(1.0, float(w)), 6),
            round(float(y2) / max(1.0, float(h)), 6),
        ]

    def serialize_render(track):
        if track is None:
            return None
        x1, y1, x2, y2 = track.bbox
        return {
            "track_id": int(track.id),
            "name": str(track.name),
            "display_name": track_display_name(track, frame_id=frame_id),
            "score": float(track.score),
            "confidence": float(track.conf),
            "lost": int(getattr(track, "lost", 0)),
            "stale": bool(getattr(track, "lost", 0) > 0),
            "matched_this_frame": bool(getattr(track, "matched_this_frame", False)),
            "last_face_raw_name": str(getattr(track, "last_face_raw_name", "none")),
            "last_face_raw_score": float(getattr(track, "last_face_raw_score", 0.0)),
            "last_face_top1_score": float(getattr(track, "last_face_top1_score", 0.0)),
            "last_face_topk_score": float(getattr(track, "last_face_topk_score", 0.0)),
            "face_evidence_state": str(getattr(track, "face_evidence_state", "none")),
            "identity_transition_reason": str(getattr(track, "identity_transition_reason", "")),
            "bbox_xyxy_norm": bbox_norm(x1, y1, x2, y2),
            "seq": int(seq),
            "ts_monotonic_ms": int(now_ms),
        }

    inactive_control = {
        "active": False,
        "control_ok": False,
        "track_id": None,
        "name": "",
        "lost": 0,
        "stale": False,
        "matched_this_frame": False,
        "has_recent_face_evidence": False,
        "confidence": 0.0,
        "area_ratio": 0.0,
        "bbox_xyxy_norm": [0.0, 0.0, 0.0, 0.0],
        "cx": float(w) / 2.0,
        "cy": float(h) / 2.0,
        "width": int(w),
        "height": int(h),
        "seq": int(seq),
        "ts_monotonic_ms": int(now_ms),
    }

    if control_track is not None:
        x1, y1, x2, y2 = control_track.bbox
        cx = 0.5 * (x1 + x2)
        cy = 0.5 * (y1 + y2)
        area_ratio = max(0.0, (x2 - x1) * (y2 - y1)) / max(1.0, float(w * h))
        lost = int(getattr(control_track, "lost", 0))
        stale = bool(lost > 0)
        matched_raw = bool(getattr(control_track, "matched_this_frame", True))
        matched = bool(matched_raw or lost == 0)
        if str(getattr(control_track, "name", "")) == "unknown":
            recent_face = has_recent_face_evidence(
                control_track,
                frame_id,
                unknown_face_max_gap,
                min_score=unknown_face_min_score,
            )
            unknown_streak = int(getattr(control_track, "unknown_streak", 0))
            control_ok = (not stale) and lost == 0 and matched and (
                bool(recent_face) or unknown_streak >= 4
            )
        else:
            recent_face = int(getattr(control_track, "last_face_frame", -1)) >= 0
            unknown_streak = int(getattr(control_track, "unknown_streak", 0))
            control_ok = (not stale) and lost == 0 and matched
        control_target = {
            "active": bool(control_ok),
            "control_ok": bool(control_ok),
            "track_id": int(control_track.id),
            "name": str(control_track.name),
            "render_name": track_display_name(control_track, frame_id=frame_id),
            "lost": int(lost),
            "stale": bool(stale),
            "matched_this_frame": bool(matched),
            "unknown_streak": int(unknown_streak),
            "has_recent_face_evidence": bool(recent_face),
            "last_face_raw_name": str(getattr(control_track, "last_face_raw_name", "none")),
            "last_face_raw_score": float(getattr(control_track, "last_face_raw_score", 0.0)),
            "last_face_top1_score": float(getattr(control_track, "last_face_top1_score", 0.0)),
            "last_face_topk_score": float(getattr(control_track, "last_face_topk_score", 0.0)),
            "face_evidence_state": str(getattr(control_track, "face_evidence_state", "none")),
            "confidence": float(control_track.conf),
            "score": float(control_track.score),
            "area_ratio": float(area_ratio),
            "bbox_xyxy_norm": bbox_norm(x1, y1, x2, y2),
            "cx": float(cx),
            "cy": float(cy),
            "width": int(w),
            "height": int(h),
            "seq": int(seq),
            "ts_monotonic_ms": int(now_ms),
        }
    else:
        control_target = inactive_control

    payload = {
        "schema_version": FOLLOW_STATE_SCHEMA_VERSION,
        "follow_name": str(follow_name),
        "ts_monotonic_ms": int(now_ms),
        "meta": {
            "write_ts_monotonic_ms": int(now_ms),
            "frame_id": int(frame_id),
            "seq": int(seq),
            "checksum_sha256": "",
        },
        "render_target": serialize_render(render_track),
        "control_target": control_target,
    }
    atomic_write_json(state_file, payload)


def main():
    ap = argparse.ArgumentParser(description="YOLO OM + TrackID + Face Recognition + Unknown Alarm")
    ap.add_argument("--model", default="/root/yolov5s_310b.om")
    ap.add_argument("--device", type=int, default=0)
    ap.add_argument("--camera", default="0")
    ap.add_argument("--width", type=int, default=1280)
    ap.add_argument("--height", type=int, default=720)
    ap.add_argument("--camera-fps", type=float, default=60.0, help="Requested camera FPS before capture starts.")
    ap.add_argument("--camera-fourcc", default="MJPG", help="Requested camera FOURCC, e.g. MJPG or YUYV.")
    ap.add_argument("--imgsz", type=int, default=640)
    ap.add_argument("--detector-mode", choices=["local", "remote"], default="local")
    ap.add_argument("--remote-url", default="", help="Remote detect endpoint, e.g. http://127.0.0.1:8765/detect")
    ap.add_argument("--remote-timeout", type=float, default=15.0)
    ap.add_argument("--remote-jpeg-quality", type=int, default=75)
    ap.add_argument("--remote-scale", type=float, default=1.0, help="Resize factor before JPEG encoding for remote detector (<=1.0).")
    ap.add_argument("--remote-max-failures", type=int, default=20)
    ap.add_argument("--remote-discovery-interval", type=int, default=3, help="Run a full-frame remote discovery pass every N inference cycles.")
    ap.add_argument("--remote-discovery-conf", type=float, default=0.0, help="Detector confidence for full-frame discovery uploads (0 uses --conf).")
    ap.add_argument("--remote-discovery-jpeg-quality", type=int, default=68, help="JPEG quality for low-cost full-frame discovery uploads.")
    ap.add_argument("--remote-discovery-scale", type=float, default=0.60, help="Resize factor for low-cost full-frame discovery uploads.")
    ap.add_argument("--remote-roi-max-tracks", type=int, default=2, help="How many active tracks may receive high-quality ROI refinement per inference cycle.")
    ap.add_argument("--remote-roi-expand", type=float, default=0.20, help="Expand each ROI crop by this ratio before sending it to the remote detector.")
    ap.add_argument("--remote-roi-conf", type=float, default=0.0, help="Detector confidence for ROI refinement uploads (0 picks a conservative value between --track-low-conf and --conf).")
    ap.add_argument("--remote-roi-jpeg-quality", type=int, default=92, help="JPEG quality for ROI refinement uploads.")
    ap.add_argument("--remote-roi-scale", type=float, default=1.0, help="Resize factor for ROI refinement uploads (<=1.0).")
    ap.add_argument("--fast-mode", action="store_true", help="Use lighter settings for better real-time performance.")
    ap.add_argument("--conf", type=float, default=0.35)
    ap.add_argument("--iou", type=float, default=0.45)
    ap.add_argument("--target-cls", type=int, default=0, help="-1 keeps all classes, 0 keeps person")
    ap.add_argument("--infer-interval", type=int, default=2)
    ap.add_argument("--face-interval", type=int, default=6)
    ap.add_argument("--face-known-interval", type=int, default=18)
    ap.add_argument("--max-face-tracks-per-cycle", type=int, default=2)
    ap.add_argument("--min-face-track-width", type=int, default=60)
    ap.add_argument("--min-face-track-height", type=int, default=80)
    ap.add_argument("--face-recheck-min-gap", type=int, default=3)
    ap.add_argument("--face-growth-trigger", type=float, default=1.35)
    ap.add_argument("--face-keep-score", type=float, default=0.55)
    ap.add_argument("--owner-accept-score", type=float, default=0.68, help="Minimum raw/smoothed face-match score required before a known identity may be accepted.")
    ap.add_argument("--owner-confirm-hits", type=int, default=3, help="How many consistent face refreshes are required before upgrading unknown to a known identity.")
    ap.add_argument("--known-hold-misses", type=int, default=2, help="How many weak face refreshes a known identity can survive before falling back to unknown.")
    ap.add_argument("--known-hold-score", type=float, default=0.42, help="Minimum smoothed known score required to temporarily hold identity on a weak refresh.")
    ap.add_argument("--track-iou", type=float, default=0.35)
    ap.add_argument("--track-max-lost", type=int, default=20)
    ap.add_argument("--track-low-conf", type=float, default=0.12, help="Low-score detection floor for ByteTrack-style second-stage association.")
    ap.add_argument("--new-track-iou-gate", type=float, default=0.25, help="Do not spawn a new track from a discovery box if it already overlaps an existing track by at least this IOU.")
    ap.add_argument("--follow-max-lost", type=int, default=2, help="Keep the current follow target for a few missed frames before dropping it.")
    ap.add_argument("--min-person-width", type=int, default=48)
    ap.add_argument("--min-person-height", type=int, default=96)
    ap.add_argument("--min-person-area", type=int, default=6000)
    ap.add_argument("--person-edge-margin", type=float, default=0.02, help="Reject weak person boxes touching left/right image edge within this width ratio.")
    ap.add_argument("--person-edge-min-conf", type=float, default=0.55, help="Minimum detector confidence required to keep left/right edge-touching person boxes.")
    ap.add_argument("--max-det-candidates", type=int, default=300)
    ap.add_argument("--print-every", type=int, default=0)
    ap.add_argument("--profile-every", type=int, default=0, help="Print stage timing JSON every N frames.")
    ap.add_argument("--record-step", type=int, default=2)
    ap.add_argument("--det-model", default="/root/face_models/face_detection_yunet_2023mar.onnx")
    ap.add_argument("--rec-model", default="/root/face_models/face_recognition_sface_2021dec.onnx")
    ap.add_argument("--face-db", default="/root/face_db/embeddings.npz")
    ap.add_argument("--face-det-thres", type=float, default=0.85, help="YuNet face detection threshold used at runtime; should match DB-building conditions.")
    ap.add_argument("--match-thres", type=float, default=0.36)
    ap.add_argument("--unknown-frames", type=int, default=12)
    ap.add_argument("--unknown-label-frames", type=int, default=3, help="Show short-lived unknown tracks as candidate before promoting the label.")
    ap.add_argument("--unknown-min-conf", type=float, default=0.45, help="Minimum detector confidence required before an unknown track may trigger follow/alarm logic.")
    ap.add_argument("--unknown-face-evidence-max-gap", type=int, default=18, help="Require an unknown track to have recent face evidence within this many frames before follow/alarm may treat it as a true unknown person.")
    ap.add_argument("--unknown-face-evidence-score", type=float, default=0.30, help="Minimum raw face-match score required before a recent face refresh counts as evidence for unknown follow/alarm logic.")
    ap.add_argument("--alarm-cooldown", type=float, default=8.0)
    ap.add_argument("--alarm-cmd", default="/usr/local/miniconda3/bin/python /root/alarm_atlas.py 2")
    ap.add_argument("--follow-name", default="owner", help="Preferred recognized name to follow")
    ap.add_argument("--follow-policy", default="owner", choices=["owner", "named", "unknown", "none"])
    ap.add_argument("--follow-unknown-min-frames", type=int, default=6)
    ap.add_argument("--state-file", default="/tmp/face_follow_state.json", help="Shared state file for follow controller")
    ap.add_argument("--no-record", action="store_true", help="Disable temporary video recording")
    ap.add_argument("--no-show", action="store_true")
    ap.add_argument("--camera-power-line", type=int, default=1, help="v4l2 power_line_frequency to apply before capture (1=50Hz, 2=60Hz).")
    ap.add_argument("--camera-exposure-auto-priority", type=int, default=0, help="v4l2 exposure_auto_priority to apply before capture.")
    ap.add_argument("--stream-port", type=int, default=0, help="Serve annotated MJPEG stream on this port (0 disables).")
    ap.add_argument("--stream-jpeg-quality", type=int, default=80, help="JPEG quality for browser stream.")
    ap.add_argument("--stream-max-fps", type=float, default=8.0, help="Max FPS for MJPEG stream updates.")
    ap.add_argument("--stream-scale", type=float, default=1.0, help="Resize factor for browser stream encoding (<=1.0).")
    ap.add_argument("--video-stream-url", default="", help="Optional low-latency ffmpeg output URL, e.g. udp://192.168.5.15:5600?pkt_size=1316")
    ap.add_argument("--video-stream-fps", type=float, default=30.0, help="Max FPS for optional ffmpeg video stream.")
    ap.add_argument("--video-stream-crf", type=int, default=20, help="CRF for optional H.264 ffmpeg stream.")
    ap.add_argument("--event-log", default="", help="Optional JSONL file for recognition/alarm/follow target events.")
    ap.add_argument("--identity-inherit-max-lost", type=int, default=6, help="Allow a known-but-lost track to seed provisional identity for a nearby fresh track within this lost-frame window.")
    ap.add_argument("--identity-inherit-new-track-window", type=int, default=2, help="Only newly spawned tracks within this many frames may receive provisional identity inheritance.")
    ap.add_argument("--identity-inherit-center-dist", type=float, default=0.12, help="Maximum normalized center distance for provisional identity inheritance.")
    ap.add_argument("--identity-inherit-area-ratio", type=float, default=1.9, help="Maximum area ratio between old/new tracks when inheriting a provisional identity.")
    ap.add_argument("--identity-inherit-hold-frames", type=int, default=12, help="How long a provisional inherited identity may be displayed while waiting for fresh face confirmation.")
    ap.add_argument("--identity-inherit-min-score", type=float, default=0.60, help="Minimum confirmed identity score required before a lost track may seed provisional inheritance.")
    args = ap.parse_args()

    if args.fast_mode:
        apply_fast_mode_defaults(args)

    args.infer_interval = max(1, args.infer_interval)
    args.face_interval = max(1, args.face_interval)
    args.face_known_interval = max(args.face_interval, int(args.face_known_interval))
    args.max_face_tracks_per_cycle = max(1, int(args.max_face_tracks_per_cycle))
    args.min_face_track_width = max(1, int(args.min_face_track_width))
    args.min_face_track_height = max(1, int(args.min_face_track_height))
    args.face_recheck_min_gap = max(1, int(args.face_recheck_min_gap))
    args.face_growth_trigger = max(1.0, float(args.face_growth_trigger))
    args.face_keep_score = min(max(0.0, float(args.face_keep_score)), 1.0)
    args.owner_accept_score = min(max(0.0, float(args.owner_accept_score)), 1.0)
    args.owner_confirm_hits = max(2, int(args.owner_confirm_hits))
    args.known_hold_misses = max(0, int(args.known_hold_misses))
    args.known_hold_score = min(max(0.0, float(args.known_hold_score)), 1.0)
    args.min_person_width = max(1, int(args.min_person_width))
    args.min_person_height = max(1, int(args.min_person_height))
    args.min_person_area = max(1, int(args.min_person_area))
    args.person_edge_margin = min(0.2, max(0.0, float(args.person_edge_margin)))
    args.person_edge_min_conf = min(max(0.0, float(args.person_edge_min_conf)), 1.0)
    args.max_det_candidates = max(0, int(args.max_det_candidates))
    args.track_low_conf = min(max(0.01, float(args.track_low_conf)), float(args.conf))
    args.new_track_iou_gate = min(max(0.0, float(args.new_track_iou_gate)), 0.95)
    args.follow_max_lost = max(0, int(args.follow_max_lost))
    args.unknown_label_frames = max(0, int(args.unknown_label_frames))
    args.unknown_min_conf = min(max(0.0, float(args.unknown_min_conf)), 1.0)
    args.face_det_thres = min(max(0.5, float(args.face_det_thres)), 0.99)
    args.unknown_face_evidence_max_gap = max(0, int(args.unknown_face_evidence_max_gap))
    args.unknown_face_evidence_score = min(max(0.0, float(args.unknown_face_evidence_score)), 1.0)
    args.profile_every = max(0, int(args.profile_every))
    args.record_step = max(1, args.record_step)
    args.camera_power_line = max(0, int(args.camera_power_line))
    args.camera_exposure_auto_priority = max(0, min(1, int(args.camera_exposure_auto_priority)))
    args.stream_port = max(0, int(args.stream_port))
    args.stream_jpeg_quality = min(max(30, int(args.stream_jpeg_quality)), 95)
    args.stream_max_fps = max(0.0, float(args.stream_max_fps))
    args.stream_scale = min(1.0, max(0.25, float(args.stream_scale)))
    args.video_stream_fps = max(0.0, float(args.video_stream_fps))
    args.video_stream_crf = min(max(12, int(args.video_stream_crf)), 35)
    args.identity_inherit_max_lost = max(1, int(args.identity_inherit_max_lost))
    args.identity_inherit_new_track_window = max(1, int(args.identity_inherit_new_track_window))
    args.identity_inherit_center_dist = min(0.5, max(0.01, float(args.identity_inherit_center_dist)))
    args.identity_inherit_area_ratio = max(1.0, float(args.identity_inherit_area_ratio))
    args.identity_inherit_hold_frames = max(1, int(args.identity_inherit_hold_frames))
    args.identity_inherit_min_score = min(max(0.0, float(args.identity_inherit_min_score)), 1.0)
    args.remote_timeout = max(1.0, float(args.remote_timeout))
    args.remote_jpeg_quality = min(max(30, int(args.remote_jpeg_quality)), 95)
    args.remote_scale = min(1.0, max(0.25, float(args.remote_scale)))
    args.remote_max_failures = max(1, int(args.remote_max_failures))
    args.remote_discovery_interval = max(1, int(args.remote_discovery_interval))
    args.remote_discovery_conf = float(args.remote_discovery_conf)
    args.remote_discovery_jpeg_quality = min(max(30, int(args.remote_discovery_jpeg_quality)), 95)
    args.remote_discovery_scale = min(1.0, max(0.25, float(args.remote_discovery_scale)))
    args.remote_roi_max_tracks = max(0, int(args.remote_roi_max_tracks))
    args.remote_roi_expand = min(0.50, max(0.0, float(args.remote_roi_expand)))
    args.remote_roi_conf = float(args.remote_roi_conf)
    args.remote_roi_jpeg_quality = min(max(30, int(args.remote_roi_jpeg_quality)), 95)
    args.remote_roi_scale = min(1.0, max(0.25, float(args.remote_roi_scale)))
    if args.remote_discovery_conf <= 0.0:
        args.remote_discovery_conf = float(args.conf)
    else:
        args.remote_discovery_conf = min(max(args.track_low_conf, float(args.remote_discovery_conf)), 1.0)
    if args.remote_roi_conf <= 0.0:
        args.remote_roi_conf = float(max(args.track_low_conf, min(args.conf, 0.22)))
    else:
        args.remote_roi_conf = min(max(args.track_low_conf, float(args.remote_roi_conf)), float(args.conf))
    if args.remote_jpeg_quality != 75 and args.remote_roi_jpeg_quality == 92:
        args.remote_roi_jpeg_quality = int(args.remote_jpeg_quality)
    if abs(args.remote_scale - 1.0) > 1e-6 and abs(args.remote_roi_scale - 1.0) < 1e-6:
        args.remote_roi_scale = float(args.remote_scale)
    if args.remote_jpeg_quality != 75 and args.remote_discovery_jpeg_quality == 68:
        args.remote_discovery_jpeg_quality = int(min(args.remote_jpeg_quality, 68))
    if abs(args.remote_scale - 1.0) > 1e-6 and abs(args.remote_discovery_scale - 0.60) < 1e-6:
        args.remote_discovery_scale = float(min(args.remote_scale, 0.60))

    backend = "remote"
    sess = None
    remote_client = None
    async_remote = None
    active_model = args.model
    active_device = f"device:{args.device}"
    if args.detector_mode == "local":
        backend, sess = load_backend(args.model, args.device)
        fixed_input_size = detect_model_input_size(sess)
        if fixed_input_size and fixed_input_size != args.imgsz:
            print(
                f"[INFO] overriding imgsz={args.imgsz} with model input size {fixed_input_size}"
            )
            args.imgsz = fixed_input_size
        active_model = args.model
        active_device = f"Ascend device {args.device}"
    else:
        if not args.remote_url:
            raise ValueError("--remote-url is required when --detector-mode remote")
        remote_client = RemoteDetectorClient(args.remote_url, args.remote_timeout)
        health = remote_healthcheck(remote_client)
        active_model = str(health.get("model") or "remote-unknown")
        active_device = str(health.get("device_name") or health.get("device") or "remote-unknown")
        print(
            f"[REMOTE] detector={args.remote_url}, model={health.get('model')}, "
            f"device={health.get('device_name', health.get('device'))}, warmup_ms={health.get('warmup_ms')}"
        )
    print(
        f"backend={backend}, active_model={active_model}, active_device={active_device}, "
        f"infer_interval={args.infer_interval}, face_interval={args.face_interval}, target_cls={args.target_cls}"
    )

    db_names, db_feats, db_meta = load_face_db(args.face_db)
    db_method = str(db_meta.get("method", "unknown"))
    db_build_version = str(db_meta.get("build_version", "unknown"))
    db_aggregate_mode = str(db_meta.get("aggregate", "unknown"))
    face_mode, detector, recognizer = choose_face_backend(args.det_model, args.rec_model, det_thres=args.face_det_thres)
    face_enabled = len(db_names) > 0 and len(db_feats) > 0
    if face_enabled:
        if db_method != "unknown" and db_method != face_mode:
            print(
                f"[FACE] DB method={db_method}, runtime method={face_mode}. "
                "Please rebuild DB with current backend."
            )
            face_enabled = False
        else:
            print(
                f"[FACE] backend={face_mode}, loaded entries={len(db_names)}, "
                f"aggregate={db_aggregate_mode}, build_version={db_build_version} from {args.face_db}"
            )
    if not face_enabled:
        print("[FACE] disabled (empty DB or method mismatch).")

    camera_source = int(args.camera) if str(args.camera).strip().isdigit() else args.camera

    apply_camera_runtime_controls(
        resolve_v4l2_device(args.camera),
        power_line_frequency=args.camera_power_line,
        exposure_auto_priority=args.camera_exposure_auto_priority,
    )

    cap = cv2.VideoCapture(camera_source, cv2.CAP_V4L2)
    if not cap.isOpened():
        cap.release()
        cap = cv2.VideoCapture(camera_source)
    fourcc = str(args.camera_fourcc or "").strip().upper()
    if fourcc:
        try:
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fourcc[:4]))
        except Exception:
            pass
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    if args.camera_fps > 0:
        cap.set(cv2.CAP_PROP_FPS, float(args.camera_fps))
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    if not cap.isOpened():
        raise RuntimeError("Cannot open camera.")

    fps = cap.get(cv2.CAP_PROP_FPS)
    if not fps or fps < 1:
        fps = 20.0

    writer = None
    tmp_video = None
    stream_store = None
    stream_server = None
    stream_min_gap = 0.0
    last_stream_push = 0.0
    video_streamer = None
    video_min_gap = 0.0
    last_video_push = 0.0
    if args.stream_port > 0:
        stream_store = LatestFrameStore()
        stream_server = start_mjpeg_server(args.stream_port, stream_store)
        if args.stream_max_fps > 0:
            stream_min_gap = 1.0 / args.stream_max_fps
        print(
            f"[STREAM] viewer ready: http://0.0.0.0:{args.stream_port}/ "
            f"(also /stream.mjpg and /latest.jpg)"
        )
    if args.video_stream_url and args.video_stream_fps > 0:
        video_min_gap = 1.0 / args.video_stream_fps
        print(f"[VIDEO] low-latency stream target: {args.video_stream_url} @ {args.video_stream_fps:.1f} fps")

    tracker = IOUTracker(
        iou_thres=args.track_iou,
        max_lost=args.track_max_lost,
        high_conf_thres=args.conf,
        low_conf_thres=args.track_low_conf,
        new_track_iou_gate=args.new_track_iou_gate,
    )
    preview_async_remote = bool(
        args.detector_mode == "remote"
        and args.follow_policy == "none"
        and args.no_show
        and args.no_record
        and stream_store is not None
    )
    remote_stale_ttl_s = 0.25
    last_async_result_id = -1
    last_async_result_ts = 0.0
    if preview_async_remote and remote_client is not None:
        async_remote = AsyncRemoteDetector(
            remote_client,
            args.remote_discovery_conf,
            args.remote_roi_conf,
            args.imgsz,
            args.infer_interval,
            args.remote_discovery_interval,
            args.remote_discovery_jpeg_quality,
            args.remote_discovery_scale,
            args.remote_roi_max_tracks,
            args.remote_roi_expand,
            args.remote_roi_jpeg_quality,
            args.remote_roi_scale,
            args.track_low_conf,
            args.target_cls,
            args.iou,
        )
        print(
            f"[REMOTE] async preview mode enabled: stale_ttl_ms={int(remote_stale_ttl_s * 1000)}"
        )
    last_alarm_ts = 0.0
    last_dets = []
    last_infer_ms = 0.0
    last_rpc_ms = 0.0
    last_remote_encode_ms = 0.0
    last_remote_server_ms = 0.0
    remote_failures = 0
    last_remote_error_ts = 0.0
    last_remote_route = "idle"
    last_raw_det_count = 0
    last_filtered_det_count = 0
    fid = 0
    last_follow_track_id = None
    last_follow_name = ""
    follow_state_seq = 0
    loop_fps = 0.0
    stream_fps = 0.0
    last_loop_ts = None
    last_stream_emit_ts = None
    while True:
        loop_t0 = time.time()
        if last_loop_ts is not None:
            frame_dt = max(1e-6, loop_t0 - last_loop_ts)
            instant_fps = 1.0 / frame_dt
            if loop_fps <= 0.0:
                loop_fps = instant_fps
            else:
                loop_fps = loop_fps * 0.85 + instant_fps * 0.15
        preprocess_ms = 0.0
        face_ms = 0.0
        draw_ms = 0.0
        rpc_ms = 0.0
        remote_encode_ms = 0.0
        remote_server_ms = 0.0

        ok, frame = cap.read()
        if (not ok) or frame is None or getattr(frame, "size", 0) == 0:
            print("[WARN] camera read failed; reopening capture")
            try:
                cap.release()
            except Exception:
                pass
            time.sleep(0.2)
            apply_camera_runtime_controls(
                resolve_v4l2_device(args.camera),
                power_line_frequency=args.camera_power_line,
                exposure_auto_priority=args.camera_exposure_auto_priority,
            )
            cap = cv2.VideoCapture(camera_source, cv2.CAP_V4L2)
            if not cap.isOpened():
                cap.release()
                cap = cv2.VideoCapture(camera_source)
            if fourcc:
                try:
                    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fourcc[:4]))
                except Exception:
                    pass
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
            if args.camera_fps > 0:
                cap.set(cv2.CAP_PROP_FPS, float(args.camera_fps))
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            time.sleep(0.15)
            continue
        tracks_snapshot = snapshot_tracks_for_remote_roi(list(tracker.tracks.values()))

        if writer is None and not args.no_record:
            h, w = frame.shape[:2]
            writer, tmp_video = make_temp_writer(w, h, fps, args.record_step)
            print(f"[Recorder] 临时录制中: {tmp_video}")

        run_this_frame = (fid == 0) or (fid % args.infer_interval == 0)
        dets = []
        have_fresh_dets = False
        if args.detector_mode == "remote" and async_remote is not None:
            if run_this_frame:
                async_remote.submit(fid, frame, tracks_snapshot)
            result = async_remote.latest()
            if result is not None and int(result.get("job_id", -1)) != last_async_result_id:
                last_async_result_id = int(result.get("job_id", -1))
                if bool(result.get("ok", False)):
                    raw_dets = list(result.get("dets", []))
                    last_raw_det_count = len(raw_dets)
                    dets = filter_person_detections(raw_dets, frame.shape, args)
                    last_filtered_det_count = len(dets)
                    last_dets = dets
                    have_fresh_dets = True
                    last_infer_ms = float(result.get("server_ms", 0.0))
                    last_rpc_ms = float(result.get("rpc_ms", 0.0))
                    last_remote_encode_ms = float(result.get("encode_ms", 0.0))
                    last_remote_server_ms = float(result.get("server_ms", 0.0))
                    last_remote_route = str(result.get("route", "async"))
                    last_async_result_ts = float(result.get("completed_ts", time.time()))
                    remote_failures = 0
                else:
                    remote_failures += 1
                    if time.time() - last_remote_error_ts > 2.0:
                        print(
                            f"[REMOTE] async detect failed ({remote_failures}/{args.remote_max_failures}): "
                            f"{result.get('error', 'unknown error')}"
                        )
                        last_remote_error_ts = time.time()
                    last_infer_ms = 0.0
                    last_rpc_ms = 0.0
                    last_remote_encode_ms = 0.0
                    last_remote_server_ms = 0.0
                    last_remote_route = "error"
                    last_raw_det_count = 0
                    last_filtered_det_count = 0
                    if remote_failures >= args.remote_max_failures:
                        raise RuntimeError(
                            f"remote detector failed {remote_failures} times in a row; aborting"
                        )
            if last_async_result_ts > 0.0 and (time.time() - last_async_result_ts) > remote_stale_ttl_s:
                dets = []
                last_infer_ms = 0.0
                last_rpc_ms = 0.0
                last_remote_encode_ms = 0.0
                last_remote_server_ms = 0.0
                last_remote_route = "stale"
                last_raw_det_count = 0
                last_filtered_det_count = 0
        elif run_this_frame:
            if args.detector_mode == "remote":
                try:
                    dets, remote_encode_ms, rpc_ms, remote_server_ms, remote_route = remote_detect_hybrid(
                        frame,
                        remote_client,
                        args.remote_discovery_conf,
                        args.remote_roi_conf,
                        args.imgsz,
                        fid,
                        args.infer_interval,
                        args.remote_discovery_interval,
                        args.remote_discovery_jpeg_quality,
                        args.remote_discovery_scale,
                        tracks_snapshot,
                        args.remote_roi_max_tracks,
                        args.remote_roi_expand,
                        args.remote_roi_jpeg_quality,
                        args.remote_roi_scale,
                        args.track_low_conf,
                        args.target_cls,
                        args.iou,
                    )
                    last_raw_det_count = len(dets)
                    dets = filter_person_detections(dets, frame.shape, args)
                    last_filtered_det_count = len(dets)
                    last_dets = dets
                    have_fresh_dets = True
                    last_infer_ms = float(remote_server_ms)
                    last_rpc_ms = float(rpc_ms)
                    last_remote_encode_ms = float(remote_encode_ms)
                    last_remote_server_ms = float(remote_server_ms)
                    last_remote_route = str(remote_route)
                    remote_failures = 0
                except Exception as e:
                    remote_failures += 1
                    if time.time() - last_remote_error_ts > 2.0:
                        print(f"[REMOTE] detect failed ({remote_failures}/{args.remote_max_failures}): {e}")
                        last_remote_error_ts = time.time()
                    dets = []
                    last_dets = dets
                    last_infer_ms = 0.0
                    last_rpc_ms = 0.0
                    last_remote_encode_ms = 0.0
                    last_remote_server_ms = 0.0
                    last_remote_route = "error"
                    last_raw_det_count = 0
                    last_filtered_det_count = 0
                    if remote_failures >= args.remote_max_failures:
                        raise RuntimeError(
                            f"remote detector failed {remote_failures} times in a row; aborting"
                        ) from e
            else:
                t_pre = time.time()
                img, ratio, dwdh = letterbox(frame, (args.imgsz, args.imgsz))
                img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
                inp = np.transpose(img, (2, 0, 1))[None, ...]
                inp = np.ascontiguousarray(inp)
                preprocess_ms = (time.time() - t_pre) * 1000.0

                t0 = time.time()
                raw = run_infer(backend, sess, inp)
                pred = normalize_output(raw)
                dets = postprocess(
                    pred,
                    ratio,
                    dwdh,
                    frame.shape,
                    args.conf,
                    args.iou,
                    args.target_cls,
                    args.max_det_candidates,
                    args.track_low_conf,
                )
                last_raw_det_count = len(dets)
                dets = filter_person_detections(dets, frame.shape, args)
                last_filtered_det_count = len(dets)
                last_infer_ms = (time.time() - t0) * 1000.0
                last_dets = dets
                have_fresh_dets = True

        tracks = tracker.update(dets, fid, detections_fresh=have_fresh_dets)
        apply_identity_inheritance(tracks, fid, frame.shape, args)
        now = time.time()

        if face_enabled:
            t_face = time.time()
            face_tracks = select_tracks_for_face_refresh(tracks, fid, args)
            for tr in face_tracks:
                prev_name = tr.name
                prev_score = float(tr.score)
                face_result = recognize_face_in_person(
                    face_mode,
                    detector,
                    recognizer,
                    db_names,
                    db_feats,
                    db_meta,
                    frame,
                    tr.bbox,
                    args.match_thres,
                    args.owner_accept_score,
                )
                name = str(face_result.get("name", "unknown"))
                score = float(face_result.get("score", 0.0))
                changed = tr.update_name(
                    name,
                    score,
                    known_hold_misses=args.known_hold_misses,
                    known_hold_score=args.known_hold_score,
                    min_accept_score=args.match_thres,
                    known_accept_score=args.owner_accept_score,
                    known_confirm_hits=args.owner_confirm_hits,
                )
                tr.last_face_raw_name = str(face_result.get("raw_name", "none"))
                tr.last_face_raw_score = float(face_result.get("score", 0.0))
                tr.last_face_top1_score = float(face_result.get("top1_score", 0.0))
                tr.last_face_topk_score = float(face_result.get("topk_score", 0.0))
                tr.last_face_match_count = int(face_result.get("match_count_above_threshold", 0))
                tr.face_evidence_state = str(face_result.get("face_evidence_state", "none"))
                if bool(face_result.get("face_detected", False)):
                    tr.last_face_frame = fid
                    tr.last_face_area = tr.area()
                if changed:
                    append_jsonl(
                        args.event_log,
                        {
                            "ts": time.time(),
                            "frame": int(fid),
                            "event": "identity_change",
                            "track_id": int(tr.id),
                            "from": prev_name,
                            "to": tr.name,
                            "raw_name": str(face_result.get("raw_name", "none")),
                            "raw_score": round(float(face_result.get("score", 0.0)), 4),
                            "top1_score": round(float(face_result.get("top1_score", 0.0)), 4),
                            "top3_mean_score": round(float(face_result.get("topk_score", 0.0)), 4),
                            "match_count": int(face_result.get("match_count_above_threshold", 0)),
                            "face_evidence_state": str(face_result.get("face_evidence_state", "none")),
                            "identity_transition_reason": str(getattr(tr, "identity_transition_reason", "")),
                            "smoothed_score_prev": round(prev_score, 4),
                            "smoothed_score": round(float(tr.score), 4),
                            "bbox": [int(v) for v in tr.bbox],
                        },
                    )
            face_ms = (time.time() - t_face) * 1000.0

        should_draw = (not args.no_show) or (writer is not None) or (stream_store is not None) or bool(args.video_stream_url)
        should_print = args.print_every > 0 and fid % args.print_every == 0
        out_rows = [] if should_print else None

        t_draw = time.time()
        for tr in tracks:
            x1, y1, x2, y2 = tr.bbox

            if tr.name == "unknown":
                if bool(getattr(tr, "matched_this_frame", False)):
                    tr.unknown_streak += 1
            else:
                tr.unknown_streak = 0
                tr.alarmed_once = False

            if (
                face_enabled
                and tr.name == "unknown"
                and bool(getattr(tr, "matched_this_frame", False))
                and tr.unknown_streak >= args.unknown_frames
                and float(tr.conf) >= float(args.unknown_min_conf)
                and has_recent_face_evidence(
                    tr,
                    fid,
                    args.unknown_face_evidence_max_gap,
                    min_score=args.unknown_face_evidence_score,
                    required_raw_name="unknown",
                )
                and (now - last_alarm_ts) >= args.alarm_cooldown
                and not tr.alarmed_once
            ):
                trigger_alarm(args.alarm_cmd)
                tr.alarmed_once = True
                last_alarm_ts = now
                append_jsonl(
                    args.event_log,
                    {
                        "ts": now,
                        "frame": int(fid),
                        "event": "unknown_alarm",
                        "track_id": int(tr.id),
                        "name": tr.name,
                        "score": round(float(tr.score), 4),
                        "unknown_streak": int(tr.unknown_streak),
                        "bbox": [int(v) for v in tr.bbox],
                    },
                )

            display_name = track_display_name(tr, args.unknown_label_frames, fid)
            label = f"id{tr.id}:{display_name}"
            if tr.score > 0:
                label += f" {tr.score:.2f}"

            if should_draw:
                color = track_color(tr)
                border = 3 if display_name == "unknown" else 2
                cv2.rectangle(frame, (x1, y1), (x2, y2), color, border)
                draw_label_box(frame, x1, y1, label, color)
            if should_print:
                out_rows.append(
                    {
                        "id": tr.id,
                        "name": tr.name,
                        "display_name": display_name,
                        "score": round(float(tr.score), 3),
                        "bbox": [int(x1), int(y1), int(x2), int(y2)],
                        "conf": round(float(tr.conf), 3),
                    }
                )
        draw_ms = (time.time() - t_draw) * 1000.0

        if should_print:
            print(
                json.dumps(
                    {
                        "frame": fid,
                        "infer_ms": round(float(last_infer_ms), 2),
                        "tracks": out_rows,
                    },
                    ensure_ascii=False,
                )
            )

        if args.profile_every > 0 and fid % args.profile_every == 0:
            print(
                json.dumps(
                    {
                        "frame": fid,
                        "profile": {
                            "detector_mode": args.detector_mode,
                            "preprocess_ms": round(float(preprocess_ms), 2),
                            "infer_ms": round(float(last_infer_ms), 2),
                            "rpc_ms": round(float(last_rpc_ms if args.detector_mode == "remote" else rpc_ms), 2),
                            "remote_encode_ms": round(
                                float(last_remote_encode_ms if args.detector_mode == "remote" else remote_encode_ms), 2
                            ),
                            "remote_server_ms": round(
                                float(last_remote_server_ms if args.detector_mode == "remote" else remote_server_ms), 2
                            ),
                            "remote_route": str(last_remote_route if args.detector_mode == "remote" else ""),
                            "face_ms": round(float(face_ms), 2),
                            "draw_ms": round(float(draw_ms), 2),
                            "loop_ms": round(float((time.time() - loop_t0) * 1000.0), 2),
                            "run_infer": bool(run_this_frame),
                            "tracks": len(tracks),
                        },
                    },
                    ensure_ascii=False,
                )
            )

        follow_target = choose_follow_target(
            tracks,
            args.follow_policy,
            args.follow_name,
            min_unknown_streak=max(1, args.follow_unknown_min_frames),
            min_unknown_conf=args.unknown_min_conf,
            preferred_track_id=last_follow_track_id,
            max_lost=args.follow_max_lost,
            frame_id=fid,
            require_unknown_face_evidence=False,
            unknown_face_max_gap=args.unknown_face_evidence_max_gap,
            unknown_face_min_score=args.unknown_face_evidence_score,
        )
        new_follow_id = int(follow_target.id) if follow_target is not None else None
        new_follow_name = str(follow_target.name) if follow_target is not None else ""
        if new_follow_id != last_follow_track_id or new_follow_name != last_follow_name:
            append_jsonl(
                args.event_log,
                {
                    "ts": time.time(),
                    "frame": int(fid),
                    "event": "follow_target",
                    "track_id": new_follow_id,
                    "name": new_follow_name,
                    "policy": args.follow_policy,
                    "follow_name": args.follow_name,
                },
            )
        render_target = choose_render_target(tracks, follow_target, preferred_track_id=last_follow_track_id)
        last_follow_track_id = int(follow_target.id) if follow_target is not None else None
        last_follow_name = new_follow_name
        follow_state_seq += 1
        write_follow_state(
            args.state_file,
            render_target,
            follow_target,
            fid,
            frame.shape,
            args.follow_name,
            follow_state_seq,
            unknown_face_max_gap=args.unknown_face_evidence_max_gap,
            unknown_face_min_score=args.unknown_face_evidence_score,
        )

        if should_draw:
            draw_runtime_hud(
                frame,
                args.detector_mode,
                stream_fps,
                loop_fps,
                float(last_infer_ms),
                float(last_rpc_ms if args.detector_mode == "remote" else rpc_ms),
                float(last_remote_server_ms if args.detector_mode == "remote" else remote_server_ms),
                tracks,
                follow_target,
                remote_route=last_remote_route if args.detector_mode == "remote" else "",
            )

        if writer is not None and fid % args.record_step == 0:
            writer.write(frame)

        if args.video_stream_url:
            if video_streamer is None:
                try:
                    video_streamer = FFmpegVideoStreamer(
                        args.video_stream_url,
                        frame.shape[1],
                        frame.shape[0],
                        fps=max(1.0, float(args.video_stream_fps)),
                        crf=int(args.video_stream_crf),
                    )
                    print(
                        f"[VIDEO] ffmpeg stream ready: {frame.shape[1]}x{frame.shape[0]} "
                        f"->{args.video_stream_url} (crf={args.video_stream_crf})"
                    )
                except Exception as e:
                    print(f"[VIDEO] failed to start ffmpeg stream: {e}")
                    args.video_stream_url = ""
            if video_streamer is not None:
                now_video = time.time()
                video_tolerance = min(0.01, video_min_gap * 0.20) if video_min_gap > 0 else 0.0
                if video_min_gap <= 0 or (now_video - last_video_push) >= max(0.0, video_min_gap - video_tolerance):
                    if video_streamer.write(frame):
                        last_video_push = now_video
                    else:
                        print("[VIDEO] write failed, restarting ffmpeg streamer on next frame")
                        video_streamer.close()
                        video_streamer = None

        if stream_store is not None:
            now_stream = time.time()
            stream_tolerance = min(0.02, stream_min_gap * 0.25) if stream_min_gap > 0 else 0.0
            if stream_min_gap <= 0 or (now_stream - last_stream_push) >= max(0.0, stream_min_gap - stream_tolerance):
                ok_jpg, enc = cv2.imencode(
                    ".jpg",
                    cv2.resize(
                        frame,
                        (
                            max(1, int(frame.shape[1] * args.stream_scale)),
                            max(1, int(frame.shape[0] * args.stream_scale)),
                        ),
                        interpolation=cv2.INTER_AREA,
                    )
                    if abs(args.stream_scale - 1.0) > 1e-6
                    else frame,
                    [int(cv2.IMWRITE_JPEG_QUALITY), int(args.stream_jpeg_quality)],
                )
                if ok_jpg:
                    if last_stream_emit_ts is not None:
                        stream_dt = max(1e-6, now_stream - last_stream_emit_ts)
                        instant_stream_fps = 1.0 / stream_dt
                        if stream_fps <= 0.0:
                            stream_fps = instant_stream_fps
                        else:
                            stream_fps = stream_fps * 0.85 + instant_stream_fps * 0.15
                    track_debug = [
                        {
                            "id": int(tr.id),
                            "name": str(tr.name),
                            "score": round(float(tr.score), 4),
                            "conf": round(float(tr.conf), 4),
                            "lost": int(tr.lost),
                            "matched_this_frame": bool(getattr(tr, "matched_this_frame", False)),
                            "detection_age_frames": int(max(0, fid - int(getattr(tr, "last_detection_frame", fid)))),
                            "bbox": [int(v) for v in tr.bbox],
                            "last_face_raw_name": str(getattr(tr, "last_face_raw_name", "none")),
                            "last_face_raw_score": round(float(getattr(tr, "last_face_raw_score", 0.0)), 4),
                            "last_face_top1_score": round(float(getattr(tr, "last_face_top1_score", 0.0)), 4),
                            "last_face_topk_score": round(float(getattr(tr, "last_face_topk_score", 0.0)), 4),
                            "last_face_match_count": int(getattr(tr, "last_face_match_count", 0)),
                            "face_evidence_state": str(getattr(tr, "face_evidence_state", "none")),
                            "identity_transition_reason": str(getattr(tr, "identity_transition_reason", "")),
                            "unknown_streak": int(getattr(tr, "unknown_streak", 0)),
                            "display_name": track_display_name(tr, args.unknown_label_frames, fid),
                            "provisional_name": str(getattr(tr, "provisional_name", "")),
                            "provisional_score": round(float(getattr(tr, "provisional_score", 0.0)), 4),
                        }
                        for tr in tracks
                    ]
                    stream_meta = make_stream_metadata(
                        args.detector_mode,
                        backend,
                        active_model,
                        active_device,
                        stream_fps,
                        loop_fps,
                        float(last_infer_ms),
                        float(last_rpc_ms if args.detector_mode == "remote" else rpc_ms),
                        float(last_remote_server_ms if args.detector_mode == "remote" else remote_server_ms),
                        tracks,
                        follow_target,
                        remote_route=last_remote_route if args.detector_mode == "remote" else "",
                        raw_det_count=last_raw_det_count,
                        filtered_det_count=last_filtered_det_count,
                        track_debug=track_debug,
                        db_build_version=db_build_version,
                        db_aggregate_mode=db_aggregate_mode,
                    )
                    stream_store.update(fid, enc.tobytes(), metadata=stream_meta)
                    last_stream_push = now_stream
                    last_stream_emit_ts = now_stream

        if not args.no_show:
            cv2.imshow("ascend_face_track_alarm", frame)
            key = cv2.waitKey(1) & 0xFF
            if key in (27, ord("q")):
                break

        last_loop_ts = loop_t0
        fid += 1

    cap.release()
    if writer is not None:
        writer.release()
    if video_streamer is not None:
        video_streamer.close()
    if stream_store is not None:
        stream_store.close()
    if stream_server is not None:
        stream_server.shutdown()
        stream_server.server_close()
    if async_remote is not None:
        async_remote.close()
    if remote_client is not None:
        remote_client.close()
    cv2.destroyAllWindows()
    follow_state_seq += 1
    write_follow_state(
        args.state_file,
        None,
        None,
        fid,
        (args.height, args.width, 3),
        args.follow_name,
        follow_state_seq,
        unknown_face_max_gap=args.unknown_face_evidence_max_gap,
        unknown_face_min_score=args.unknown_face_evidence_score,
    )
    if not args.no_record:
        ask_save_recording(tmp_video)


if __name__ == "__main__":
    main()
