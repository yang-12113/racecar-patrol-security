#!/usr/bin/env python3
import argparse
import json
import os
import tempfile
import threading
import time
from http import server
from socketserver import ThreadingMixIn

import cv2
import numpy as np

COCO_NAMES = [
    'person', 'bicycle', 'car', 'motorcycle', 'airplane', 'bus', 'train', 'truck', 'boat', 'traffic light',
    'fire hydrant', 'stop sign', 'parking meter', 'bench', 'bird', 'cat', 'dog', 'horse', 'sheep', 'cow',
    'elephant', 'bear', 'zebra', 'giraffe', 'backpack', 'umbrella', 'handbag', 'tie', 'suitcase', 'frisbee',
    'skis', 'snowboard', 'sports ball', 'kite', 'baseball bat', 'baseball glove', 'skateboard', 'surfboard',
    'tennis racket', 'bottle', 'wine glass', 'cup', 'fork', 'knife', 'spoon', 'bowl', 'banana', 'apple',
    'sandwich', 'orange', 'broccoli', 'carrot', 'hot dog', 'pizza', 'donut', 'cake', 'chair', 'couch',
    'potted plant', 'bed', 'dining table', 'toilet', 'tv', 'laptop', 'mouse', 'remote', 'keyboard', 'cell phone',
    'microwave', 'oven', 'toaster', 'sink', 'refrigerator', 'book', 'clock', 'vase', 'scissors', 'teddy bear',
    'hair drier', 'toothbrush'
]


def monotonic_ms():
    return int(time.monotonic_ns() // 1_000_000)


def atomic_write_json(path, data, prefix=".fallback_local_", suffix=".json"):
    if not path:
        return
    out_dir = os.path.dirname(path) or "."
    os.makedirs(out_dir, exist_ok=True)
    fd, tmp_path = tempfile.mkstemp(prefix=prefix, suffix=suffix, dir=out_dir)
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as f:
            json.dump(data, f, ensure_ascii=False, sort_keys=True)
            f.flush()
            os.fsync(f.fileno())
        os.replace(tmp_path, path)
    finally:
        if os.path.exists(tmp_path):
            try:
                os.remove(tmp_path)
            except OSError:
                pass


def select_control_target(detections, frame_shape):
    h, w = frame_shape[:2]
    best = None
    best_key = None
    for det in detections or []:
        if int(det.get("cls", -1)) != 0:
            continue
        x1, y1, x2, y2 = [int(v) for v in det.get("bbox", [0, 0, 0, 0])]
        bw = max(1, x2 - x1)
        bh = max(1, y2 - y1)
        area_ratio = float((bw * bh) / max(1.0, float(w * h)))
        conf = float(det.get("conf", 0.0))
        key = (area_ratio, conf)
        if best is None or key > best_key:
            best_key = key
            best = {
                "track_id": 1,
                "identity": "none",
                "face_evidence_state": "none",
                "confidence": conf,
                "width": int(w),
                "height": int(h),
                "cx_norm": float(((x1 + x2) * 0.5) / max(1.0, float(w))),
                "cy_norm": float(((y1 + y2) * 0.5) / max(1.0, float(h))),
                "distance_proxy": area_ratio,
                "bbox_xyxy_norm": [
                    float(x1 / max(1.0, float(w))),
                    float(y1 / max(1.0, float(h))),
                    float(x2 / max(1.0, float(w))),
                    float(y2 / max(1.0, float(h))),
                ],
            }
    return best



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


def load_backend(model, device_id):
    errs = []
    try:
        from ais_bench.infer.interface import InferSession
        sess = InferSession(device_id, model)
        return 'ais_bench', sess
    except Exception as e:
        errs.append(f'ais_bench: {e}')
    try:
        import aclruntime
        try:
            sess = aclruntime.InferenceSession(model, device_id)
        except TypeError:
            sess = aclruntime.InferenceSession(model, device_id, 0)
        return 'aclruntime', sess
    except Exception as e:
        errs.append(f'aclruntime: {e}')
    raise RuntimeError('No inference backend available. ' + ' | '.join(errs))


def run_infer(backend, sess, inp):
    if backend == 'ais_bench':
        return sess.infer([inp])
    trials = []
    if hasattr(sess, 'infer'):
        trials += [lambda: sess.infer([inp]), lambda: sess.infer(inp)]
    if hasattr(sess, 'run'):
        trials += [lambda: sess.run([inp]), lambda: sess.run(None, [inp]), lambda: sess.run([inp], None)]
    last = None
    for f in trials:
        try:
            return f()
        except Exception as e:
            last = e
    raise RuntimeError(f'aclruntime infer failed: {last}')


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
        raise RuntimeError('Cannot parse model output tensor shape.')
    return np.concatenate(mats, axis=0).astype(np.float32, copy=False)




def class_color(cls_id):
    palette = [
        (56, 56, 255), (151, 157, 255), (31, 112, 255), (29, 178, 255), (49, 210, 207),
        (10, 249, 72), (23, 204, 146), (134, 219, 61), (52, 147, 26), (187, 212, 0),
        (168, 153, 44), (255, 194, 0), (147, 69, 52), (255, 115, 100), (236, 24, 0),
        (255, 56, 132), (133, 0, 82), (255, 56, 203), (200, 149, 255), (199, 55, 255),
    ]
    idx = int(cls_id) % len(palette)
    return palette[idx]

def postprocess(pred, ratio, dwdh, orig_shape, conf_thres=0.25, iou_thres=0.45, target_cls=-1):
    h0, w0 = orig_shape[:2]
    dw, dh = dwdh
    if pred.shape[1] >= 7:
        boxes = pred[:, :4]
        obj = pred[:, 4]
        cls_prob = pred[:, 5:]
        cls_id = np.argmax(cls_prob, axis=1)
        cls_conf = cls_prob[np.arange(cls_prob.shape[0]), cls_id]
        scores = obj * cls_conf
        keep = scores >= conf_thres
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
        keep = scores >= conf_thres
        xyxy, scores, cls_id = xyxy[keep], scores[keep], cls_id[keep]
        if len(xyxy) == 0:
            return []
    xyxy[:, [0, 2]] = (xyxy[:, [0, 2]] - dw) / ratio
    xyxy[:, [1, 3]] = (xyxy[:, [1, 3]] - dh) / ratio
    xyxy[:, 0] = np.clip(xyxy[:, 0], 0, w0 - 1)
    xyxy[:, 1] = np.clip(xyxy[:, 1], 0, h0 - 1)
    xyxy[:, 2] = np.clip(xyxy[:, 2], 0, w0 - 1)
    xyxy[:, 3] = np.clip(xyxy[:, 3], 0, h0 - 1)
    nms_boxes = []
    for b in xyxy:
        x1, y1, x2, y2 = b.tolist()
        nms_boxes.append([x1, y1, max(0.0, x2 - x1), max(0.0, y2 - y1)])
    idxs = cv2.dnn.NMSBoxes(nms_boxes, scores.tolist(), conf_thres, iou_thres)
    if len(idxs) == 0:
        return []
    idxs = np.array(idxs).reshape(-1)
    out = []
    for i in idxs:
        x1, y1, x2, y2 = xyxy[i].tolist()
        cls = int(cls_id[i])
        if target_cls >= 0 and cls != target_cls:
            continue
        out.append({'cls': cls, 'conf': float(scores[i]), 'bbox': [int(x1), int(y1), int(x2), int(y2)]})
    return out




def open_camera(camera, width, height, fps=30.0, fourcc='MJPG'):
    candidates = []
    if isinstance(camera, int):
        candidates.append(int(camera))
    else:
        try:
            candidates.append(int(str(camera).strip()))
        except Exception:
            candidates.append(camera)
    for idx in [0, 1, 2, 3]:
        if idx not in candidates:
            candidates.append(idx)

    for cand in candidates:
        cap = None
        try:
            cap = cv2.VideoCapture(cand)
            if not cap.isOpened():
                cap.release()
                continue
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, int(width))
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, int(height))
            cap.set(cv2.CAP_PROP_FPS, float(fps))
            if fourcc:
                cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*str(fourcc)))
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            ok, frame = cap.read()
            if ok and frame is not None and frame.size > 0:
                print(f'[PURE-YOLO] camera selected: {cand} fourcc={fourcc} fps={fps}')
                return cap, cand
            cap.release()
        except Exception:
            if cap is not None:
                try:
                    cap.release()
                except Exception:
                    pass
    raise RuntimeError('Cannot open any camera from candidates: ' + ', '.join(map(str, candidates)))


def reopen_camera_with_retry(camera, width, height, fps=30.0, fourcc='MJPG', retry_sleep_ms=500):
    while True:
        try:
            return open_camera(camera, width, height, fps=fps, fourcc=fourcc)
        except Exception as exc:
            print(f'[PURE-YOLO] camera reopen failed: {exc}')
            time.sleep(max(0.05, float(retry_sleep_ms) / 1000.0))

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
    server_version = 'PureYoloV5MJPEG/1.0'

    def do_GET(self):
        if self.path in ('/', '/index.html'):
            body = (
                "<html><head><meta charset='utf-8'><title>Pure YOLOv5 Stream</title>"
                "<meta name='viewport' content='width=device-width, initial-scale=1'/>"
                "<style>body{margin:0;background:#101418;color:#eef2f7;font-family:Segoe UI,sans-serif;}"
                ".wrap{max-width:1180px;margin:0 auto;padding:18px;}"
                ".top{display:flex;justify-content:space-between;align-items:center;gap:12px;flex-wrap:wrap;margin-bottom:16px;}"
                ".title{font-size:28px;font-weight:700;}"
                ".sub{font-size:14px;opacity:.78;margin-top:6px;}"
                ".badge{padding:6px 10px;border-radius:999px;background:#1f2937;color:#8de1ff;font-size:12px;font-weight:700;}"
                ".grid{display:grid;grid-template-columns:2fr 1fr;gap:16px;}"
                ".card{background:rgba(255,255,255,.04);border:1px solid rgba(255,255,255,.08);border-radius:18px;overflow:hidden;}"
                ".card img{display:block;width:100%;height:auto;background:#000;}"
                ".links{display:flex;gap:10px;flex-wrap:wrap;padding:14px 16px;border-top:1px solid rgba(255,255,255,.08);}"
                ".links a{color:#d8f3ff;text-decoration:none;background:#243041;padding:8px 12px;border-radius:10px;font-size:13px;}"
                ".panel{padding:16px 18px;}"
                ".meta-row{display:flex;justify-content:space-between;gap:12px;font-size:14px;padding:10px 0;border-bottom:1px solid rgba(255,255,255,.06);}"
                ".meta-row:last-child{border-bottom:none;}"
                ".meta-key{opacity:.72;}.meta-val{font-weight:600;}"
                "@media (max-width:980px){.grid{grid-template-columns:1fr;}}"
                "</style></head><body><div class='wrap'>"
                "<div class='top'><div><div class='title'>Pure YOLOv5 Live Stream</div><div class='sub'>Only YOLOv5 OM detection and drawing, no face recognition, no patrol logic.</div></div><div class='badge'>LIVE</div></div>"
                "<div class='grid'><div class='card'><img src='/stream.mjpg' alt='pure yolo stream'/>"
                "<div class='links'><a href='/stream.mjpg' target='_blank'>Open Raw Stream</a><a href='/latest.jpg' target='_blank'>Open Latest Frame</a><a href='/status.json' target='_blank'>Open Status JSON</a></div></div>"
                "<div class='card panel'>"
                "<div class='meta-row'><div class='meta-key'>Model</div><div class='meta-val' id='model'>--</div></div>"
                "<div class='meta-row'><div class='meta-key'>Backend</div><div class='meta-val' id='backend'>--</div></div>"
                "<div class='meta-row'><div class='meta-key'>Stream FPS</div><div class='meta-val' id='stream_fps'>--</div></div>"
                "<div class='meta-row'><div class='meta-key'>Infer ms</div><div class='meta-val' id='infer_ms'>--</div></div>"
                "<div class='meta-row'><div class='meta-key'>Objects</div><div class='meta-val' id='objects'>--</div></div>"
                "<div class='meta-row'><div class='meta-key'>Frame</div><div class='meta-val' id='frame_id'>--</div></div>"
                "</div></div>"
                "<script>async function poll(){try{const r=await fetch('/status.json',{cache:'no-store'});const s=await r.json();"
                "const set=(id,v)=>{const el=document.getElementById(id);if(el)el.textContent=v;};"
                "set('model',s.model||'--');set('backend',s.backend||'--');"
                "set('stream_fps',s.stream_fps==null?'--':Number(s.stream_fps).toFixed(1));"
                "set('infer_ms',s.infer_ms==null?'--':Number(s.infer_ms).toFixed(1)+' ms');"
                "set('objects',s.objects==null?'--':s.objects);set('frame_id',s.frame_id==null?'--':s.frame_id);}catch(e){}}"
                "poll();setInterval(poll,1000);</script></body></html>"
            ).encode('utf-8')
            self.send_response(200)
            self.send_header('Content-Type', 'text/html; charset=utf-8')
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return
        if self.path == '/latest.jpg':
            _, jpeg, _, _ = self.server.frame_store.latest()
            if not jpeg:
                self.send_error(503, 'No frame available yet')
                return
            self.send_response(200)
            self.send_header('Content-Type', 'image/jpeg')
            self.send_header('Content-Length', str(len(jpeg)))
            self.send_header('Cache-Control', 'no-store, no-cache, must-revalidate, max-age=0')
            self.end_headers()
            self.wfile.write(jpeg)
            return
        if self.path == '/status.json':
            frame_id, _, closed, meta = self.server.frame_store.latest()
            body = json.dumps({'ok': bool(frame_id >= 0 and not closed), 'frame_id': int(frame_id), **meta}, ensure_ascii=False).encode('utf-8')
            self.send_response(200)
            self.send_header('Content-Type', 'application/json; charset=utf-8')
            self.send_header('Cache-Control', 'no-store, no-cache, must-revalidate, max-age=0')
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return
        if self.path not in ('/stream', '/stream.mjpg'):
            self.send_error(404)
            return
        self.send_response(200)
        self.send_header('Age', '0')
        self.send_header('Cache-Control', 'no-store, no-cache, must-revalidate, max-age=0')
        self.send_header('Pragma', 'no-cache')
        self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
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
                self.wfile.write(b'--frame\r\n')
                self.wfile.write(b'Content-Type: image/jpeg\r\n')
                self.wfile.write(f'Content-Length: {len(jpeg)}\r\n\r\n'.encode('ascii'))
                self.wfile.write(jpeg)
                self.wfile.write(b'\r\n')
        except (BrokenPipeError, ConnectionResetError):
            return

    def log_message(self, fmt, *args):
        return


def start_mjpeg_server(port, frame_store):
    httpd = ThreadedHTTPServer(('0.0.0.0', int(port)), MjpegHandler)
    httpd.frame_store = frame_store
    thread = threading.Thread(target=httpd.serve_forever, daemon=True)
    thread.start()
    return httpd


def main():
    ap = argparse.ArgumentParser(description='Pure YOLOv5 OM live stream.')
    ap.add_argument('--model', default='/root/yolov5s_310b.om')
    ap.add_argument('--device', type=int, default=0)
    ap.add_argument('--camera', type=int, default=0)
    ap.add_argument('--width', type=int, default=640)
    ap.add_argument('--height', type=int, default=480)
    ap.add_argument('--camera-fps', type=float, default=30.0)
    ap.add_argument('--camera-fourcc', default='MJPG')
    ap.add_argument('--imgsz', type=int, default=640)
    ap.add_argument('--conf', type=float, default=0.35)
    ap.add_argument('--iou', type=float, default=0.45)
    ap.add_argument('--target-cls', type=int, default=-1, help='-1 keeps all COCO classes, 0 keeps person only')
    ap.add_argument('--infer-interval', type=int, default=2)
    ap.add_argument('--print-every', type=int, default=10)
    ap.add_argument('--stream-port', type=int, default=8091)
    ap.add_argument('--stream-max-fps', type=float, default=12.0)
    ap.add_argument('--stream-jpeg-quality', type=int, default=85)
    ap.add_argument('--stream-scale', type=float, default=1.0)
    ap.add_argument('--camera-reopen-sleep-ms', type=int, default=500)
    ap.add_argument('--state-file', default='')
    ap.add_argument('--state-source', default='fallback_local_detector')
    args = ap.parse_args()
    args.infer_interval = max(1, int(args.infer_interval))
    args.stream_jpeg_quality = min(max(30, int(args.stream_jpeg_quality)), 95)
    args.stream_scale = min(1.0, max(0.25, float(args.stream_scale)))
    args.stream_max_fps = max(1.0, float(args.stream_max_fps))

    backend, sess = load_backend(args.model, args.device)
    print(f'[PURE-YOLO] backend={backend}, model={args.model}, infer_interval={args.infer_interval}, target_cls={args.target_cls}, requested_camera={args.camera}')

    cap, selected_camera = reopen_camera_with_retry(
        args.camera,
        args.width,
        args.height,
        args.camera_fps,
        args.camera_fourcc,
        retry_sleep_ms=args.camera_reopen_sleep_ms,
    )

    store = LatestFrameStore()
    httpd = None
    if int(args.stream_port) > 0:
        httpd = start_mjpeg_server(args.stream_port, store)
        print(f'[PURE-YOLO] stream ready: http://0.0.0.0:{args.stream_port}/')
    else:
        print('[PURE-YOLO] stream server disabled (stream_port<=0)')

    fid = 0
    last_dets = []
    last_infer_ms = 0.0
    last_stream_ts = 0.0
    min_gap = 1.0 / args.stream_max_fps
    last_emit_ts = None
    stream_fps = 0.0
    state_seq = 0

    try:
        while True:
            ok, frame = cap.read()
            if not ok or frame is None or frame.size == 0:
                print('[PURE-YOLO] camera read failed; reopening camera')
                try:
                    cap.release()
                except Exception:
                    pass
                cap, selected_camera = reopen_camera_with_retry(
                    args.camera,
                    args.width,
                    args.height,
                    args.camera_fps,
                    args.camera_fourcc,
                    retry_sleep_ms=args.camera_reopen_sleep_ms,
                )
                continue
            run_this_frame = (fid == 0) or (fid % args.infer_interval == 0)
            if run_this_frame:
                img, ratio, dwdh = letterbox(frame, (args.imgsz, args.imgsz))
                img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
                inp = np.transpose(img, (2, 0, 1))[None, ...]
                inp = np.ascontiguousarray(inp)
                t0 = time.time()
                raw = run_infer(backend, sess, inp)
                pred = normalize_output(raw)
                last_dets = postprocess(pred, ratio, dwdh, frame.shape, args.conf, args.iou, args.target_cls)
                last_infer_ms = (time.time() - t0) * 1000.0
            for d in last_dets:
                x1, y1, x2, y2 = d['bbox']
                name = COCO_NAMES[d['cls']] if 0 <= int(d['cls']) < len(COCO_NAMES) else str(d['cls'])
                txt = f"{name} {d['conf']:.2f}"
                color = class_color(d['cls'])
                cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
                label_y = max(0, y1 - 8)
                cv2.putText(frame, txt, (x1, label_y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2)
            cv2.putText(frame, f'PURE YOLOv5  infer {last_infer_ms:.1f} ms  objects {len(last_dets)}', (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (40, 220, 255), 2)

            if args.state_file:
                state_seq += 1
                selected = select_control_target(last_dets, frame.shape)
                state_payload = {
                    'source': str(args.state_source),
                    'schema_version': 1,
                    'seq': int(state_seq),
                    'frame_seq': int(fid),
                    'recv_monotonic_ms': int(monotonic_ms()),
                    'message_age_ms': 0,
                    'fresh': True,
                    'active': bool(selected),
                    'target_present': bool(selected),
                    'mode': 'FALLBACK_LOCAL',
                    'track_id': int(selected['track_id']) if selected else None,
                    'identity': str(selected['identity']) if selected else 'none',
                    'face_evidence_state': str(selected['face_evidence_state']) if selected else 'none',
                    'cx_norm': float(selected['cx_norm']) if selected else 0.5,
                    'cy_norm': float(selected['cy_norm']) if selected else 0.5,
                    'distance_proxy': float(selected['distance_proxy']) if selected else 0.0,
                    'confidence': float(selected['confidence']) if selected else 0.0,
                    'top1_score': 0.0,
                    'topk_score': 0.0,
                    'match_count': 0,
                    'width': int(frame.shape[1]),
                    'height': int(frame.shape[0]),
                    'bbox_xyxy_norm': selected['bbox_xyxy_norm'] if selected else [],
                }
                atomic_write_json(args.state_file, state_payload)

            now = time.time()
            if now - last_stream_ts >= min_gap:
                last_stream_ts = now
                view = frame
                if abs(args.stream_scale - 1.0) > 1e-6:
                    view = cv2.resize(frame, (max(1, int(frame.shape[1] * args.stream_scale)), max(1, int(frame.shape[0] * args.stream_scale))), interpolation=cv2.INTER_AREA)
                ok_jpg, enc = cv2.imencode('.jpg', view, [int(cv2.IMWRITE_JPEG_QUALITY), int(args.stream_jpeg_quality)])
                if ok_jpg:
                    if last_emit_ts is not None:
                        dt = max(1e-6, now - last_emit_ts)
                        stream_fps = 0.8 * stream_fps + 0.2 * (1.0 / dt) if stream_fps > 0 else (1.0 / dt)
                    last_emit_ts = now
                    store.update(fid, enc.tobytes(), metadata={'model': args.model, 'backend': backend, 'infer_ms': round(float(last_infer_ms), 2), 'objects': int(len(last_dets)), 'stream_fps': round(float(stream_fps), 2), 'camera': int(selected_camera)})
            if args.print_every > 0 and fid % args.print_every == 0:
                print(json.dumps({'frame': fid, 'infer_ms': round(float(last_infer_ms), 2), 'bbox': last_dets}, ensure_ascii=False))
            fid += 1
    finally:
        try:
            cap.release()
        except Exception:
            pass
        store.close()
        if httpd is not None:
            httpd.shutdown()
            httpd.server_close()


if __name__ == '__main__':
    main()


