# Vision Package Performance Analysis – Jetson Orin Nano

**Context**: Standalone YOLO detection reaches ~30 FPS on Jetson Orin Nano (PyTorch 2.10.0). The full vision system (camera_manager + detector_node + alignment_controller) now runs at **20–25 FPS** (was ~5 FPS before Phase 1 & 2 optimizations).

---

## 1. Pipeline Architecture

### Current (post Phase 1 & 2)

```
[camera_manager]  Timer @ 30 Hz
       │
       ▼  read_frame() → cv2_to_ros_image() → publish Image
       │
[detector_node]   subscribe /camera/forward/image_raw
       │
       ├─ ros_image_to_cv2()           ← copy 1 (every frame)
       ├─ model(frame, ...)             ← YOLO inference (~33 ms at 30 fps)
       ├─ publish DetectionArray       ← full rate
       │
       ├─ [when should_annotate]       ← rate-limited: display @ display_rate, publish @ annotated_publish_rate
       │  ├─ _annotate_frame()         ← frame.copy() + reused SV annotators
       │  ├─ _draw_alignment_overlay()
       │  ├─ _draw_kalman_overlay()    ← reused KF annotators (no per-frame alloc)
       │  ├─ cv2_to_ros_image()        ← only when publishing
       │  ├─ publish Image (annotated) ← ~10 Hz default
       │  └─ _display_frame = copy     → [async display thread @ 20 Hz] imshow, waitKey
       │
[alignment_controller]  Timer @ 10 Hz
       │
       └─ subscribe /vision/detections, /driver/command
          publish /vision/alignment_status, /driver/command
```

---

## 2. Bottleneck Analysis

### 2.1 Bottleneck Status (post Phase 1 & 2)

| Component | Status | Notes |
|-----------|--------|-------|
| **Kalman overlay** | ✅ Fixed | Reused annotators; no per-frame alloc |
| **Terminal logging** | ✅ Fixed | Throttled to log_rate (2 Hz default) |
| **Annotated Image publish** | ✅ Fixed | Rate-limited (10 Hz default) |
| **Display (cv2.imshow)** | ✅ Fixed | Moved to async thread; doesn’t block callback |
| **Annotation for display** | ✅ Fixed | Only at display_rate (20 Hz) when enable_display |
| **ROS Image serialization** | ⚠️ Remaining | `tobytes()`/`frombuffer()` ~2–5 ms each; only when publishing (10 Hz) |
| **ros_image_to_cv2** | ⚠️ Remaining | Copy on every frame; ~2–5 ms |
| **frame.copy()** | ⚠️ Remaining | In `_annotate_frame()` when we annotate |
| **Alignment overlay** | ⚠️ Minor | ~15 cv2 calls when annotating |
| **sv.Detections.from_ultralytics** | ⚠️ Minor | Per-frame when annotating |

### 2.2 Architectural Bottlenecks

| Issue | Status |
|-------|--------|
| **Single-threaded callback** | Still serial; display decoupled. Further gains need image conversion or executor changes. |
| **BEST_EFFORT + depth=1** | Correct; keeps latency low. |
| **Sync processing** | Annotate/publish/display now rate-limited; inference runs at full rate. |
| **Camera vs detector** | Now in sync (~25 fps); camera @ 30 Hz, detector keeps up. |

### 2.3 Why Standalone is Faster

- **Direct VideoCapture**: No ROS, no serialization.
- **No AlignmentStatus subscription**: No Kalman overlay.
- **Simpler overlay**: Only boxes/labels, no alignment arrows/dead zone/KF.
- **Single process**: No DDS/IPC between camera and detector.
- **Possible difference**: Standalone may have been tested with `enable_display=false` or different resolution.

---

## 3. Theory: Optimisation Principles

### 3.1 Reduce Allocations

- Reuse annotators (create once in `__init__`).
- Avoid per-frame `frame.copy()` where drawing can be in-place or on a reused buffer.
- Use `np.asarray()` or shared-memory patterns instead of `tobytes()`/`frombuffer()` where possible.

### 3.2 Decouple Heavy Operations

- **Display**: Move `imshow`/`waitKey` to a separate thread or lower rate (e.g. 10 Hz).
- **Publish annotated**: Make optional and/or rate-limited (e.g. 10 Hz for telemetry).
- **Detection**: Keep at full rate; annotation/publish/display can run at reduced rate.

### 3.3 Jetson-Specific

- Use `cv2.CAP_V4L2` for capture.
- Prefer `cv2.imshow` with `WINDOW_NORMAL`; consider headless mode (`enable_display=false`) for max FPS.
- Ensure YOLO uses GPU: `device='cuda:0'` or `auto`.
- Consider TensorRT export for Ultralytics on Jetson for extra inference speed.

### 3.4 ROS 2 Tuning

- QoS: `BEST_EFFORT` + `depth=1` is correct to avoid queuing; ensure no `RELIABLE` on hot paths.
- Consider `IntraProcessManager` for zero-copy if nodes run in same process (advanced).

---

## 4. Implementation Plan

### Phase 1: Quick Wins (Target: 15–20 FPS)

1. **Reuse Kalman annotators**  
   - Move `BoxAnnotator` and `LabelAnnotator` creation out of `_draw_kalman_overlay()` into `__init__`.  
   - Store as `self._kf_box_annotator`, `self._kf_label_annotator`.

2. **Throttle terminal logging**  
   - Replace per-detection `info()` with rate-limited logging (e.g. 2 Hz).

3. **Avoid redundant annotation when headless**  
   - If `enable_display=false` and `publish_annotated=false`, skip annotation and overlays.

4. **Optional: skip alignment/Kalman overlay when alignment inactive**  
   - Only draw when `_last_alignment` is valid and alignment is active.

5. **Jetson-friendly launch defaults**  
   - Default `enable_display:=false` for production; add `enable_display:=true` for debugging.

### Phase 2: Decouple Display (Target: 20–25 FPS) ✅ DONE

6. **Async display** ✅  
   - Run `imshow`/`waitKey` in a dedicated thread with a shared latest frame (lock-protected).  
   - Main callback: infer, annotate (rate-limited for display), publish DetectionArray, push frame to display buffer.  
   - Display thread: runs at `display_rate` (default 20 Hz), `imshow`, `waitKey(1)`.

7. **Rate-limit annotated publish** ✅ (Phase 1)  
   - `annotated_publish_rate` param (default 10 Hz); only publish annotated Image at that rate.  
   - DetectionArray remains at full rate.

### Phase 3: Reduce Copy/Serialisation (Target: 25–30 FPS) ✅ DONE

8. **Lazy annotated publish** ✅  
   - Only build and publish annotated Image when `annotated_pub.get_subscription_count() > 0`.

9. **Skip overlays when alignment inactive** ✅  
   - Alignment overlay: only draw when `_last_alignment` is not None.  
   - Kalman overlay: already skips when no status/target (in `_draw_kalman_overlay`).

10. **Optimise ros_image_to_cv2 / cv2_to_ros_image**  
    - Profile; consider `np.asarray` + view where encoding allows, to minimise copies.

### Phase 4: Advanced (if needed)

11. **TensorRT / ONNX**  
    - Export YOLO to TensorRT for Jetson.  
    - Requires separate model export and loader.

12. **Multi-threaded executor**  
    - Use `MultiThreadedExecutor` so detection callback and other nodes do not block each other.  
    - Must ensure thread-safe access to shared state (e.g. `_last_alignment`).

13. **Shared memory transport**  
    - Use ROS 2 shared-memory for Image transport where supported.

---

## 5. Recommended Order

| Step | Change | Effort | Impact |
|------|--------|--------|--------|
| 1 | Reuse KF annotators | Low | Medium (5–10 ms saved) |
| 2 | Throttle logging | Low | Low–Medium |
| 3 | Rate-limit annotated publish (10 Hz) | Low | Medium |
| 4 | Default enable_display=false on Jetson | Trivial | High if display is slow |
| 5 | Async display thread | Medium | High |
| 6 | Lazy annotated publish | Low | Medium |
| 7 | Skip overlays when alignment inactive | Low | Low |

---

## 6. Validation

After changes:

1. Run `ros2 launch vision vision.launch.py enable_display:=false` and measure FPS from detector logs.
2. Compare against `ros2 run vision detector_standalone` for baseline.
3. Run with `enable_display:=true` to confirm display does not regress behaviour.
4. Test alignment controller with different `control_rate` to ensure no functional regressions.

---

## 8. Current Results & Next Steps

### Achieved (Phase 1 & 2)

| Metric | Before | After |
|--------|--------|-------|
| **FPS** | ~5 | **20–25** |
| **Improvement** | — | ~4–5× |

### Phase 3 ✅ DONE

| # | Change | Status |
|---|--------|--------|
| 1 | **Lazy annotated publish** | ✅ Only build/publish when `get_subscription_count() > 0` |
| 2 | **Skip overlays when alignment inactive** | ✅ Alignment overlay only when `_last_alignment` not None |
| 3 | **Optimise ros_image_to_cv2** | Deferred (profile first; needs validation) |

### Phase 4 (advanced, if needed)

- TensorRT export for YOLO on Jetson.
- MultiThreadedExecutor (with thread-safety).
- Shared-memory transport for Image topics.

---

**Awaiting approval** before implementing Phase 3 changes.
