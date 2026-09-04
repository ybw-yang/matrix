#!/usr/bin/env python3
"""Verify that lidar / IMU header stamps are synchronized with the odometry stamps.

Four checks, in order of how much they prove. A stamp can look perfectly
self-consistent and still describe the wrong instant, so the cheap checks bound
the problem and the last one settles it.

1. STAMP LEVEL -- always available
   Record (header.stamp, wall_recv_time) per message; age = recv - stamp is the
   publish+transport delay against the SAME wall clock for all topics, so a
   difference in mean age between topics is a relative stamp bias.
   CALIBRATE before trusting a large-message age: a cross-process loopback of an
   equally sized (0.52 MB) PointCloud2 under this repo's cyclonedds.xml
   (FragmentSize 60000B) costs only ~1.4 ms, so anything past that in the
   lidar's age is publisher-side, not transport.

2. STAMP SOURCE -- always available
   Is the stamp drawn from the simulator's clock, or from a free-running timer
   in the publisher's own thread? A stamp on the sim grid sits ~0 from the
   nearest odom stamp; a floating one is uniformly phased (sd -> tick/sqrt(12)).
   This is stronger evidence than the raw age: a floating stamp is not tied to
   the simulated state at all.

3. LIDAR per-point `timestamp` -- always available
   Reveals scan duration and whether header.stamp is scan START or END (a whole
   scan period of systematic error if the consumer guesses wrong), and whether
   there is any intra-scan timing to deskew with.

4. CONTENT LEVEL -- the real proof, REQUIRES MOTION
   Stamps 1-3 only check bookkeeping. Here we compare what the sensors actually
   measured. IMU angular velocity and odom twist observe the same rotation, so
   the lag maximizing their cross-correlation is the true stamp offset, with no
   transport assumption. For the lidar we recover its OWN yaw rate from cloud
   content -- the azimuth/range profile of a static scene shifts by the yaw
   increment between scans -- and correlate that against odom. This is the only
   check that can catch render/pipeline lag, where the stamp is fine but the
   content is stale.

Usage:
  python3 scripts/stamp_sync_probe.py                 # waits for motion, then 25 s
  python3 scripts/stamp_sync_probe.py --duration 40
  python3 scripts/stamp_sync_probe.py --no-wait       # start collecting at once
  python3 scripts/stamp_sync_probe.py --imu /front_lidar/imu

For check 4 drive the robot: IN-PLACE ROTATION is best (translation distorts the
lidar profile), and include several direction reversals -- a slow constant spin
puts no energy at high frequency and the correlation peak goes flat, which the
script reports rather than turning into a confident wrong number.
"""
import argparse
import sys
import time

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, PointCloud2
from nav_msgs.msg import Odometry

NBINS = 720          # 0.5 deg azimuth bins for the lidar yaw estimator
R_MIN, R_MAX = 0.5, 60.0
EL_MAX = 25.0        # keep the near-horizontal band: it carries the yaw signal


def stamp_s(hdr):
    return hdr.stamp.sec + hdr.stamp.nanosec * 1e-9


def azimuth_profile(xyz):
    """Mean range per azimuth bin, over the near-horizontal band. For a static
    scene this profile is a signature of the surroundings that translates
    (circularly) by exactly the yaw increment when the sensor rotates."""
    r = np.linalg.norm(xyz, axis=1)
    ok = np.isfinite(r) & (r > R_MIN) & (r < R_MAX)
    if ok.sum() < 500:
        return None
    xyz, r = xyz[ok], r[ok]
    el = np.degrees(np.arcsin(np.clip(xyz[:, 2] / r, -1.0, 1.0)))
    m = np.abs(el) < EL_MAX
    if m.sum() < 500:
        return None
    xyz, r = xyz[m], r[m]
    az = np.arctan2(xyz[:, 1], xyz[:, 0])
    idx = np.clip(((az + np.pi) / (2 * np.pi) * NBINS).astype(int), 0, NBINS - 1)
    s = np.bincount(idx, weights=r, minlength=NBINS)
    c = np.bincount(idx, minlength=NBINS)
    prof = np.where(c > 0, s / np.maximum(c, 1), np.nan)
    if np.isnan(prof).sum() > NBINS // 4:
        return None
    bad = np.isnan(prof)
    if bad.any():   # circular fill so the FFT sees no discontinuity
        good = ~bad
        prof[bad] = np.interp(np.flatnonzero(bad), np.flatnonzero(good), prof[good], period=NBINS)
    return prof


def circular_shift(a, b):
    """Signed circular shift (in bins, sub-bin refined) that best aligns b onto a."""
    a = a - a.mean()
    b = b - b.mean()
    if a.std() < 1e-6 or b.std() < 1e-6:
        return None
    c = np.fft.irfft(np.fft.rfft(a) * np.conj(np.fft.rfft(b)), n=NBINS)
    k = int(np.argmax(c))
    y0, y1, y2 = c[(k - 1) % NBINS], c[k], c[(k + 1) % NBINS]
    den = y0 - 2 * y1 + y2
    frac = 0.5 * (y0 - y2) / den if abs(den) > 1e-12 else 0.0
    sh = k + frac
    return sh - NBINS if sh > NBINS / 2 else sh


class Probe(Node):
    def __init__(self, imu_topic, lidar_topic, odom_topic, want_lidar_yaw):
        super().__init__('stamp_sync_probe')
        self.imu = []      # (stamp, recv, wx, wy, wz)
        self.odom = []     # (stamp, recv, wx, wy, wz, vx, vy, vz)
        self.lidar = []    # (stamp, recv, npts, pt_min, pt_max)
        self.prof = []     # (stamp, profile)
        self.want_lidar_yaw = want_lidar_yaw
        self.collecting = True
        self.create_subscription(Imu, imu_topic, self._imu_cb, qos_profile_sensor_data)
        self.create_subscription(Odometry, odom_topic, self._odom_cb, qos_profile_sensor_data)
        self.create_subscription(PointCloud2, lidar_topic, self._lidar_cb, qos_profile_sensor_data)

    def _imu_cb(self, m):
        if not self.collecting:
            return
        w = m.angular_velocity
        self.imu.append((stamp_s(m.header), time.time(), w.x, w.y, w.z))

    def _odom_cb(self, m):
        w, v = m.twist.twist.angular, m.twist.twist.linear
        self.odom.append((stamp_s(m.header), time.time(), w.x, w.y, w.z, v.x, v.y, v.z))

    def _lidar_cb(self, m):
        if not self.collecting:
            return
        recv = time.time()
        st = stamp_s(m.header)
        npts = m.width * m.height
        buf = np.frombuffer(m.data, dtype=np.uint8)[: npts * m.point_step].reshape(npts, m.point_step)
        ts_min = ts_max = float('nan')
        off = next((f.offset for f in m.fields if f.name == 'timestamp'), None)
        if off is not None and npts:
            ts = np.ascontiguousarray(buf[:, off:off + 8]).view(np.float64).ravel()
            ts = ts[np.isfinite(ts)]
            if ts.size:
                ts_min, ts_max = float(ts.min()), float(ts.max())
        self.lidar.append((st, recv, npts, ts_min, ts_max))
        if self.want_lidar_yaw and npts:
            xyz = np.ascontiguousarray(buf[:, 0:12]).view(np.float32).reshape(-1, 3).astype(np.float64)
            p = azimuth_profile(xyz)
            if p is not None:
                self.prof.append((st, p))

    def recent_omega(self, n=100):
        if len(self.odom) < n:
            return 0.0
        a = np.array([r[2:5] for r in self.odom[-n:]], dtype=float)
        return float(np.abs(a[:, 2]).max())


def stats(name, rows):
    if len(rows) < 3:
        print(f"  {name:22s} NO DATA ({len(rows)} msgs)")
        return None
    a = np.array([r[:2] for r in rows], dtype=float)
    st, rc = a[:, 0], a[:, 1]
    age = rc - st
    dt = np.diff(st)
    mean_rate = (len(rows) - 1) / (st[-1] - st[0])
    med_rate = 1.0 / np.median(dt)
    note = ""
    if abs(mean_rate - med_rate) / mean_rate > 0.02:
        # Bimodal intervals: the stream is published at mean_rate but its stamps
        # are QUANTIZED onto a coarser clock, so intervals take only 1x or 2x the
        # quantum and the median lands on the common one. Reporting the median as
        # "the rate" invents a rate the publisher never had (measured: median said
        # 470.6 Hz for a stream that is really 500.0 Hz on a ~1.075 ms grid).
        note = f"  <-- median says {med_rate:.2f} Hz: stamps are QUANTIZED, not {med_rate:.0f} Hz"
    print(f"  {name:22s} n={len(rows):5d}  rate={mean_rate:7.2f} Hz (mean){note}")
    print(f"  {'':22s} age(recv-stamp)  mean={age.mean()*1e3:8.2f} ms  sd={age.std()*1e3:6.2f}  "
          f"min={age.min()*1e3:8.2f}  max={age.max()*1e3:8.2f}")
    print(f"  {'':22s} stamp period     med={np.median(dt)*1e3:8.3f} ms  "
          f"jitter(sd)={dt.std()*1e3:6.3f} ms  non-monotonic={int((dt <= 0).sum())}")
    return float(np.median(age))


def grid_alignment(name, rows, odom_rows, _tick=None):
    """Are this topic's stamps drawn from the same clock as odom's?

    Earlier versions compared the scatter of the nearest-odom-stamp difference to
    tick/sqrt(12), with `tick` taken as the median odom interval. That is unsafe
    twice over: the median interval is not the period when stamps are quantized,
    and the resulting ratio flipped between "47.06 ticks (independent)" and
    "46.99 ticks (integer multiple)" across runs purely from the tick estimate
    wobbling. So compare against the real null instead: draw times uniformly over
    the window and measure THEIR distance to the nearest odom stamp. That null
    needs no model of the grid, and it automatically accounts for the odom
    stream's own irregular spacing."""
    if len(rows) < 5 or len(odom_rows) < 5:
        return
    X = np.array([r[0] for r in rows], dtype=float)
    O = np.array([r[0] for r in odom_rows], dtype=float)

    def nearest(T):
        i = np.clip(np.searchsorted(O, T), 1, O.size - 1)
        return np.abs(np.where(np.abs(T - O[i - 1]) < np.abs(T - O[i]), T - O[i - 1], T - O[i]))

    rng = np.random.default_rng(0)
    null = nearest(rng.uniform(O[0], O[-1], 100000))
    a = nearest(X)
    tol = 50e-6
    hit, hit_null = (a < tol).mean(), (null < tol).mean()
    ratio = a.mean() / max(null.mean(), 1e-12)
    if ratio < 0.1:
        verdict = "LOCKED to the odom clock"
    elif ratio < 0.7:
        verdict = f"ASSOCIATED with the odom clock ({hit/max(hit_null,1e-9):.1f}x enriched near a stamp)"
    else:
        verdict = "INDEPENDENT of the odom clock (indistinguishable from random)"
    print(f"  {name:22s} |diff| to nearest odom stamp: mean={a.mean()*1e6:7.1f} us "
          f"(null {null.mean()*1e6:.1f} us, ratio {ratio:.2f})")
    print(f"  {'':22s} within 50 us of an odom stamp: {hit*100:5.1f}% (null {hit_null*100:.1f}%)")
    print(f"  {'':22s} => {verdict}")


def best_lag(t_a, v_a, t_b, v_b, fs=1000.0, max_lag=0.4):
    """Lag L maximizing corr(a(t), b(t+L)). Sign convention, verified by injecting
    a known delay (see the calibration in the commit message): L > 0 means b's
    feature lands LATER on the time axis than a's for the same physical event,
    i.e. b's header.stamp is L too late -- the content b carries is L OLD.
    Returns (lag, peak, err)."""
    t0, t1 = max(t_a[0], t_b[0]), min(t_a[-1], t_b[-1])
    if t1 - t0 < 2.0:
        return None, 0.0, "overlap < 2 s"
    grid = np.arange(t0, t1, 1.0 / fs)
    A = np.interp(grid, t_a, v_a)
    B = np.interp(grid, t_b, v_b)
    A, B = A - A.mean(), B - B.mean()
    if A.std() < 1e-4 or B.std() < 1e-4:
        return None, 0.0, "signal is flat -- robot not moving"
    A /= A.std()
    B /= B.std()
    K = int(max_lag * fs)
    lags = np.arange(-K, K + 1)
    c = np.empty(lags.size)
    for i, k in enumerate(lags):
        x, y = (A[: len(A) - k], B[k:]) if k >= 0 else (A[-k:], B[: len(B) + k])
        c[i] = float(np.dot(x, y)) / max(len(x), 1)
    j = int(np.argmax(c))
    peak, lag = c[j], lags[j] / fs
    if 0 < j < len(c) - 1:
        y0, y1, y2 = c[j - 1], c[j], c[j + 1]
        den = y0 - 2 * y1 + y2
        if abs(den) > 1e-12:
            lag += (0.5 * (y0 - y2) / den) / fs
    return float(lag), float(peak), ""


def report_lag(label, t_a, v_a, t_b, v_b, try_sign_flip=False, fs=1000.0):
    lag, peak, err = best_lag(t_a, v_a, t_b, v_b, fs=fs)
    if try_sign_flip:
        lag2, peak2, err2 = best_lag(t_a, v_a, t_b, -np.asarray(v_b), fs=fs)
        if lag2 is not None and (lag is None or peak2 > peak):
            lag, peak, err = lag2, peak2, err2
            print(f"  {'':30s} (sign-flipped series matched better: b's axis is inverted vs a's)")
    if lag is None:
        print(f"  {label:30s} -- {err}")
        return None
    who = (f"stamp is {lag*1e3:.0f} ms LATE (content that old)" if lag > 0
           else f"stamp is {-lag*1e3:.0f} ms EARLY")
    verdict = "in sync" if abs(lag) < 0.003 else who
    flag = "" if peak >= 0.5 else "   <-- weak corr, unreliable"
    print(f"  {label:30s} lag={lag*1e3:+8.2f} ms  corr={peak:7.4f}  {verdict}{flag}")
    return lag


class Collected:
    """Same four lists a live Probe fills, but sourced from a bag. Keeps the
    analysis below identical for live and offline input."""
    def __init__(self):
        self.imu, self.odom, self.lidar, self.prof = [], [], [], []

    def destroy_node(self):
        pass


def load_bag(path, imu_topic, lidar_topic, odom_topic):
    """Read a rosbag2 (sqlite3) recording into the same shape the live probe uses.

    The bag's per-message timestamp is rosbag2's RECEIVE time, which is what the
    `age` metric needs. It is in fact cleaner than the live probe's: rosbag2
    stamps in C++ at the subscription, where the Python probe pays callback
    overhead first (measured: 0.06 ms vs 0.23 ms on the same imu topic), so
    offline ages are slightly TIGHTER than live ones, never looser."""
    from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    out = Collected()
    r = SequentialReader()
    r.open(StorageOptions(uri=path, storage_id='sqlite3'), ConverterOptions('cdr', 'cdr'))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    wanted = {imu_topic, lidar_topic, odom_topic}
    missing = wanted - set(types)
    if missing:
        print(f"  WARNING: bag has no {', '.join(sorted(missing))} "
              f"(present: {', '.join(sorted(types))})")
    n = 0
    while r.has_next():
        topic, data, t_recv = r.read_next()
        if topic not in wanted:
            continue
        m = deserialize_message(data, get_message(types[topic]))
        st, recv = stamp_s(m.header), t_recv * 1e-9
        if topic == imu_topic:
            w = m.angular_velocity
            out.imu.append((st, recv, w.x, w.y, w.z))
        elif topic == odom_topic:
            w, v = m.twist.twist.angular, m.twist.twist.linear
            out.odom.append((st, recv, w.x, w.y, w.z, v.x, v.y, v.z))
        elif topic == lidar_topic:
            npts = m.width * m.height
            buf = np.frombuffer(m.data, dtype=np.uint8)[: npts * m.point_step].reshape(npts, m.point_step)
            ts_min = ts_max = float('nan')
            off = next((f.offset for f in m.fields if f.name == 'timestamp'), None)
            if off is not None and npts:
                ts = np.ascontiguousarray(buf[:, off:off + 8]).view(np.float64).ravel()
                ts = ts[np.isfinite(ts)]
                if ts.size:
                    ts_min, ts_max = float(ts.min()), float(ts.max())
            out.lidar.append((st, recv, npts, ts_min, ts_max))
            if npts:
                xyz = np.ascontiguousarray(buf[:, 0:12]).view(np.float32).reshape(-1, 3).astype(np.float64)
                pr = azimuth_profile(xyz)
                if pr is not None:
                    out.prof.append((st, pr))
        n += 1
        if n % 20000 == 0:
            print(f"    ... {n} messages", flush=True)
    for lst in (out.imu, out.odom, out.lidar, out.prof):
        lst.sort(key=lambda q: q[0])
    return out


def run_analysis(node, args):
    print("\n=== 1. STAMP LEVEL (same wall clock for all topics) ===")
    a_imu = stats(args.imu, node.imu)
    a_lid = stats(args.lidar, node.lidar)
    a_odo = stats(args.odom, node.odom)
    if a_odo is not None:
        print("\n  relative stamp bias vs odom (median age difference;"
              " ~1.4 ms of the lidar's is real transport):")
        for nm, av in ((args.imu, a_imu), (args.lidar, a_lid)):
            if av is not None:
                d = (av - a_odo) * 1e3
                verdict = "OK" if abs(d) < 5 else ("SUSPECT" if abs(d) < 20 else "OUT OF SYNC")
                print(f"    {nm:22s} {d:+8.2f} ms  "
                      f"{'stamped earlier than odom' if d > 0 else 'ahead':28s} [{verdict}]")

    if len(node.odom) > 5:
        ost = np.array([r[0] for r in node.odom])
        odt = np.diff(ost)
        print(f"\n=== 2. STAMP SOURCE (odom: {(ost.size-1)/(ost[-1]-ost[0]):.3f} Hz mean, "
              f"intervals p05={np.percentile(odt,5)*1e6:.0f} / p50={np.median(odt)*1e6:.0f} / "
              f"p95={np.percentile(odt,95)*1e6:.0f} us) ===")
        grid_alignment(args.imu, node.imu, node.odom)
        grid_alignment(args.lidar, node.lidar, node.odom)

    print("\n=== 3. LIDAR per-point timestamp vs header.stamp ===")
    lid = [r for r in node.lidar if np.isfinite(r[3])]
    if not lid:
        print("  no usable per-point timestamp field")
    else:
        arr = np.array([(r[0], r[3], r[4]) for r in lid], dtype=float)
        if not np.any(arr[:, 1]) and not np.any(arr[:, 2]):
            print("  per-point `timestamp` field is present but ALL ZERO in every scan.")
            print("  => the publisher never populates it, so a downstream LIO cannot deskew and")
            print("     must treat the whole cloud as one instant at header.stamp.")
            print("     In THIS sim that is roughly valid: measuring the azimuth profile's")
            print("     fine-scale (<10 deg) energy against yaw rate showed only a ~20% drop at")
            print("     14 deg of rotation per scan period, where a true 100 ms sweep would have")
            print("     destroyed ~95% of it -- i.e. the cloud is close to an instantaneous")
            print("     snapshot, not a sweep. Do NOT carry that assumption to real Livox HW.")
        else:
            ns = arr[:, 1].mean() > 1e17
            absolute = arr[:, 1].mean() > 1e8
            if ns:
                first, last, unit = arr[:, 1] * 1e-9, arr[:, 2] * 1e-9, "ns-epoch"
            elif absolute:
                first, last, unit = arr[:, 1], arr[:, 2], "s-epoch"
            else:
                first, last, unit = arr[:, 0] + arr[:, 1], arr[:, 0] + arr[:, 2], "relative to header"
            print(f"  point stamps are {unit}; scan span mean={np.mean(last-first)*1e3:.3f} ms")
            print(f"  header - first_point = {np.mean(arr[:,0]-first)*1e3:+8.3f} ms")
            print(f"  header - last_point  = {np.mean(arr[:,0]-last)*1e3:+8.3f} ms")
            conv = "START" if abs(np.mean(arr[:, 0] - first)) < abs(np.mean(arr[:, 0] - last)) else "END"
            print(f"  => header.stamp corresponds to {conv} of scan")

    print("\n=== 4. CONTENT LEVEL (needs motion) ===")
    if len(node.imu) < 50 or len(node.odom) < 50:
        print("  not enough IMU/odom samples")
        return

    I = np.array(node.imu, dtype=float)
    O = np.array(node.odom, dtype=float)
    om = np.abs(O[:, 4])
    moving = om > args.omega_min
    frac = moving.mean()
    print(f"  odom |omega_z|: max={om.max():.3f} rad/s  moving {frac*100:.0f}% of samples "
          f"(gate {args.omega_min})")
    if om.max() < args.omega_min:
        print("  ROBOT DID NOT ROTATE -- content-level checks cannot run.")
        return
    t0, t1 = O[moving, 0].min(), O[moving, 0].max()
    print(f"  using the moving window {t1-t0:.1f} s\n")

    def clip(A):
        m = (A[:, 0] >= t0) & (A[:, 0] <= t1)
        return A[m]
    Ic, Oc = clip(I), clip(O)

    print("  IMU vs odom:")
    report_lag("|omega| (frame-invariant)", Oc[:, 0], np.linalg.norm(Oc[:, 2:5], axis=1),
               Ic[:, 0], np.linalg.norm(Ic[:, 2:5], axis=1))
    for k, ax in enumerate('xyz'):
        report_lag(f"omega_{ax}", Oc[:, 0], Oc[:, 2 + k], Ic[:, 0], Ic[:, 2 + k], try_sign_flip=True)

    print("\n  LIDAR vs odom (yaw rate recovered from cloud content):")
    wz = Oc[:, 4]
    # A cross-correlation can only locate a lag if the signal has FEATURES. A
    # constant-rate spin (however fast) is featureless: every shift of it looks
    # like every other, so the peak is noise and the reported lag is garbage.
    # Measured case: |wz| ~ 3.0 rad/s with sd 0.06 gave corr 0.10 and a -275 ms
    # "lag". Refuse to report a number there instead of dressing it up.
    if wz.std() < 0.15:
        print(f"  odom omega_z sd={wz.std():.3f} rad/s (mean={wz.mean():+.3f}): the rotation is")
        print("  near-CONSTANT, which carries no timing feature. The lag is unconstrained --")
        print("  drive REVERSALS (spin one way, stop, spin back) and re-run.")
        return
    print(f"  odom omega_z sd={wz.std():.3f} rad/s -- enough variation to locate a lag")
    P = [(s, pr) for s, pr in node.prof if t0 <= s <= t1]
    if len(P) < 8:
        print(f"  only {len(P)} usable scan profiles -- need a longer moving window")
    else:
        # The profile-shift model assumes the sensor only ROTATES: translation
        # moves the scene by a range-dependent amount, which is not a rigid
        # circular shift and biases the recovered yaw. But simply DROPPING those
        # pairs punches holes in the series, and np.interp then draws straight
        # lines across the gaps -- fabricated signal that wrecks the correlation
        # (measured: corr 0.93 -> 0.70). So keep the series intact and instead
        # correlate only WITHIN contiguous low-translation runs, then combine.
        v_xy = np.linalg.norm(Oc[:, 5:7], axis=1)
        pairs = []
        for (s0, p0), (s1, p1) in zip(P[:-1], P[1:]):
            dt = s1 - s0
            if not (0.05 < dt < 0.3):
                continue
            sh = circular_shift(p0, p1)
            if sh is None:
                continue
            seg = (Oc[:, 0] >= s0) & (Oc[:, 0] <= s1)
            ok = (not seg.sum()) or (v_xy[seg].mean() <= args.v_max)
            pairs.append((0.5 * (s0 + s1), np.radians(sh * 360.0 / NBINS) / dt, ok))
        tl = np.array([q[0] for q in pairs])
        wl = np.array([q[1] for q in pairs])
        okm = np.array([q[2] for q in pairs], dtype=bool)
        print(f"  median |v_xy| over the window = {np.median(v_xy):.3f} m/s; "
              f"{tl.size} pairs, {okm.sum()} rotation-dominant (<= {args.v_max} m/s)")
        if tl.size < 8:
            print("  too few usable scan pairs")
        else:
            print(f"  recovered {tl.size} yaw-rate samples from scan pairs, "
                  f"|w|max={np.abs(wl).max():.3f} rad/s")

            def seg_lag(t, w):
                a, pa, _ = best_lag(Oc[:, 0], Oc[:, 4], t, w, fs=100.0)
                b, pb, _ = best_lag(Oc[:, 0], Oc[:, 4], t, -w, fs=100.0)
                if a is None and b is None:
                    return None, 0.0
                if a is None or (b is not None and pb > pa):
                    return b, pb
                return a, pa

            # contiguous runs of rotation-dominant pairs, each correlated on its own
            runs, i = [], 0
            while i < okm.size:
                if okm[i]:
                    j = i
                    while j < okm.size and okm[j]:
                        j += 1
                    if j - i >= 15:          # >= 1.5 s of 10 Hz scans
                        runs.append((i, j))
                    i = j
                else:
                    i += 1
            ests, weak = [], 0
            for (i, j) in runs:
                lg, pk = seg_lag(tl[i:j], wl[i:j])
                if lg is not None and pk >= 0.5:
                    ests.append((lg, pk, j - i))
                else:
                    weak += 1
            print(f"  {len(runs)} contiguous rotation-dominant runs; "
                  f"{len(ests)} with corr >= 0.5, {weak} too weak to use")
            lg_all, pk_all = seg_lag(tl, wl)
            if lg_all is not None:
                print(f"  {'':30s} all-pairs (ungated) reference: lag={lg_all*1e3:+7.1f} ms "
                      f"corr={pk_all:.3f}")
            for lg, pk, n in ests:
                print(f"  {'':30s} run of {n:3d} scans: lag={lg*1e3:+7.1f} ms  corr={pk:.3f}")
            if ests:
                L = np.array([e[0] for e in ests])
                med = float(np.median(L))
                print(f"  lidar yaw vs odom omega_z      MEDIAN over {len(ests)} clean runs = "
                      f"{med*1e3:+.1f} ms  (spread {(L.max()-L.min())*1e3:.1f} ms)")
                print(f"  {'':30s} => lidar header.stamp is {med*1e3:.0f} ms LATER than the"
                      f" instant its content depicts" if med > 0 else "")
            else:
                print("  no run gave a usable correlation -- the motion in this window does not")
                print("  constrain the lag; re-run during sustained in-place rotation.")
            print("  NOTE: lidar yaw is sampled at 10 Hz, so this lag is good to ~tens of ms,")
            print("        enough to catch render/pipeline staleness, not a 5 ms stamp bias.")

    return


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--imu', default='/imu')
    p.add_argument('--lidar', default='/livox/lidar')
    p.add_argument('--odom', default='/odom/mujoco_odom')
    p.add_argument('--bag', help='analyse a rosbag2 directory instead of live topics')
    p.add_argument('--duration', type=float, default=25.0)
    p.add_argument('--no-wait', action='store_true', help='do not wait for motion')
    p.add_argument('--omega-min', type=float, default=0.15, help='rad/s gate for "moving"')
    p.add_argument('--wait-timeout', type=float, default=180.0)
    p.add_argument('--v-max', type=float, default=0.25,
                   help='m/s; scan pairs with more translation are dropped from the '
                        'lidar yaw estimate (profile shift assumes pure rotation)')
    args = p.parse_args()

    rclpy.init(args=[])

    if args.bag:
        print(f"reading {args.bag} ...", flush=True)
        node = load_bag(args.bag, args.imu, args.lidar, args.odom)
        print(f"loaded imu={len(node.imu)} odom={len(node.odom)} lidar={len(node.lidar)} "
              f"profiles={len(node.prof)}")
        run_analysis(node, args)
        rclpy.shutdown()
        return

    node = Probe(args.imu, args.lidar, args.odom, want_lidar_yaw=True)

    if not args.no_wait:
        node.collecting = False
        print(f"waiting for motion (|omega_z| > {args.omega_min} rad/s) -- drive the robot now, "
              f"in-place rotation with a few reversals ...", flush=True)
        t_end = time.time() + args.wait_timeout
        while rclpy.ok() and time.time() < t_end:
            rclpy.spin_once(node, timeout_sec=0.05)
            if node.recent_omega() > args.omega_min:
                break
        else:
            print("timed out waiting for motion; collecting anyway")
        node.odom.clear()
        node.collecting = True

    print(f"collecting {args.duration:.0f} s ...", flush=True)
    end = time.time() + args.duration
    while rclpy.ok() and time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.02)
    node.collecting = False

    run_analysis(node, args)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main() or 0)
