from dataclasses import dataclass, field
import numpy as np
from scipy.optimize import linear_sum_assignment


@dataclass
class Track:
    id: int
    label: str
    state: np.ndarray
    covariance: np.ndarray
    bbox: np.ndarray
    last_seen: float
    score: float
    hits: int = 1
    observed: bool = True
    streak: int = 1
    history: list = field(default_factory=list)

    @property
    def confirmed(self): return self.streak >= 5
    @property
    def sigma_deg(self): return float(np.sqrt(np.linalg.eigvalsh(self.covariance[:2, :2]).max()))


class Tracker:
    """tracks angular bearings, so camera rotation doesn't become object motion."""
    def __init__(self, max_age=.8, measurement_std=.18, acceleration_std=12):
        self.max_age, self.measurement_std, self.acceleration_std = max_age, measurement_std, acceleration_std
        self.tracks, self.next_id, self.timestamp = [], 1, None
        self.H = np.eye(2, 4)

    def update(self, detections, timestamp, calibration, pan, tilt, mode='daylight'):
        if not np.isfinite(timestamp) or (self.timestamp is not None and timestamp <= self.timestamp):
            raise ValueError("frame times must increase")
        dt = 0 if self.timestamp is None else timestamp - self.timestamp
        self.timestamp = timestamp
        if dt > self.max_age: self.tracks.clear()
        measurements = [calibration.bearing(d.center, pan, tilt) for d in detections]
        F = np.eye(4); F[0, 2] = F[1, 3] = dt
        G = np.array([[dt*dt/2, 0], [0, dt*dt/2], [dt, 0], [0, dt]])
        # noisy night proposals and quicker flight need a less overconfident filter.
        measurement_std=.9 if mode=='night' else self.measurement_std
        acceleration_std=45 if mode=='night' else self.acceleration_std
        Q = G @ G.T * acceleration_std**2
        R = np.eye(2) * measurement_std**2
        for t in self.tracks:
            if not t.observed: t.streak = 0
            t.state = F @ t.state
            t.covariance = F @ t.covariance @ F.T + Q
            t.observed = False
        cost = np.full((len(self.tracks), len(detections)), 1e6)
        for i, t in enumerate(self.tracks):
            S = self.H @ t.covariance @ self.H.T + R
            for j, d in enumerate(detections):
                if t.label != d.label: continue
                innovation = measurements[j] - t.state[:2]
                distance = float(innovation @ np.linalg.solve(S, innovation))
                if distance <= 9.21: cost[i, j] = distance
        rows, cols = linear_sum_assignment(cost)
        used = set()
        for i, j in zip(rows, cols):
            if cost[i, j] >= 1e6: continue
            t, d = self.tracks[i], detections[j]
            S = self.H @ t.covariance @ self.H.T + R
            K = np.linalg.solve(S, self.H @ t.covariance).T
            t.state += K @ (measurements[j] - t.state[:2])
            A = np.eye(4) - K @ self.H
            t.covariance = A @ t.covariance @ A.T + K @ R @ K.T
            t.bbox, t.last_seen, t.score = d.bbox.copy(), timestamp, d.score
            t.hits += 1; t.streak += 1; t.observed = True; used.add(j)
        self.tracks = [t for t in self.tracks if timestamp - t.last_seen <= self.max_age]
        for j, d in enumerate(detections):
            if j in used: continue
            if mode=='night':
                duplicate=False
                for existing in self.tracks:
                    if existing.label!=d.label:continue
                    center=calibration.pixel(existing.state[:2],pan,tilt)
                    if center is None:continue
                    half=(existing.bbox[2:]-existing.bbox[:2])/2
                    predicted_box=np.r_[center-half,center+half]
                    intersection=np.maximum(0,np.minimum(predicted_box[2:],d.bbox[2:])-np.maximum(predicted_box[:2],d.bbox[:2])).prod()
                    union=np.prod(half*2)+np.prod(d.bbox[2:]-d.bbox[:2])-intersection
                    if intersection/max(union,1)>.15:duplicate=True;break
                # an outlier inside an existing silhouette must not create a competing ID.
                if duplicate:continue
            self.tracks.append(Track(self.next_id, d.label, np.r_[measurements[j], 0., 0.],
                np.diag([measurement_std**2]*2 + [36., 36.]), d.bbox.copy(), timestamp, d.score))
            self.next_id += 1
        for t in self.tracks:
            p = calibration.pixel(t.state[:2], pan, tilt)
            if p is not None:
                t.history.append(tuple(p)); del t.history[:-30]
        return list(self.tracks)


class CameraServo:
    def __init__(self, gain=3.0, deadband=.15):
        self.gain, self.deadband, self.lock_id = gain, deadband, None

    def command(self, tracks, timestamp, frame_age, pan, tilt):
        if frame_age > .12 or frame_age < 0: return np.zeros(2), "stale frame"
        confirmed = [t for t in tracks if t.confirmed and t.label == 'drone' and t.observed]
        if self.lock_id is None and confirmed:
            self.lock_id = min(confirmed, key=lambda t: np.linalg.norm(t.state[:2] - [pan, tilt])).id
        selected = next((t for t in tracks if t.id == self.lock_id), None)
        if selected is None: return np.zeros(2), "no confirmed track"
        if not selected.observed or timestamp - selected.last_seen > .12:
            return np.zeros(2), "occluded / lost"
        if selected.sigma_deg > .8: return np.zeros(2), "uncertain track"
        error = selected.state[:2] - np.array([pan, tilt])
        error[np.abs(error) < self.deadband] = 0
        # follow estimated target motion, then correct residual centering error.
        velocity = np.clip(error * self.gain + selected.state[2:], [-45, -30], [45, 30])
        for i, limits in enumerate(((-150, 150), (-25, 60))):
            angle = (pan, tilt)[i]
            if (angle <= limits[0] + 1 and velocity[i] < 0) or (angle >= limits[1] - 1 and velocity[i] > 0): velocity[i] = 0
        return velocity, "tracking"
