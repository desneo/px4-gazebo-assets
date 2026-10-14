"""连续仿真时间窗口中的稳定平飞速度；瞬态峰值单独统计。"""
from collections import deque
import math
import statistics


class StableSpeed:
    def __init__(self, duration=10., max_gap=.15, altitude_tolerance=2.,
                 speed_std=.5, acceleration=.2, target=95., target_tolerance=1., speed_range=2.):
        for value in (duration, max_gap, altitude_tolerance, speed_std, acceleration,
                      target, target_tolerance, speed_range):
            if not math.isfinite(value) or value <= 0:
                raise ValueError('稳定窗口参数必须为有限正数')
        self.duration, self.max_gap = duration, max_gap
        self.altitude_tolerance = altitude_tolerance
        self.speed_std, self.acceleration = speed_std, acceleration
        self.target, self.target_tolerance = target, target_tolerance
        self.speed_range = speed_range
        self.rows = deque()
        self.best = None
        self.previous = None
        self.peak = None
        self.max_altitude_error = 0.
        self.sample_count = 0
        self.discontinuities = 0
        self.last_evaluation = -math.inf

    def update(self, stamp, speed, altitude_error):
        if not all(math.isfinite(v) for v in (stamp, speed, altitude_error)) or speed < 0:
            raise ValueError('真值采样包含非法数值')
        if self.previous is not None and stamp <= self.previous:
            raise ValueError('真值仿真时间必须严格递增')
        if self.previous is not None and stamp-self.previous > self.max_gap:
            self.rows.clear()
            self.discontinuities += 1
        self.previous = stamp
        self.sample_count += 1
        self.peak = max(self.peak or 0., speed)
        self.max_altitude_error = max(self.max_altitude_error, abs(altitude_error))
        if abs(altitude_error) > self.altitude_tolerance:
            self.rows.clear()
            return None
        self.rows.append((stamp, speed, abs(altitude_error)))
        # 保留恰好覆盖所需时长的最短连续窗口；不插值掩盖数据缺口。
        while len(self.rows) > 1 and stamp-self.rows[1][0] >= self.duration:
            self.rows.popleft()
        if stamp-self.rows[0][0] < self.duration or stamp-self.last_evaluation < .1:
            return None
        self.last_evaluation = stamp
        rows = list(self.rows)
        values = [r[1] for r in rows]
        mean = statistics.fmean(values)
        deviation = statistics.pstdev(values)
        times = [r[0]-rows[0][0] for r in rows]
        time_mean = statistics.fmean(times)
        denominator = sum((t-time_mean)**2 for t in times)
        slope = sum((t-time_mean)*(v-mean) for t, v in zip(times, values))/denominator
        end_slope = (values[-1]-values[0])/(times[-1]-times[0])
        if (deviation > self.speed_std or max(values)-min(values) > self.speed_range
                or abs(slope) > self.acceleration or abs(end_slope) > self.acceleration):
            return None
        window = {'start_sim_time_s': rows[0][0], 'end_sim_time_s': stamp,
                  'duration_s': times[-1], 'samples': len(rows),
                  'mean_m_s': mean, 'median_m_s': statistics.median(values),
                  'min_m_s': min(values), 'max_m_s': max(values),
                  'range_m_s': max(values)-min(values),
                  'std_m_s': deviation, 'fitted_acceleration_m_s2': slope,
                  'endpoint_acceleration_m_s2': end_slope,
                  'max_altitude_error_m': max(r[2] for r in rows),
                  'target_reached': abs(mean-self.target) <= self.target_tolerance}
        if self.best is None or mean > self.best['mean_m_s']:
            self.best = window
        return window

    def summary(self):
        return {'valid_stable_window': self.best is not None, 'stable_window': self.best,
                'peak_speed_m_s': self.peak, 'max_altitude_error_m': self.max_altitude_error,
                'samples': self.sample_count, 'discontinuities': self.discontinuities,
                'target_reached': bool(self.best and self.best['target_reached'])}
