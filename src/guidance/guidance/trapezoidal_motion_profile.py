import numpy as np

class TrapezoidalMotionProfile:
    def __init__(self, start, end, max_vel, max_acc, dt):
        self.start = start
        self.end = end
        self.max_vel = max_vel
        self.max_acc = max_acc
        self.dt = dt
        self.profile, self.times = self.generate_profile()

    def generate_profile(self):
        distance = self.end - self.start
        sign = np.sign(distance)
        distance = np.abs(distance)

        t_acc = self.max_vel / self.max_acc
        d_acc = 0.5 * self.max_acc * t_acc ** 2

        if 2 * d_acc < distance:
            t_cruise = (distance - 2 * d_acc) / self.max_vel
            t_total = 2 * t_acc + t_cruise
        else:
            t_acc = np.sqrt(distance / self.max_acc)
            t_cruise = 0
            t_total = 2 * t_acc

        times = np.arange(0, t_total, self.dt)
        positions = []

        for t in times:
            if t < t_acc:
                pos = self.start + 0.5 * self.max_acc * t ** 2 * sign
            elif t < t_acc + t_cruise:
                pos = self.start + (d_acc + self.max_vel * (t - t_acc)) * sign
            else:
                t_dec = t - t_acc - t_cruise
                pos = self.end - 0.5 * self.max_acc * t_dec ** 2 * sign
            positions.append(pos)

        return np.array(positions), times

    def get_desired_position(self, current_time):
        if current_time >= self.times[-1]:
            return self.end
        return np.interp(current_time, self.times, self.profile)
