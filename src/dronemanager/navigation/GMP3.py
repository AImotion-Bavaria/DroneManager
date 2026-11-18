import numpy as np
from scipy.interpolate import CubicSpline

# Algorithm written by Dr. techn. Babak Salamat, OOP structure created by Tim Drouven
# Date: 23.10.2025
# Update: Class. 23.05.2024

from scipy.interpolate import CubicSpline


class GMP3Config:
    def __init__(self, maxit, alpha, wdamp, delta,
                 vx_max, vy_max, vz_max,
                 Q11, Q22, Q33, Q12, Q13, Q23,
                 dt, x_max, x_min, y_max, y_min, z_max, z_min, obstacles=None):
        """Obstacles: list of tuples (x, y, z, r)
        """
        self.maxit  = maxit
        self.alpha  = alpha
        self.wdamp  = wdamp
        self.delta  = delta
        self.vx_max = vx_max
        self.vy_max = vy_max
        self.vz_max = vz_max
        self.Q11    = Q11
        self.Q22    = Q22
        self.Q33    = Q33
        self.Q12    = Q12
        self.Q13    = Q13
        self.Q23    = Q23
        self.dt     = dt
        self.x_max  = x_max
        self.x_min = x_min
        self.y_max  = y_max
        self.y_min = y_min
        self.z_max  = z_max
        self.z_min = z_min
        self.obstacles = obstacles if obstacles is not None else []
        self.nobs   = len(self.obstacles)
        self.pathfound = []
        self.iterations_needed = []


class GMP3:
    def __init__(self, gmpConfig: GMP3Config):
        self.__dict__.update(vars(gmpConfig))
        self.tf = None
        self.verbose = False

        # data placeholders
        self.x = []; self.y = []; self.z = []
        self.t = []; self.xdot = []; self.ydot = []; self.zdot = []
        self.theta = []; self.violation = []; self.computedValue = []
        self.robs = []; self.Xobs = []; self.Yobs = []; self.Zobs = []

    def enforce_bounds(self, x, y, z):
        x = np.clip(x, self.x_min, self.x_max)
        y = np.clip(y, self.y_min, self.y_max)
        z = np.clip(z, self.z_min, self.z_max)
        return x, y, z

    def _unpack_theta(self, theta):
        third = len(theta) // 3
        return theta[:third], theta[third:2*third], theta[2*third:]

    def Cost1(self, xobs, yobs, zobs, robs, x_in, y_in, z_in,
              x_f1, x_f2, y_f1, y_f2, z_f1, z_f2, theta):
        J, _ = self.Cost(xobs, yobs, zobs, robs, x_in, y_in, z_in,
                         x_f1, x_f2, y_f1, y_f2, z_f1, z_f2, theta)
        return J

    def Cost(self, xobs, yobs, zobs, robs, x_in, y_in, z_in,
             x_f1, x_f2, y_f1, y_f2, z_f1, z_f2, theta):
        wx, wy, wz = self._unpack_theta(theta)
        Ts = self.dt
        tf = self.tf
        t = np.arange(0, tf + Ts, Ts)
        k = len(t)
        tt = np.linspace(0, tf, len(wx) + 2)

        XS = np.array([x_in] + list(wx) + [x_f2])
        YS = np.array([y_in] + list(wy) + [y_f2])
        ZS = np.array([z_in] + list(wz) + [z_f2])

        cs_x = CubicSpline(tt, XS, bc_type='natural')
        cs_y = CubicSpline(tt, YS, bc_type='natural')
        cs_z = CubicSpline(tt, ZS, bc_type='natural')

        rxd = cs_x(t)
        ryd = cs_y(t)
        rzd = cs_z(t)
        rxd, ryd, rzd = self.enforce_bounds(rxd, ryd, rzd)

        vrxd = np.append(cs_x(t, 1), 0)
        vryd = np.append(cs_y(t, 1), 0)
        vrzd = np.append(cs_z(t, 1), 0)

        x, y, z = np.full(k, x_in), np.full(k, y_in), np.full(k, z_in)
        xdot = np.zeros(k); ydot = np.zeros(k); zdot = np.zeros(k)

        for i in range(1, k):
            x[i] = x[i-1] + vrxd[i-1] * Ts
            y[i] = y[i-1] + vryd[i-1] * Ts
            z[i] = z[i-1] + vrzd[i-1] * Ts
            xdot[i], ydot[i], zdot[i] = vrxd[i-1], vryd[i-1], vrzd[i-1]
            x[i], y[i], z[i] = self.enforce_bounds(x[i], y[i], z[i])

        QQ = np.array([[self.Q11, self.Q12, self.Q13],
                       [self.Q12, self.Q22, self.Q23],
                       [self.Q13, self.Q23, self.Q33]])

        dx, dy, dz = np.diff(x), np.diff(y), np.diff(z)
        quad = np.sum(QQ[0,0]*dx**2 + QQ[1,1]*dy**2 + QQ[2,2]*dz**2 +
                      2*QQ[0,1]*dx*dy + 2*QQ[0,2]*dx*dz + 2*QQ[1,2]*dy*dz)

        Violation = 0
        for j in range(len(xobs)):
            d = np.sqrt((x - xobs[j])**2 + (y - yobs[j])**2 + (z - zobs[j])**2)
            v = np.maximum(1 - d / (robs[j] + 0.5), 0)
            vvx = np.maximum(np.abs(xdot) / self.vx_max - 1, 0)
            vvy = np.maximum(np.abs(ydot) / self.vy_max - 1, 0)
            vvz = np.maximum(np.abs(zdot) / self.vz_max - 1, 0)
            Violation += np.mean(v) + np.mean(vvx) + np.mean(vvy) + np.mean(vvz) + 0.1*np.std(v)

        J = -(0.1 * quad + 3 * Violation)
        return J, Violation

    def Agents(self, xobs, yobs, zobs, robs, x_in, y_in, z_in,
               x_f1, x_f2, y_f1, y_f2, z_f1, z_f2, theta, delta, n):
        grad = np.zeros(n)
        for i in range(n):
            e = np.zeros(n); e[i] = 1
            grad[i] = (-self.Cost1(xobs, yobs, zobs, robs, x_in, y_in, z_in,
                                   x_f1, x_f2, y_f1, y_f2, z_f1, z_f2, theta + 2*delta*e)
                        + 8*self.Cost1(xobs, yobs, zobs, robs, x_in, y_in, z_in,
                                       x_f1, x_f2, y_f1, y_f2, z_f1, z_f2, theta + delta*e)
                        - 8*self.Cost1(xobs, yobs, zobs, robs, x_in, y_in, z_in,
                                       x_f1, x_f2, y_f1, y_f2, z_f1, z_f2, theta - delta*e)
                        + self.Cost1(xobs, yobs, zobs, robs, x_in, y_in, z_in,
                                     x_f1, x_f2, y_f1, y_f2, z_f1, z_f2, theta - 2*delta*e)) / (12*delta)
        return grad

    def results(self, x_in, y_in, z_in, x_f1, x_f2, y_f1, y_f2, z_f1, z_f2, theta):
        wx, wy, wz = self._unpack_theta(theta)
        Ts = self.dt
        tf = self.tf
        t = np.arange(0, tf + Ts, Ts)
        tt = np.linspace(0, tf, len(wx) + 2)
        cs_x = CubicSpline(tt, [x_in] + list(wx) + [x_f2], bc_type='natural')
        cs_y = CubicSpline(tt, [y_in] + list(wy) + [y_f2], bc_type='natural')
        cs_z = CubicSpline(tt, [z_in] + list(wz) + [z_f2], bc_type='natural')

        rxd, ryd, rzd = cs_x(t), cs_y(t), cs_z(t)
        vrxd, vryd, vrzd = cs_x(t, 1), cs_y(t, 1), cs_z(t, 1)
        return t, rxd, ryd, rzd, vrxd, vryd, vrzd

    def calculate(self, startposition, finalposition):
        self.x_in, self.y_in, self.z_in = startposition
        self.x_f1, self.y_f1, self.z_f1 = finalposition
        self.x_f2, self.y_f2, self.z_f2 = finalposition

        dist = np.linalg.norm(np.array(finalposition) - np.array(startposition))
        vmax = np.sqrt(self.vx_max**2 + self.vy_max**2 + self.vz_max**2)
        self.tf = 2 * dist / vmax

        theta = np.zeros(3 * self.nobs)
        if self.nobs > 0:
            x_values = np.linspace(self.x_in, self.x_f2, self.nobs)
            y_values = np.linspace(self.y_in, self.y_f2, self.nobs)
            z_values = np.linspace(self.z_in, self.z_f2, self.nobs)
            theta[:] = np.concatenate([x_values, y_values, z_values])

        m = 0; v = 0; beta1, beta2 = 0.05, 0.95
        previous_value = float('inf')

        for i in range(self.maxit):
            if len(self.obstacles) > 0:
                Xobs, Yobs, Zobs, robs = np.transpose(np.array(self.obstacles))
            else:
                Xobs = Yobs = Zobs = robs = np.array([])

            grad = self.Agents(Xobs, Yobs, Zobs, robs,
                               self.x_in, self.y_in, self.z_in,
                               self.x_f1, self.x_f2, self.y_f1, self.y_f2, self.z_f1, self.z_f2,
                               theta, self.delta, len(theta))
            self.alpha *= self.wdamp

            m = beta1 * m + (1 - beta1) * grad
            v = beta2 * v + (1 - beta2) * grad**2
            mhat = m / (1 - beta1**(i+1))
            vhat = v / (1 - beta2**(i+1))
            theta += self.alpha * mhat / (np.sqrt(1 + vhat))

            jj, V = self.Cost(Xobs, Yobs, Zobs, robs,
                              self.x_in, self.y_in, self.z_in,
                              self.x_f1, self.x_f2, self.y_f1, self.y_f2, self.z_f1, self.z_f2, theta)
            current_value = abs(jj)
            if (V == 0 and abs(current_value - previous_value) < 0.05) or np.linalg.norm(grad) < 1e-5:
                self.pathfound = True
                break
            previous_value = current_value
            if self.verbose:
                print(f"Iter {i+1}: Violation={V:.4f} Value={current_value:.4f}")

        self.iterations_needed = i + 1
        self.violation = V
        self.computedValue = current_value
        self.theta = theta
        self.Xobs, self.Yobs, self.Zobs, self.robs = Xobs, Yobs, Zobs, robs
        self.t, self.x, self.y, self.z, self.xdot, self.ydot, self.zdot = self.results(
            self.x_in, self.y_in, self.z_in, self.x_f1, self.x_f2, self.y_f1, self.y_f2, self.z_f1, self.z_f2, theta
        )
