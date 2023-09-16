import numpy as np


class car:
    def __init__(self, W, H, R):
        self.W = W
        self.H = H
        self.R = R
        self.vel = []
        self.MaxRpm = 360
        self.MinRpm = 250
        self.J_Inv = np.array([
            [1/self.R, -W/self.R],
            [1/self.R, W/self.R]
        ])
        self.J = np.array([
            [self.R/2, self.R/2],
            [-self.R/(2*W), self.R/(2*W)]
        ])

    def Ro(self, phi):
        r = np.array([
            [np.cos(phi), np.sin(phi), 0],
            [0, 0, 1]
        ])
        return r

    def RT(self, phi):
        r = np.array([
            [np.cos(phi), 0],
            [np.sin(phi), 0],
            [0, 1]
        ])
        return r

    def inverseKine(self, x1, y1, phi1, x2, y2, phi2):
        dphi = 0
        time = 0.05
        timestep = 0.001
        Vs = []
        for i in np.arange(0, time, timestep):
            print(i)
            dx = x2-x1
            dy = y2-y1
            dphi = phi2-phi1
            if (dphi > np.pi):
                dphi = 2*np.pi - dphi

            elif (dphi < -np.pi):
                dphi = 2*np.pi + dphi

            dtime = time-i
            Botvel = np.array([[dx/dtime],
                              [dy/dtime],
                               [dphi/dtime]])
            Rot = self.Ro(dphi)
            vs = np.array(
                np.dot(self.J_Inv, np.dot(Rot, Botvel))).reshape(2, 1)
            Vs = np.array(vs).reshape(2, 1)
            if abs(Vs[0][0]) > self.MaxRpm or abs(Vs[1][0]) > self.MaxRpm:
                MaxVel = abs(Vs[0][0])
                if abs(vs[1][0]) > MaxVel:
                    MaxVel = abs(Vs[1][0])
                Vs[0][0] *= self.MaxRpm/MaxVel
                Vs[1][0] *= self.MaxRpm/MaxVel
            if abs(Vs[0][0]) < self.MinRpm or abs(Vs[1][0]) < self.MinRpm:
                Vs[0][0] = self.MinRpm
                Vs[1][0] = self.MinRpm

            Rt = self.RT(dphi)
            self.J = np.array(self.J).reshape(2, 2)
            Eta = np.dot(Rt, np.dot(self.J, Vs))
            x1 += timestep*Eta[0][0]
            y1 += timestep*Eta[1][0]
            self.vel.append([Vs[0][0], Vs[1][0]])
        print(self.vel)


c = car(33, 20, 4)
c.inverseKine(0, 0, 0, 100, 0, 0)
