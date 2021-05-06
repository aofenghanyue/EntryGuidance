class Earth:
    Re = 6.371e6  # 平均地球半径 m
    omega_e = 7.292e-5  # 地球自转角速度 rad/s
    Me = 5.9722e24  # 地球质量 kg
    G = 6.674e-11  # 万有引力常数 m^3 kg^-1 s^-2
    mu = Me * G  # 地心引力常数 m^3 s^-2
    g0 = 9.8  # m/s^2

    def g(self, h):
        return self.mu / (self.Re + h) ** 2

    def E(self, v, h):
        return v ** 2 / 2 - self.mu / (self.Re + h)


earth = Earth()

if __name__ == '__main__':
    print(Earth.mu)
