import numpy as np

#L = 8cm
class Kinematics:
    def __init__(self, theta_r=0, theta_p=0, L=8):
        self.theta_r = theta_r
        self.theta_p = theta_p
        self.L = L

    def initial_position(self):
        return [[self.L / (2 * np.sqrt(3)), self.L / (2 * np.sqrt(3)), (-self.L * np.sqrt(3)) / 4],
                [self.L / 2, -self.L / 2, 0],
                [0, 0, 0]]

    def translation_vector(self, z_A, z_B, z_C):
        return [z_A, z_B, z_C]

    def roll_rotation(self):
        return [[np.cos(self.theta_r), 0, -np.sin(self.theta_r)],
                [0, 1, 0],
                [np.sin(self.theta_r), 0, np.cos(self.theta_r)]]

    def pitch_rotation(self):
        return [[1, 0, 0],
                [0, np.cos(self.theta_p), -np.sin(self.theta_p)],
                [0, np.sin(self.theta_p), np.cos(self.theta_p)]]

    def homogeneous_transform(self, translation_vector):
        roll_transform = self.roll_rotation()
        pitch_transform = self.pitch_rotation()

        T = np.eye(4)
        T[:3, :3] = np.dot(roll_transform, pitch_transform)
        T[:3, 3] = translation_vector

        return T

    def update_position(self, T, initial_pos):
        array_eq = np.vstack([initial_pos, [1,1,1]])
        new_pos = np.dot(T, initial_pos)
        return new_pos

    def position_to_angle(self, z, r):
        return np.arcsin(z / r)

    def motor_angle(self, new_pos):
        ard_motor_A = self.position_to_angle(new_pos[2][0], self.L)
        ard_motor_B = self.position_to_angle(new_pos[2][1], self.L)
        ard_motor_C = self.position_to_angle(new_pos[2][2], self.L)
        return np.array([ard_motor_A, ard_motor_B, ard_motor_C])

    def inverse_kinematics(self, x, y, z):
        translation_vec = self.translation_vector(x, y, z)
        T = self.homogeneous_transform(translation_vec)
        new_pos = self.update_position(T, self.initial_position())
        motor_angles = self.motor_angle(new_pos)

        return motor_angles
