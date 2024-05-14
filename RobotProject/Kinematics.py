import numpy as np

#L = 24cm
class Kinematics:
    def __init__(self, theta_r=0, theta_p=0, L=24, R=2.5):
        self.theta_r = theta_r
        self.theta_p = theta_p
        self.L = L
        self.R = R
        self.z = 0

    def initial_position(self):
        return [[self.L / (2 * np.sqrt(3)), self.L / (2 * np.sqrt(3)), (-self.L /np.sqrt(3)) ],
                [-self.L / 2, +self.L / 2, 0],
                [0, 0, 0]]

    def translation_vector(self, z_A, z_B, z_C):
        return np.array([z_A, z_B, z_C])

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
        new_pos = np.dot(T, array_eq)
        return new_pos

    def position_to_angle(self, z, r):
        return np.arcsin(z / r)

    def motor_angle(self, new_pos):
        ard_motor_A = self.position_to_angle(new_pos[2][0], self.L)
        ard_motor_B = self.position_to_angle(new_pos[2][1], self.L)
        ard_motor_C = self.position_to_angle(new_pos[2][2], self.L)
        return np.array([ard_motor_A, ard_motor_B, ard_motor_C])
    

    def inverse_kinematics(self, roll, pitch, z):
        self.theta_p = pitch
        self.theta_r = roll
        self.z = z
        translation_vec = self.translation_vector(0, 0, z)

        print(f"{translation_vec=}")
        T = self.homogeneous_transform(translation_vec)
        new_pos = self.update_position(T, self.initial_position())
        motor_angles = self.motor_angle(new_pos)

        motor_angles_deg = np.rad2deg(motor_angles)
        motor_angles_deg = 90 + motor_angles_deg

        motor_angles_deg = np.round(motor_angles_deg,2)

        return motor_angles_deg.tolist()