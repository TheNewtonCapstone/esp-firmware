from typing import Tuple
import numpy as np
from scipy import signal
import pandas as pd
from pathlib import Path
from dataclasses import dataclass
import matplotlib.pyplot as plt
from os import path
import csv

class MotorModel:
    def __init__(self,file_path: str):
        self.dead_band = 30 # Dead band for the system
        self.reference_point = 1500 # Set point for the system
        self.save_path = file_path
        # Motor parameters (estimated) will be updated
        self.Kv = 340  # Motor velocity constant [rad/sec/pwm]
        self.Kt = (1.0 / self.Kv)
        self.Kt = self.Kt + self.Kt * 0.84# Motor torque constant [Nm/PWM]

        self.J = 0.000635    # Moment of inertia [kg*m^2]
        self.b = 0.004377    # Viscous friction coefficient [N*m*s]
        self.Ke = 0.01       # Back-EMF constant [PWM units/rad/sec]
        self.R = 0.023       # Combined PWM-to-torque factor
        self.L = 0.0002      # Electrical response time constant
        self.time = np.ndarray([])
        self.input = np.ndarray([])
        self.velocity = np.ndarray([])

        basename = path.splitext(path.basename(file_path))[0]
        self.save_path_img = path.join('log/models/img', basename + "up" + '.png')
        self.save_path_csv = path.join('log/models/csv', basename + "up" +'.csv')
        self.load_data(file_path)

    def step(self):
            pass

    def update_kt(self):
        """
        Estimate motor torque constant (Kt) from step response data.

        Method:
        1. Find steady-state velocity and input PWM
        2. At steady state: Kt * PWM = b * velocity
        3. Solve for Kt: Kt = (b * steady_state_velocity) / PWM_input

        Returns:
            float: Estimated torque constant [N⋅m/PWM]
        """
        # Find steady-state velocity and input PWM
        window_size = int(0.1 / 0.001)  # 100ms with 1ms timesteps

        acceleration = self.calculate_acceleration()



        # self.Kt = (self.b * ss_velocity) / ss_pwm

    def calculate_acceleration(self, window_size_ms: int = 20):
        window_size = int(0.268/0.001) # get the first 0.4 seconds of the data

        # Calculate the velocity at the start of the step
        v_window = self.velocity[:window_size]
        t_window = self.time[:window_size]

        # calculate acceleration using linear regression
        coeffs = np.polyfit(t_window, v_window, 1)
        a = coeffs[0]
        return a

    def update_B(self) -> float:
        """
        Estimate viscous friction coefficient (b) from step response at steady state.

        Method:
        At steady state: Kt * PWM = b * velocity
        Therefore: b = (Kt * PWM) / velocity

        Returns:
            float: Estimated viscous friction coefficient [N⋅m⋅s]
        """
        # Use last 20% of the data for steady state
        ss_start_idx = int(0.8 * len(self.time))
        print("staring index in calculate b", ss_start_idx)
        # Get mean steady state values
        ss_velocity = np.mean(self.velocity[ss_start_idx:])
        ss_input = np.mean(self.input[ss_start_idx:])
        # conversion
        conversion_factor = 0.0015455 # 1 PWM = 0.0015455 Nm
        ss_input = ss_input * conversion_factor


        # Calculate b using torque balance
        b_estimated = (self.Kt * ss_input) / ss_velocity

        # Print diagnostics
        print(f"\nSteady State Analysis:")
        print(f"Velocity: {ss_velocity:.2f} rad/s")
        print(f"Input PWM: {ss_input:.2f}")
        print(f"Using Kt: {self.Kt:.6f}")
        print(f"Estimated b: {b_estimated:.6f} N⋅m⋅s")

        # Verify estimate using multiple segments
        # Split last 20% into 4 segments and calculate b for each
        segment_size = int(0.05 * len(self.time))
        b_estimates = []

        for i in range(4):
            start_idx = ss_start_idx + (i * segment_size)
            end_idx = start_idx + segment_size

            v_mean = np.mean(self.velocity[start_idx:end_idx])
            i_mean = np.mean(self.input[start_idx:end_idx])

            b_segment = (self.Kt * i_mean) / v_mean
            b_estimates.append(b_segment)

        # Calculate statistics of segments
        b_std = np.std(b_estimates)
        print(f"\nVariation Analysis:")
        print(f"Standard deviation: {b_std:.6f}")
        print(f"Relative variation: {(b_std / b_estimated) * 100:.1f}%")

        return b_estimated
    def update_J(self):
        input_step = self.input[0]
        # J
        applied_torque = self.Kt * input_step
        a = self.calculate_acceleration()
        print(f'Applied Torque: {applied_torque}, Acceleration: {a}')
        # Calculate the inertia
        self.J = applied_torque / a

    def update_R(self) -> float:
        """
        Estimate motor resistance (R) from initial voltage/current relationship.

        Method:
        1. At t=0, V = IR (since back-EMF is zero)
        2. Use initial velocity response to estimate current
        3. R = V/I where I = τ/Kt and τ = J*α

        Returns:
            float: Estimated resistance [Ohm]
        """
        # Get the step size

        step_idx = 0
        V_step = self.input[step_idx]  # PWM value after step

        # Calculate initial acceleration (use first 5ms)
        window_size = int(0.005 / (self.time[1] - self.time[0]))  # 5ms window
        t_window = self.time[step_idx:step_idx + window_size]
        v_window = self.velocity[step_idx:step_idx + window_size]

        # Linear fit to get acceleration
        accel = self.calculate_acceleration()

        # Calculate initial current using I = τ/Kt = (J*α)/Kt
        initial_current = (self.J * accel) / self.Kt

        # Calculate R = V/I
        R_estimated = V_step / initial_current

        # Print diagnostics
        print(f"\nInitial Transient Analysis:")
        print(f"Step voltage (PWM): {V_step:.2f}")
        print(f"Initial acceleration: {accel:.2f} rad/s²")
        print(f"Estimated initial current: {initial_current:.2f} A")
        print(f"Estimated R: {R_estimated:.6f} Ohm")

        # Verify using multiple short windows
        windows_ms = [2, 3, 4, 5, 6]
        R_estimates = []

        for ms in windows_ms:
            window_size = int(ms * 0.001 / (self.time[1] - self.time[0]))
            t_win = self.time[step_idx:step_idx + window_size]
            v_win = self.velocity[step_idx:step_idx + window_size]
            current_win = ((self.J * accel) / self.Kt) / 1000
            print("Current", current_win)
            print("Acceleration ", accel)
            R_win = V_step / current_win
            R_estimates.append(R_win)

        print("\nVariation Analysis:")
        print(f"Mean R: {np.mean(R_estimates):.6f} Ohm")
        print(f"Std R: {np.std(R_estimates):.6f} Ohm")
        print("R estimated: ", R_estimated)
        self.R = R_estimated
        return R_estimated

    def update_Ke(self) -> float:
        """
        Estimate back-EMF constant (Ke) from steady state conditions.

        Method:
        At steady state: V = IR + Ke*ω
        Therefore: Ke = (V - IR)/ω

        Returns:
            float: Estimated back-EMF constant [V/(rad/s)]
        """
        # Use last 20% of data for steady state
        ss_start_idx = int(0.8 * len(self.time))

        # Get steady state values
        ss_velocity = np.mean(self.velocity[ss_start_idx:])
        ss_input = np.mean(self.input[ss_start_idx:])

        # Calculate steady state current using torque balance
        # τ = Kt*i = b*ω
        ss_current = (self.b * ss_velocity) / self.Kt

        # Calculate Ke using voltage balance
        # V = IR + Ke*ω
        # Ke = (V - IR)/ω
        Ke_estimated = -(ss_input - self.R * ss_current) / ss_velocity

        # Print diagnostics
        print("\nSteady State Analysis:")
        print(f"Velocity: {ss_velocity:.2f} rad/s")
        print(f"Input voltage: {ss_input:.2f} V")
        print(f"Estimated current: {ss_current:.2f} A")
        print(f"Estimated Ke: {Ke_estimated:.6f} V/(rad/s)")

        # Verify using multiple segments
        segment_size = int(0.05 * len(self.time))
        Ke_estimates = []

        for i in range(4):
            start_idx = ss_start_idx + (i * segment_size)
            end_idx = start_idx + segment_size

            v_mean = np.mean(self.velocity[start_idx:end_idx])
            i_mean = np.mean(self.input[start_idx:end_idx])
            current = (self.b * v_mean) / self.Kt

            Ke_seg = (i_mean - self.R * current) / v_mean
            Ke_estimates.append(Ke_seg)

        # Calculate statistics
        Ke_std = np.std(Ke_estimates)
        print("\nVariation Analysis:")
        print(f"Standard deviation: {Ke_std:.6f}")
        print(f"Relative variation: {(Ke_std / Ke_estimated) * 100:.1f}%")
        print(f"\nUpdating Ke to {Ke_estimated:.6f} V/(rad/s)")
        print(f"kt: {self.Kt}")

        return Ke_estimated
    def load_data(self, file_path: str)->None:
        """
        Load and preprocess step response data.

        Args:
            file_path: Path to CSV file
            filter_method: Filtering method ('ema', 'moving_avg', 'butterworth', None)

        Returns:
            Tuple containing:
                - Time array
                - Input signal array
                - Filtered velocity array
                - Step input index

        Raises:
            FileNotFoundError: If file_path doesn't exist
            ValueError: If invalid filter_method specified
        """
        file_path = file_path + '.csv'
        print(file_path)
        df = pd.read_csv(file_path)
        if not Path(file_path).exists():
            raise FileNotFoundError(f"Data file not found: {file_path}")

        required_columns = ['timestamp', 'pwm', 'velocity']
        if not all(col in df.columns for col in required_columns):
            raise ValueError(f"CSV must contain columns: {required_columns}")

        # Detect step input timing
        # HACK assuming step input is at index 3 but should be corrected in the future
        step_index = 2

        # remove the first
        self.velocity = df['velocity'].values[step_index:]

        # Normalize time to start from step input
        timestamp = df['timestamp'].values[step_index:]

        self.input = df['pwm'][step_index:] - self.reference_point
        time_range = abs(timestamp - timestamp[0]) / 1000000.0  # in seconds
        # First get the time range
        time_max = time_range.max()
        # Create uniform time array with 1ms steps
        self.time = np.arange(0, time_max, 0.001)
        if self.time.size !=  self.velocity.size:
            # Interpolate velocity data to match time array
            self.velocity = np.interp(self.time, time_range, self.velocity)

        if self.input.size != self.time.size:
            # Interpolate input data to match time array
            self.input = np.interp(self.time, time_range, self.input)




    def get_tf(self):
        # Transfer function for the motor model
        #num = motor torque constant Kt estimated
        #den = [J, b, Ke, Kt, R]
        num = [self.Kt]
        den = [self.J * self.L, self.J * self.R + self.b * self.L, self.b * self.R + self.Kt * self.Ke]

        print(f'num: {num}, den: {den}')
        return signal.TransferFunction(num, den)

    def simulate_response(self):
        system = self.get_tf()
        t_out, y_out, x_out = signal.lsim(system, self.input, self.time)
        # convert from rad/s to RPM
        # velocity = self.velocity * 60 / (2 * np.pi)

        self.save_model(t_out, self.velocity, y_out)

        return t_out, y_out, x_out

    def save_model(self, time: np.ndarray, measured: np.ndarray, predicted: np.ndarray):
        # Save the model parameters

        with open(self.save_path_csv, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['time', 'measured', 'predicted'])
            for i in range(len(time)):
                writer.writerow([time[i], measured[i], predicted[i]])


        fig, (ax1) = plt.subplots(1, 1, figsize=(15, 12))
        fig.tight_layout(pad=3.0)
        # Plot 1: Velocity vs Time with target line
        ax1.plot(time, predicted, linewidth=2, color='#2E86C1', label='Predicted')
        ax1.plot(time, measured, linewidth=2, color='#27AE60', label='measured')

        ax1.set_xlabel('Time')
        ax1.set_ylabel('Velocity')
        ax1.set_title('Velocity vs Time')
        ax1.grid(True)
        ax1.legend()
        plt.savefig(self.save_path_img, dpi=300, bbox_inches='tight')
        plt.show()



def main():
    training_files = [
        'log/step_responses/csv/r_1',
        'log/step_responses/csv/r_2',
        'log/step_responses/csv/r_3',
        'log/step_responses/csv/r_4',
        'log/step_responses/csv/r_5',
        'log/step_responses/csv/r_6'
        'log/step_responses/csv/r_8'
    ]
    validation_files = [
        'log/step_responses/csv/r_1',
        'log/step_responses/csv/r_5',
        'log/step_responses/csv/r_7'
    ]

    # Load training data
    model = MotorModel(training_files[2])
    model.update_J()
    model.update_B()
    model.update_R()
    model.update_Ke()
    # model.update_kt()
    #
    model.simulate_response()



if __name__ == '__main__':
    main()

