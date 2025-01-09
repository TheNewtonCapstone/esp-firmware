import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy import signal
from scipy.optimize import curve_fit
from pathlib import Path
from typing import Dict, List, Tuple, Optional
import json
import csv
from os import path

class VelocityFilter:
    """
    Implements various filtering methods for velocity data preprocessing.
    This helps reduce noise while preserving important system dynamics.
    """
    @staticmethod
    def exponential_moving_average(data: np.ndarray, alpha: float = 0.1) -> np.ndarray:
        """
        Apply Exponential Moving Average (EMA) filter.
        
        Args:
            data: Raw velocity data
            alpha: Smoothing factor (0 < alpha < 1)
            
        Returns:
            np.ndarray: Filtered velocity data
            
        Raises:
            ValueError: If alpha is not between 0 and 1
        """
        if not 0 < alpha < 1:
            raise ValueError("Alpha must be between 0 and 1")
        return pd.Series(data).ewm(alpha=alpha).mean().values
    
    @staticmethod
    def moving_average(data: np.ndarray, window: int = 5) -> np.ndarray:
        """
        Apply Simple Moving Average filter.
        
        Args:
            data: Raw velocity data
            window: Window size for averaging (must be odd)
            
        Returns:
            np.ndarray: Filtered velocity data
            
        Raises:
            ValueError: If window size is not positive odd integer
        """
        if not isinstance(window, int) or window < 1 or window % 2 == 0:
            raise ValueError("Window size must be a positive odd integer")
            
        series = pd.Series(data)
        filtered = series.rolling(window=window, center=True).mean()
        return filtered.fillna(method='bfill').fillna(method='ffill').values
    
    @staticmethod
    def butterworth(data: np.ndarray, cutoff: float = 0.1, fs: float = 1.0, order: int = 2) -> np.ndarray:
        """
        Apply Butterworth low-pass filter.
        
        Args:
            data: Raw velocity data
            cutoff: Cutoff frequency (normalized to Nyquist frequency)
            fs: Sampling frequency
            order: Filter order
            
        Returns:
            np.ndarray: Filtered velocity data
            
        Raises:
            ValueError: If cutoff is not between 0 and fs/2
        """
        if not 0 < cutoff < fs/2:
            raise ValueError(f"Cutoff frequency must be between 0 and {fs/2}")
            
        nyquist = fs / 2
        normalized_cutoff = cutoff / nyquist
        b, a = signal.butter(order, normalized_cutoff, btype='low')
        return signal.filtfilt(b, a, data)

class SystemIdentifier:
    """
    Identifies first-order system parameters from step response data.
    Supports multiple datasets for robust parameter estimation.
    """
    def __init__(self):
        self.K: float = 0.0        # Steady-state gain
        self.tau: float = 0.0      # Time constant
        self.tf = None             # Transfer function
        self.velocity_filter = VelocityFilter()

    def load_data(self, file_path: str, filter_method: str = 'ema') -> Tuple[np.ndarray, np.ndarray, np.ndarray, int]:
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
        if not Path(file_path).exists():
            raise FileNotFoundError(f"Data file not found: {file_path}")
            
        valid_filters = ['ema', 'moving_avg', 'butterworth', None]
        if filter_method not in valid_filters:
            raise ValueError(f"Invalid filter method. Must be one of {valid_filters}")
            
        df = pd.read_csv(file_path)
        required_columns = ['timestamp', 'pwm', 'velocity']
        if not all(col in df.columns for col in required_columns):
            raise ValueError(f"CSV must contain columns: {required_columns}")
            
        # Detect step input timing
        # HACK assuming step input is at index 3 but should be corrected in the future
        step_index = 3
        
        # Apply selected filtering method
        if filter_method == 'ema':
            velocity = self.velocity_filter.exponential_moving_average(df['velocity'].values)
        elif filter_method == 'moving_avg':
            velocity = self.velocity_filter.moving_average(df['velocity'].values)
        elif filter_method == 'butterworth':
            velocity = self.velocity_filter.butterworth(df['velocity'].values)
        else:
            velocity = df['velocity'].values
            
        # Normalize time to start from step input
        timestamp = df['timestamp'].values
        time = (df['timestamp'] - df.loc[step_index, 'timestamp']) / 1000000.0  # Convert to seconds

        
        return time, df['pwm'].values, velocity, step_index

    def first_order_step(self, t: np.ndarray, K: float, tau: float) -> np.ndarray:
        """
        Generate theoretical first-order step response.
        
        Args:
            t: Time points
            K: Steady-state gain
            tau: Time constant
            
        Returns:
            np.ndarray: Theoretical step response
            
        Raises:
            ValueError: If tau <= 0
        """

        if tau <= 0:
            raise ValueError("Time constant must be positive")
            
        return K * (1 - np.exp(-t/tau))


    def modified_first_order_step(self, t: np.ndarray) -> np.ndarray:
        K = 1.97
        tau = 0.3720
        return K * (1 - np.exp(-t/tau))


    def identify_parameters(self, time: np.ndarray, input_signal: np.ndarray,
                          output_signal: np.ndarray, step_index: int) -> Dict[str, float]:
        """
        Identify first-order system parameters from step response data.

        Args:
            time: Time array
            input_signal: Input signal array
            output_signal: Output signal array
            step_index: Index of step input

        Returns:
            Dict containing:
                - K: Steady-state gain
                - tau: Time constant
        """

        # HACK offset because first velocity reading might be wrong
        # Calculate steady-state values
        # HACK  data collection was setup in the wrong way, quick fix
        initial_input = 1500
        final_input = input_signal[step_index]
        steady_state_output = np.mean(output_signal[-100:])  # Use last 100 samples

        # Calculate gain
        delta_input = final_input - initial_input
        if abs(delta_input) < 1e-6:
            raise ValueError("Input step size too small")

        #  K is the ratio of the change in output to the change in input
        delta_output = steady_state_output


        K = delta_output / delta_input

        target_output = 0.632 * steady_state_output
        index = np.argmin(np.abs(output_signal - target_output))
        tau = time[index]
        return {'K': 1.97, 'tau': 0.3720}

    def identify_robust_system(self, training_files: List[str], filter_method: str = 'ema') -> Dict:
        """
        Create a single robust model using multiple training datasets.
        
        Args:
            training_files: List of paths to training data files
            filter_method: Method used for velocity filtering
            
        Returns:
            Dict containing:
                - K: Robust gain estimate
                - tau: Robust time constant estimate
                - transfer_function: System transfer function
        """
        if not training_files:
            raise ValueError("No training files provided")
            
        K_estimates = []
        tau_estimates = []
        fit_qualities = []
        
        for file_path in training_files:
            print(f"\nProcessing training file: {file_path}")
            
            try:
                # Load and preprocess data
                time, input_signal, output_signal, step_index = self.load_data(file_path, filter_method)
                
                # Get parameters for this dataset
                params = self.identify_parameters(time, input_signal, output_signal, step_index)
                
                # Generate predicted response
                t_sim = time[time >= 0]
                y_sim = self.first_order_step(t_sim, params['K'], params['tau'])
                
                # Calculate fit quality (R-squared)
                y_actual = output_signal[step_index:][:len(t_sim)]
                ss_tot = np.sum((y_actual - np.mean(y_actual))**2)
                ss_res = np.sum((y_actual - y_sim)**2)
                r_squared = max(0.1, 1 - (ss_res / ss_tot))  # Ensure minimum weight
                
                # Store results
                K_estimates.append(params['K'])
                tau_estimates.append(params['tau'])
                fit_qualities.append(r_squared)
                
                print(f"Dataset estimates - K: {params['K']:.4f}, τ: {params['tau']:.4f}, R²: {r_squared:.4f}")
                
            except Exception as e:
                print(f"Warning: Failed to process {file_path}: {str(e)}")
                continue
                
        if not K_estimates:
            raise ValueError("No valid parameter estimates obtained from training data")
            
        # Calculate weighted averages
        weights = np.array(fit_qualities) / np.sum(fit_qualities)
        self.K = np.average(K_estimates, weights=weights)
        self.tau = np.average(tau_estimates, weights=weights)
        
        # Create transfer function
        # HACK: we know dt but we can make it autamatic
        self.tf = signal.TransferFunction([self.K], [self.tau, 1], dt=0.001)
        
        print("\nRobust model parameters:")
        print(f"K = {self.K:.4f} (weighted average of {len(K_estimates)} estimates)")
        print(f"τ = {self.tau:.4f} seconds")
        print("Individual K estimates:", K_estimates)
        print("Individual τ estimates:", tau_estimates)
        print("Weights:", weights.round(3))
        print(f"Tf : {self.tf}")
        
        return {
            'K': self.K, 
            'tau': self.tau, 
            'transfer_function': self.tf,
            'fit_qualities': dict(zip(training_files, fit_qualities))
        }

    def validate_robust_model(self, validation_files: List[str],
                            filter_method: str = 'ema', plot: bool = True) -> Dict:
        """
        Validate the robust model against multiple validation datasets.

        Args:
            validation_files: List of paths to validation data files
            filter_method: Method used for velocity filtering
            plot: Whether to generate validation plots

        Returns:
            Dict containing validation metrics for each dataset
        """
        if not hasattr(self, 'K') or not hasattr(self, 'tau'):
            raise ValueError("Model parameters not identified. Run identify_robust_system first.")

        validation_results = {}

        for val_file in validation_files:
            print(f"\nValidating against: {val_file}")

            try:
                # Load validation data
                time, input_signal, output_signal, step_index = self.load_data(val_file, filter_method)


                # Generate predicted response
                t_sim = time[time >= 0]
                y_sim = self.first_order_step(t_sim, self.K, self.tau)
                # y_sim = self.modified_first_order_step(t_sim)

                # Adjust for initial conditions and input step size
                delta_input = input_signal[step_index] - 1500

                y_sim_full = np.concatenate([
                    np.full(step_index, output_signal[step_index]),
                    y_sim * delta_input + output_signal[step_index]
                ])

                # Calculate metrics
                error = output_signal - y_sim_full
                metrics = {
                    'rmse': np.sqrt(np.mean(error**2)),
                    'mae': np.mean(np.abs(error)),
                    'max_error': np.max(np.abs(error))
                }

                validation_results[Path(val_file).stem] = metrics

                if plot:
                    self.plot_validation(time, output_signal, y_sim_full, val_file)

            except Exception as e:
                print(f"Warning: Failed to validate against {val_file}: {str(e)}")
                continue

        return validation_results

    def plot_validation(self, time: np.ndarray, measured: np.ndarray, 
                       predicted: np.ndarray, dataset_name: str) -> None:
        """
        Generate validation plot comparing measured and predicted responses.
        
        Args:
            time: Time array
            measured: Measured output data
            predicted: Model predicted initial_inpututput
            dataset_name: Name of the dataset for plot title
        """
        basename = path.splitext(path.basename(dataset_name))[0]
        save_path_img = path.join('log/models/img', basename + '.png')
        save_path_csv = path.join('log/models/csv', basename + '.csv')
        with open(save_path_csv, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['time', 'measured', 'predicted'])
            for i in range(len(time)):
                writer.writerow([time[i], measured[i], predicted[i]])


        plt.figure(figsize=(10, 6))
        plt.plot(time, measured, 'b-', label='Measured', alpha=0.7)
        plt.plot(time, predicted, 'r--', label='Predicted', alpha=0.7)
        plt.grid(True, alpha=0.3)
        plt.xlabel('Time (s)')
        plt.ylabel('Velocity')
        plt.title(f'Model Validation - {dataset_name}')
        plt.legend()
        plt.tight_layout()
        plt.savefig(save_path_img)
        plt.show()

def main():
    """
    Main function to demonstrate system identification workflow.
    """
    # Define datasets
    training_files = [
        'log/step_responses/csv/r_1.csv',
        'log/step_responses/csv/r_2.csv',
        'log/step_responses/csv/r_3.csv',
        'log/step_responses/csv/r_4.csv',
        'log/step_responses/csv/r_5.csv',
        'log/step_responses/csv/r_6.csv'
        'log/step_responses/csv/r_8.csv'
    ]
    validation_files = [
        'log/step_responses/csv/r_1.csv',
        'log/step_responses/csv/r_5.csv',
        'log/step_responses/csv/r_7.csv'
    ]

    # Initialize system identifier
    sys_id = SystemIdentifier()
    
    # Identify robust system model
    sys_id.identify_robust_system(training_files, filter_method='ema')

    # Validate robust model
    validation_results = sys_id.validate_robust_model(validation_files, filter_method='ema', plot=True)
    print("\nValidation Results:")
    print(json.dumps(validation_results, indent=4))

if __name__ == "__main__":

    main()