"""
    Python script to compare different velocity filters
"""
import numpy as np
from scipy.signal import butter, filtfilt
import matplotlib.pyplot as plt
import pandas as pd
import csv  

class VelocityFilters:
    def __init__(self):
        # Filter parameters
        self.ema_alpha = 0.3
        self.ma_window = 5
        self.butterworth_order = 2
        self.butterworth_cutoff = 0.1  # Normalized cutoff frequency
        
        # SimpleFOC state variables
        self.prev_timestamp = 0
        self.prev_pulse_counter = 0
        self.prev_Th = 0
        self.pulse_per_second = 0
        
        
    def butterworth_filter(self, data):
        """
        Apply Butterworth filter to velocity data
        Parameters:
            data (array): Input velocity data
        Returns:
            array: Filtered velocity data
        """
        b, a = butter(self.butterworth_order, self.butterworth_cutoff)
        return filtfilt(b, a, data)
    
    def exponential_moving_average(self, data):
        """
        Apply Exponential Moving Average filter
        Parameters:
            data (array): Input velocity data
        Returns:
            array: Filtered velocity data
        """
        filtered = np.zeros_like(data)
        filtered[0] = data[0]
        
        for i in range(1, len(data)):
            filtered[i] = self.ema_alpha * data[i] + (1 - self.ema_alpha) * filtered[i-1]

            
        return filtered
    
    def moving_average(self, data):
        """
        Apply Simple Moving Average filter with proper padding
        Parameters:
            data (array): Input velocity data
        Returns:
            array: Filtered velocity data with same length as input
        """
        # Calculate the valid convolution
        ma_valid = np.convolve(data, np.ones(self.ma_window)/self.ma_window, mode='valid')
        
        # Calculate padding size
        pad_size = len(data) - len(ma_valid)
        # Add padding at the start of the array
        padding = np.ones(pad_size) * ma_valid[0]
        
        return np.concatenate([padding, ma_valid])
    
    def simplefoc_filter(self, timestamps, pulse_counts, cpr):
        """
        Implement SimpleFOC's velocity estimation algorithm
        Parameters:
            timestamps (array): Timestamp data in microseconds
            pulse_counts (array): Encoder pulse count data
            cpr (int): Counts per revolution
        Returns:
            array: Estimated velocities
        """
        velocities = []
        prev_velocity = 0
        

        self.prev_timestamp = timestamps[0]
        self.prev_pulse_counter = pulse_counts[0]
        count = 0
        print(f"Timestamps: {timestamps}") 
        for i in range(1, len(timestamps)):
            # Time calculations
            Ts = (timestamps[i] - self.prev_timestamp) * 1e-6  # Convert to seconds

            # Handle timestamp overflow or invalid sampling time
            if Ts <= 0 or Ts > 0.5:
                Ts = 1e-3
            
            # Time from last pulse
            Th = (timestamps[i] - timestamps[i-1]) * 1e-6

            # Pulse count difference
            dN = pulse_counts[i] - self.prev_pulse_counter
            
            # SimpleFOC velocity calculation
            dt = Ts + self.prev_Th - Th
            
            if dN != 0 and dt > Ts/2:
                self.pulse_per_second = dN / dt
            
            # Zero velocity detection
            if Th > 0.1:
                self.pulse_per_second = 0
                
            # Calculate velocity in rad/s
            velocity = self.pulse_per_second  * ( np.pi / cpr) * -1
            velocity = velocity * 0.3 + prev_velocity * 0.7
            if(abs(velocity) < 0.001): 
                count += 1
                print(f"Zero velocity detected at {timestamps[i] / 1000} ms velocity: {velocity}, counter: {count} corrected velocity: {prev_velocity}")
                velocity = prev_velocity
            prev_velocity = velocity
            velocities.append(velocity)
            
            # Update state variables
            self.prev_timestamp = timestamps[i]
            self.prev_Th = Th
            self.prev_pulse_counter = pulse_counts[i]
            
        return np.array(velocities)

def load_and_process_data(filename):
    """
    Load data from CSV and process it through all filters
    Parameters:
        filename (str): Path to the CSV file
    """
    # Load data
    filename = filename + '.csv'
    df = pd.read_csv(filename)

    # Store the original timestamps for plotting
    timestamps = df['timestamp'].values
    velocities = df['velocity'].values

    # Initialize filters
    filters = VelocityFilters()

    # Apply filters and ensure they all have the same length
    butterworth_filtered = filters.butterworth_filter(velocities)
    ema_filtered = filters.exponential_moving_average(velocities)
    ma_filtered = filters.moving_average(velocities)
    simplefoc_filtered = filters.simplefoc_filter(
        timestamps,
        df['count'].values,
        cpr=512
    )

    # Pad SimpleFOC output if needed (it might be one element shorter)
    if len(simplefoc_filtered) < len(timestamps):
        simplefoc_filtered = np.append(simplefoc_filtered, simplefoc_filtered[-1])

    # Print lengths to verify
    print(f"Original data length: {len(timestamps)}")
    print(f"Butterworth length: {len(butterworth_filtered)}")
    print(f"EMA length: {len(ema_filtered)}")
    print(f"MA length: {len(ma_filtered)}")
    print(f"SimpleFOC length: {len(simplefoc_filtered)}")

    # Plot results
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(15, 12))
    fig.tight_layout(pad=3.0)

    # First subplot with all filters
    ax1.plot(timestamps[1:], velocities[1:], linewidth=2, color='#2E86C1', label='Raw Velocity')
    ax1.plot(timestamps[1:], butterworth_filtered[1:], linewidth=2, color='#E74C3C', label='Butterworth')
    ax1.plot(timestamps[1:], ema_filtered[1:], linewidth=2, color='#27AE60', label='EMA')
    ax1.plot(timestamps[1:], ma_filtered[1:], linewidth=2, color='#8E44AD', label='Moving')

    ax1.set_title('Velocity Filters Comparison')
    ax1.set_xlabel('Time (μs)')
    ax1.set_ylabel('Velocity (rad/s)')
    ax1.grid(True)
    ax1.legend()

    # Second subplot with selected filters
    ax2.plot(timestamps, velocities, linewidth=2, color='#2E86C1', label='Raw Velocity')
    ax2.plot(timestamps, ema_filtered, linewidth=2, color='#E74C3C', label='EMA')
    ax2.plot(timestamps, ma_filtered, linewidth=2, color='#27AE60', label='Moving Average')
    ax2.set_title('Selected Filters Comparison')
    ax2.set_xlabel('Time (μs)')
    ax2.set_ylabel('Velocity (rad/s)')
    ax2.grid(True)
    ax2.legend()

    
    # Third subplot with SimpleFOC filter
    # In the load_and_process_data function, for the third subplot:
    ax3.plot(timestamps, velocities, linewidth=2, color='#2E86C1', label='Raw Velocity')
    ax3.plot(timestamps[1:], simplefoc_filtered[1:], linewidth=2, color='#8E44AD', label='SimpleFOC')

    # Add ylim to match the range of your raw velocity data
    ax3.set_ylim([min(velocities) * 1.2, max(velocities) * 1.2])
    ax3.set_title('SimpleFOC Filter')
    ax3.set_xlabel('Time (μs)')
    ax3.set_ylabel('Velocity (rad/s)')
    ax3.grid(True)
    ax3.legend()

    # save to to csv 
    csv_filename = filename.split('/')[-1].replace('.csv', '_filtered.csv')
    with open('log/filters/' + csv_filename, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(['timestamp', 'velocity', 'butterworth', 'ema', 'ma', 'simplefoc'])
        for i in range(len(timestamps)):
            writer.writerow([timestamps[i], velocities[i], butterworth_filtered[i], ema_filtered[i], ma_filtered[i], simplefoc_filtered[i]])

    save_path = 'log/filters/' + filename.split('/')[-1].replace('.csv', '.png')
    plt.savefig(save_path, dpi=300, bbox_inches='tight')
    plt.show()

# Example usage
if __name__ == "__main__":
    load_and_process_data('log/step_responses/csv/r_1')  