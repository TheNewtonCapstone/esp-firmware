import pandas as pd
import numpy as np
from scipy import stats
import matplotlib.pyplot as plt
from scipy import signal

def load_and_prepare_data(filepath):
    """
    Load the CSV file and prepare the data for analysis.
    Returns the DataFrame and the sampling frequency.
    """
    # Read the CSV file
    df = pd.read_csv(filepath)
    
    # Calculate sampling frequency based on timestamps
    time_diff = np.diff(df['timestamp']).mean()
    sampling_freq = 1 / (time_diff / 1000)  # Converting to Hz assuming timestamps are in ms
    
    return df, sampling_freq

def calculate_filter_statistics(df):
    """
    Calculate basic statistics for each filter.
    """
    filter_columns = ['butterworth', 'ema', 'ma', 'simplefoc']
    stats_dict = {}
    
    for filter_name in filter_columns:
        stats_dict[filter_name] = {
            'mean': df[filter_name].mean(),
            'std': df[filter_name].std(),
            'median': df[filter_name].median(),
            'max': df[filter_name].max(),
            'min': df[filter_name].min(),
            'lag': calculate_lag(df['velocity'], df[filter_name])
        }
    
    return pd.DataFrame(stats_dict).round(4)

def calculate_lag(original, filtered):
    """
    Calculate the lag between original and filtered signals using cross-correlation.
    """
    correlation = signal.correlate(original - original.mean(), 
                                 filtered - filtered.mean(), 
                                 mode='full')
    lags = signal.correlation_lags(len(original), len(filtered))
    lag = lags[np.argmax(correlation)]
    return lag

def calculate_frequency_response(signal_data, sampling_freq):
    """
    Calculate the frequency response of a signal using FFT.
    """
    n = len(signal_data)
    fft_result = np.fft.fft(signal_data)
    frequencies = np.fft.fftfreq(n, 1/sampling_freq)
    
    # Get positive frequencies only
    pos_freq_idx = frequencies >= 0
    frequencies = frequencies[pos_freq_idx]
    magnitude = np.abs(fft_result)[pos_freq_idx]
    
    return frequencies, magnitude

def plot_time_domain_comparison(df):
    """
    Create a time domain comparison plot of all filters.
    """
    plt.figure(figsize=(15, 10))
    
    # Plot original velocity
    plt.plot(df['timestamp'], df['velocity'], 'gray', alpha=0.5, label='Original')
    
    # Plot filtered signals
    plt.plot(df['timestamp'], df['butterworth'], label='Butterworth')
    plt.plot(df['timestamp'], df['ema'], label='EMA')
    plt.plot(df['timestamp'], df['ma'], label='MA')
    plt.plot(df['timestamp'], df['simplefoc'], label='SimpleFOC')
    
    plt.xlabel('Time (ms)')
    plt.ylabel('Velocity')
    plt.title('Time Domain Comparison of Different Filters')
    plt.legend()
    plt.grid(True)
    plt.savefig('time_domain_comparison.png')
    plt.close()

def plot_frequency_response_comparison(df, sampling_freq):
    """
    Create a frequency response comparison plot of all filters.
    """
    plt.figure(figsize=(15, 10))
    
    for column in ['velocity', 'butterworth', 'ema', 'ma', 'simplefoc']:
        freq, mag = calculate_frequency_response(df[column], sampling_freq)
        plt.semilogx(freq[1:], mag[1:], label=column)
    
    plt.xlabel('Frequency (Hz)')
    plt.ylabel('Magnitude')
    plt.title('Frequency Response Comparison')
    plt.legend()
    plt.grid(True)
    plt.savefig('frequency_response_comparison.png')
    plt.close()

def analyze_noise_reduction(df):
    """
    Calculate noise reduction metrics for each filter.
    """
    original_std = df['velocity'].std()
    noise_reduction = {}
    
    for filter_name in ['butterworth', 'ema', 'ma', 'simplefoc']:
        filtered_std = df[filter_name].std()
        noise_reduction[filter_name] = {
            'noise_reduction_percent': ((original_std - filtered_std) / original_std * 100),
            'signal_smoothness': 1 / np.mean(np.abs(np.diff(df[filter_name])))
        }
    
    return pd.DataFrame(noise_reduction).round(4)

def main():
    # Load and analyze the data
    filepath = 'log/filters/'
    filename = filepath + 'r_7_filtered.csv'
    df, sampling_freq = load_and_prepare_data(filename)
    
    # Calculate and display basic statistics
    print("\nFilter Statistics:")
    print(calculate_filter_statistics(df))
    
    # Calculate and display noise reduction metrics
    print("\nNoise Reduction Analysis:")
    print(analyze_noise_reduction(df))
    
    # Create visualization plots
    plot_time_domain_comparison(df)
    plot_frequency_response_comparison(df, sampling_freq)
    
    # Additional analysis: Calculate and display filter latency
    print("\nFilter Response Time Analysis:")
    response_times = {}
    threshold = df['velocity'].std() * 0.1  # 10% of signal standard deviation
    
    for filter_name in ['butterworth', 'ema', 'ma', 'simplefoc']:
        error = np.abs(df[filter_name] - df['velocity'])
        response_time = np.mean(error < threshold)
        response_times[filter_name] = response_time
    
    print(pd.Series(response_times).round(4))

if __name__ == "__main__":
    main()