import numpy as np
from scipy import signal
import matplotlib.pyplot as plt
from dataclasses import dataclass
from typing import Dict, Tuple, Optional


@dataclass
class PIDGains:
    """Store PID controller gains with optional derivative filter coefficient"""
    Kp: float
    Ki: float
    Kd: float
    N: float = 10.0  # Derivative filter coefficient


class PIDDesigner:
    """
    Design and tune PID controllers for the identified first-order system.
    Implements multiple tuning methods and performance validation.
    """

    def __init__(self, K: float, tau: float, dt: float = 0.001):
        """
        Initialize with system parameters.

        Args:
            K: System gain
            tau: System time constant
            dt: Sampling time
        """
        self.K = K
        self.tau = tau
        self.dt = dt

        # Create system transfer function
        self.sys = signal.TransferFunction([K], [tau, 1])
        self.sys_discrete = signal.TransferFunction([K], [tau, 1], dt=dt)

    def get_ziegler_nichols_gains(self) -> PIDGains:
        """
        Calculate PID gains using Ziegler-Nichols first method.
        For first-order systems, we'll use modified rules optimized for
        step response characteristics.

        Returns:
            PIDGains object containing the calculated gains
        """
        # Modified ZN rules for first-order systems
        Kp = 1.2 * self.tau / (self.K * self.dt)  # Aggressive
        Ti = 2.0 * self.tau  # Integral time
        Td = 0.5 * self.tau  # Derivative time

        return PIDGains(
            Kp=Kp,
            Ki=Kp / Ti,
            Kd=Kp * Td
        )

    def get_chien_hrones_reswick_gains(self) -> PIDGains:
        """
        Calculate PID gains using Chien-Hrones-Reswick method.
        This method tends to give more conservative gains than ZN.

        Returns:
            PIDGains object containing the calculated gains
        """
        # CHR parameters for 20% overshoot setpoint response
        Kp = 0.6 * self.tau / (self.K * self.dt)
        Ti = self.tau
        Td = 0.5 * self.tau

        return PIDGains(
            Kp=Kp,
            Ki=Kp / Ti,
            Kd=Kp * Td
        )

    def get_cohen_coon_gains(self) -> PIDGains:
        """
        Calculate PID gains using Cohen-Coon method.
        This method often provides good disturbance rejection.

        Returns:
            PIDGains object containing the calculated gains
        """
        # Cohen-Coon parameters
        Kp = (1.35 / self.K) * (self.tau / self.dt) + 0.27
        Ti = self.tau * (2.5 + 2.0 * self.dt / self.tau)
        Td = 0.37 * self.tau

        return PIDGains(
            Kp=Kp,
            Ki=Kp / Ti,
            Kd=Kp * Td
        )

    def simulate_response(self, gains: PIDGains,
                          t_final: float = 5.0,
                          setpoint: float = 1.0) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Simulate closed-loop system response with given PID gains.

        Args:
            gains: PID controller gains
            t_final: Simulation duration
            setpoint: Desired setpoint

        Returns:
            Tuple containing:
                - Time array
                - Output array
                - Control signal array
        """
        # Create time array
        t = np.arange(0, t_final, self.dt)

        # Initialize arrays
        y = np.zeros_like(t)
        u = np.zeros_like(t)
        e = np.zeros_like(t)
        e_int = 0
        e_prev = 0
        y_prev = 0

        # Discrete-time simulation
        for i in range(1, len(t)):
            # Error
            e[i] = setpoint - y[i - 1]

            # Integral term with anti-windup
            e_int += e[i] * self.dt
            e_int = np.clip(e_int, -100, 100)  # Anti-windup limits

            # Derivative term with filtering
            e_diff = (e[i] - e_prev) / self.dt
            e_diff = e_diff / (1 + gains.N * self.dt)  # First-order filter

            # PID control law
            u[i] = (gains.Kp * e[i] +
                    gains.Ki * e_int +
                    gains.Kd * e_diff)

            # Apply input saturation
            u[i] = np.clip(u[i], -1000, 1000)

            # System response (simple Euler integration)
            dy = (self.K * u[i] - y[i - 1]) / self.tau
            y[i] = y[i - 1] + dy * self.dt

            # Update previous values
            e_prev = e[i]
            y_prev = y[i]

        return t, y, u

    def evaluate_performance(self, t: np.ndarray, y: np.ndarray, u: np.ndarray,
                             setpoint: float = 1.0) -> Dict[str, float]:
        """
        Calculate performance metrics for the control response.

        Args:
            t: Time array
            y: Output array
            u: Control signal array
            setpoint: Desired setpoint

        Returns:
            Dict containing performance metrics
        """
        # Find settling time (within 2% of setpoint)
        settling_band = 0.02 * setpoint
        settled_idx = np.where(np.abs(y - setpoint) <= settling_band)[0]
        settling_time = t[settled_idx[0]] if len(settled_idx) > 0 else np.inf

        # Calculate overshoot
        overshoot = (np.max(y) - setpoint) / setpoint * 100 if np.max(y) > setpoint else 0

        # Calculate rise time (10% to 90%)
        rise_indices = np.where((y >= 0.1 * setpoint) & (y <= 0.9 * setpoint))[0]
        rise_time = t[rise_indices[-1]] - t[rise_indices[0]] if len(rise_indices) > 0 else np.inf

        # Calculate control effort
        control_effort = np.sum(np.abs(np.diff(u))) * self.dt

        # Calculate steady-state error
        ss_error = np.abs(setpoint - y[-1])

        return {
            'settling_time': settling_time,
            'overshoot': overshoot,
            'rise_time': rise_time,
            'control_effort': control_effort,
            'steady_state_error': ss_error
        }

    def plot_response(self, t: np.ndarray, y: np.ndarray, u: np.ndarray,
                      title: str = "PID Control Response",
                      setpoint: float = 1.0) -> None:
        """
        Plot system response and control signal.

        Args:
            t: Time array
            y: Output array
            u: Control signal array
            title: Plot title
            setpoint: Desired setpoint
        """
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8))
        fig.tight_layout(pad=3.0)

        # Plot system response
        ax1.plot(t, y, 'b-', label='Output')
        ax1.plot(t, np.full_like(t, setpoint), 'r--', label='Setpoint')
        ax1.set_xlabel('Time (s)')
        ax1.set_ylabel('Output')
        ax1.grid(True)
        ax1.legend()
        ax1.set_title(title)

        # Plot control signal
        ax2.plot(t, u, 'g-', label='Control Signal')
        ax2.set_xlabel('Time (s)')
        ax2.set_ylabel('Control Signal')
        ax2.grid(True)
        ax2.legend()

        plt.show()


def main():
    """
    Example usage of the PID designer.
    """
    # System parameters (from system identification)
    K = 0.5  # Gain
    tau = 0.3  # Time constant
    dt = 0.001  # Sampling time

    # Create PID designer
    designer = PIDDesigner(K, tau, dt)

    # Get gains from different methods
    zn_gains = designer.get_ziegler_nichols_gains()
    chr_gains = designer.get_chien_hrones_reswick_gains()
    cc_gains = designer.get_cohen_coon_gains()

    # Simulate and compare responses
    methods = {
        'Ziegler-Nichols': zn_gains,
        'Chien-Hrones-Reswick': chr_gains,
        'Cohen-Coon': cc_gains
    }

    for method_name, gains in methods.items():
        print(f"\nTesting {method_name} method:")
        print(f"Kp = {gains.Kp:.3f}, Ki = {gains.Ki:.3f}, Kd = {gains.Kd:.3f}")

        # Simulate response
        t, y, u = designer.simulate_response(gains)

        # Evaluate performance
        metrics = designer.evaluate_performance(t, y, u)
        print("\nPerformance metrics:")
        for metric, value in metrics.items():
            print(f"{metric}: {value:.3f}")

        # Plot response
        designer.plot_response(t, y, u, f"PID Response - {method_name}")


if __name__ == "__main__":
    main()