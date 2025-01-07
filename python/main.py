import serial
import csv

# Configuration
SERIAL_PORT = '/dev/ttyUSB0'  # Replace with your ESP32's serial port
BAUD_RATE = 115200
CSV_FILE = 'log/pid_responses/csv/r_1.csv'
MESSAGES_PER_FILE = 50


def main():
    # Open serial connection
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        print(f"Connected to {SERIAL_PORT} at {BAUD_RATE} baud.")
    except serial.SerialException as e:
        print(f"Error opening serial port: {e}")
        return

    messages = []
    message_count = 0

    try:
        while True:
            # Read a line from the serial port
            line = ser.readline().decode('utf-8').strip()
            if line:
                print(f"Received: {line}")
                messages.append(line.split(", "))  # Assuming data is comma-separated

                # Check if we reached the limit for saving
                if len(messages) >= MESSAGES_PER_FILE:
                    save_to_csv(messages)
                    messages.clear()  # Clear the buffer
                    message_count += 1
                    print(f"Saved batch {message_count} to CSV.")
    except KeyboardInterrupt:
        print("Terminating script.")
    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        ser.close()


def save_to_csv(data):
    # Append the data to the CSV file
    with open(CSV_FILE, mode='a', newline='') as file:
        writer = csv.writer(file)
        writer.writerows(data)


if __name__ == '__main__':
    main()
