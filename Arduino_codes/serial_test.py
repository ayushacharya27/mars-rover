import serial
import time

# Replace 'COMX' with your actual port (e.g., 'COM3' for Windows, '/dev/ttyUSB0' for Linux)
arduino_port = "/dev/ttyACM0"  
baud_rate = 9600

try:
    ser = serial.Serial(arduino_port, baud_rate, timeout=1)
    time.sleep(2)  # Allow time for Arduino to reset

    while True:
        command = input("Enter 'a' to arm, 'd' to disarm, 'q' to quit: ").strip().lower()

        if command == 'q':
            print("Exiting...")
            break

        if command in ['o', 'p']:
            ser.write(command.encode())  # Send command to Arduino
        else:
            print("Invalid input. Use 'a' or 'd'.")

except serial.SerialException as e:
    print(f"Error: {e}")
finally:
    if 'ser' in locals() and ser.is_open:
        ser.close()