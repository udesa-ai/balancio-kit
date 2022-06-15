import numpy as np
import socket
import matplotlib.pyplot as plt
import pickle

HOST = "0.0.0.0"  # Standard loopback interface address (localhost)
PORT = 8090  # Port to listen on (non-privileged ports are > 1023)

data_hist = ""

# with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
with socket.socket() as s:
    s.bind((HOST, PORT))
    s.listen(0)
    conn, addr = s.accept()
    with conn:
        print(f"Connected by {addr}")
        # for _ in range(1000):
        while True:
            data = conn.recv(1024)
            if not data:
                break
            decoded_data = data.decode().strip()
            # print(decoded_data) 
            data_hist += decoded_data

print("Connection closed.")
# print(data_hist)

# Data Processing
print("Processing data...")
current_angle = []
target_angle = []
pwm = []
separated_data = data_hist.split("|")[1:]

print("Processing current_angle...")
for val in separated_data[0].split(",")[1:]:
    current_angle += [float(val)]

print("Processing target_angle...")
for val in separated_data[1].split(",")[1:]:
    target_angle += [float(val)]

print("Processing pwm...")
for val in separated_data[2].split(",")[1:]:
    pwm += [float(val)]

data = {
    "current_angle": current_angle,
    "target_angle": target_angle,
    "pwm": pwm
}

print("Saving data...")
with open("data.pkl", "wb") as f:
    pickle.dump(data, f, protocol=pickle.HIGHEST_PROTOCOL)

print("Plotting data...")
plt.plot(current_angle, label="Current Angle")
plt.plot(target_angle, label="Target Angle")
plt.xlabel("Time (s)")
plt.ylabel("Angle (rad)")
plt.legend()
plt.show()

print("Done.")
