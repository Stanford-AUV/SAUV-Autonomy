import re
import matplotlib.pyplot as plt
import datetime
import mplcursors

# Initialize lists to store data
timestamps = []
position_errors = []
yaw_errors = []

# Define regex patterns to extract relevant data
timestamp_pattern = re.compile(r'\[INFO\] \[(\d+\.\d+)\]')
position_error_pattern = re.compile(r'Position Error: ([\d\.]+)')
yaw_error_pattern = re.compile(r'Yaw Error: ([\d\.]+)')

# Open and read the log file
with open('SEMI_sim_untethered.log', 'r') as file:
    for line in file:
        # Extract timestamp
        timestamp_match = re.search(timestamp_pattern, line)
        if timestamp_match:
            timestamp = float(timestamp_match.group(1))
            # Convert UNIX timestamp to datetime for better readability
            timestamp = datetime.datetime.fromtimestamp(timestamp)
            timestamps.append(timestamp)
        
        # Extract position error
        position_error_match = re.search(position_error_pattern, line)
        if position_error_match and len(timestamps) > len(position_errors):
            position_error = float(position_error_match.group(1))
            position_errors.append(position_error)
        
        # Extract yaw error
        yaw_error_match = re.search(yaw_error_pattern, line)
        if yaw_error_match and len(timestamps) > len(yaw_errors):
            yaw_error = float(yaw_error_match.group(1))
            yaw_errors.append(yaw_error)

# Ensure all lists have the same length
min_length = min(len(timestamps), len(position_errors), len(yaw_errors))
timestamps = timestamps[:min_length]
position_errors = position_errors[:min_length]
yaw_errors = yaw_errors[:min_length]

# Print lengths for verification
print(f"Number of timestamps: {len(timestamps)}")
print(f"Number of position errors: {len(position_errors)}")
print(f"Number of yaw errors: {len(yaw_errors)}")

# Plotting the data
plt.figure(figsize=(14, 8))

# Plot position error
plt.subplot(2, 1, 1)
position_plot, = plt.plot(timestamps, position_errors, label='Position Error', color='blue')
plt.xlabel('Time')
plt.ylabel('Position Error')
plt.title('Position Error Over Time')
plt.grid(True)

# Plot yaw error
plt.subplot(2, 1, 2)
yaw_plot, = plt.plot(timestamps, yaw_errors, label='Yaw Error', color='orange')
plt.xlabel('Time')
plt.ylabel('Yaw Error')
plt.title('Yaw Error Over Time')
plt.grid(True)

# Adjust layout for better spacing
plt.tight_layout()

# Enable interactive cursor for position and yaw errors
mplcursors.cursor(position_plot).connect("add", lambda sel: sel.annotation.set_text(f"Time: {timestamps[sel.target.index]}\nPos Error: {position_errors[sel.target.index]}"))
mplcursors.cursor(yaw_plot).connect("add", lambda sel: sel.annotation.set_text(f"Time: {timestamps[sel.target.index]}\nYaw Error: {yaw_errors[sel.target.index]}"))

# Show the plots
plt.show()
