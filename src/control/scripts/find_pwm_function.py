import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

# Step 1: Load x and y from a CSV file
# Replace 'your_data.csv' with the path to your CSV file
# Assuming the CSV file has no headers and two columns: the first for x and the second for y
df = pd.read_csv("14 V-Table 1.csv", header=None)
y = np.array([float(val) for val in df[0].values.tolist()])
x = np.array([float(val) for val in df[1].values.tolist()])

print(x)
print(y)

# Step 2: Use numpy.polyfit with a degree of 2 for quadratic approximation
# The function returns the coefficients of the polynomial
coefficients = np.polyfit(x, y, 3)

# Step 3: Create a polynomial function from the coefficients
quadratic_function = np.poly1d(coefficients)

# Optional: Print the quadratic function
print(f"The best quadratic approximation is: \n{quadratic_function}")

# Optional: Plotting
# Generate a series of x values for plotting the quadratic approximation
x_plot = np.linspace(min(x), max(x), 400)
# Calculate the y values based on the quadratic approximation
# y_plot = quadratic_function(x_plot)


def new_function(x):
    return np.sign(x) * np.sqrt(30000 * np.abs(x)) + 1500


# def new_function(x):
#     return np.cbrt(7000000 * x) + 1500


y_plot = new_function(x_plot)

# Plot the original data points
plt.scatter(x, y, color="red", label="Data Points")
# Plot the quadratic approximation
plt.plot(x_plot, y_plot, color="blue", label="Quadratic Approximation")
plt.xlabel("x")
plt.ylabel("y")
plt.title("Quadratic Approximation of Data Points")
plt.legend()
plt.show()
