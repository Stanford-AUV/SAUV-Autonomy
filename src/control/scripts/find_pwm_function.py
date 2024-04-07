import numpy as np
import json
import matplotlib.pyplot as plt
from scipy.optimize import curve_fit
import pandas as pd

# COEFFS = {Voltage (int): Eq_coeffs (list of len 2)}
# Eq_coeffs = [[a_pos, b_pos], [a_neg, b_neg]] of 'a * sqrt(x) + b' for pos and neg thrusts
COEFFS = {10 : [], 12 : [], 14: [], 16: [], 18: [], 20: []}

def find_coeffs(voltage, plot=False):
    
    # Load CSV
    filename = str(voltage) + "V.csv"
    df = pd.read_csv(filename)
    y = np.array([float(val) for val in df[' PWM (µs)'].values.tolist()])
    x = np.array([float(val) for val in df[' Force (Kg f)'].values.tolist()])
    idx_1500 = np.where(y == 1500)[0][0]

    def sqrt_model(x, a, b):
        return a * np.sqrt(x) + b

    # Piecewise fit sqrt curves
    popt_pos, _ = curve_fit(sqrt_model, x[idx_1500:], y[idx_1500:])
    popt_neg, _ = curve_fit(sqrt_model, -1 * x[:idx_1500], -1 * y[:idx_1500] + 2 * y[idx_1500])
    popt_neg[0] *= -1

    if plot:
        x_plot_pos = np.linspace(min(x[idx_1500:]), max(x[idx_1500:]), 200)
        x_plot_neg = np.linspace(min(x[:idx_1500]), max(x[:idx_1500]), 200)
        y_plot_pos = sqrt_model(x_plot_pos, *popt_pos)
        y_plot_neg = sqrt_model(-1 * x_plot_neg, *popt_neg)

        # Plot the original data points
        plt.scatter(x, y, color="red", label="Data Points")
        # Plot sqrt approx
        plt.plot(x_plot_pos, y_plot_pos, color="blue", label="Pos")
        plt.plot(x_plot_neg, y_plot_neg, color="green", label="Neg")
        plt.xlabel("x")
        plt.ylabel("y")
        title = "Sqrt Approx of " + str(voltage) + "V"
        plt.title(title)
        plt.legend()
        plt.show()

    return [popt_pos, popt_neg]

def main():
    # Find coeffs for each voltage
    for volt, _ in COEFFS.items():
        COEFFS[volt].extend(find_coeffs(volt, plot=True))
    
    # Convert to list
    for key in COEFFS:
        COEFFS[key] = [arr.tolist() for arr in COEFFS[key]]

    # Store coeffs in JSON file
    with open('../coeffs.json', 'w') as f:
        json.dump(COEFFS, f)

    print(COEFFS)

if __name__ == '__main__':
    main()