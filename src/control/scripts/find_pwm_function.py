import numpy as np
import json
import matplotlib.pyplot as plt
from scipy.optimize import curve_fit
import pandas as pd

# COEFFS = {Voltage (int): Eq_coeffs (list of len 2)}
# Eq_coeffs = [[a_pos, b_pos], [a_neg, b_neg]] of 'a * sqrt(x) + b' for pos and neg thrusts
COEFFS = {10: [], 12: [], 14: [], 16: [], 18: [], 20: []}


def quadratic_model(x, a, b):
    return a * np.square(x - b)


def inverse_quadratic_model(x, a, b):
    second_part = np.sqrt(np.abs(x)) / np.sqrt(a)
    return np.where(x >= 0, b + second_part, b - second_part)


def find_coeffs(voltage, plot=False):
    # Load CSV
    filename = str(voltage) + "V.csv"
    df = pd.read_csv(filename)
    x = np.array([float(val) for val in df[" PWM (µs)"].values.tolist()])
    y = np.array([float(val) for val in df[" Force (Kg f)"].values.tolist()])

    # Piecewise fit quadratic model with a and b positive
    popt, _ = curve_fit(quadratic_model, x, np.abs(y), bounds=(0, [np.inf, np.inf]))

    if plot:
        x_plot = np.linspace(min(x), max(x), 200)
        y_plot = quadratic_model(x_plot, *popt)
        # Plot the original data points
        plt.scatter(x, np.abs(y), color="red", label="Data Points")
        # Plot sqrt approx
        plt.plot(x_plot, y_plot, color="blue")
        plt.xlabel("PWM (µs)")
        plt.ylabel("Force (Kg f)")
        title = "Quadratic Approx"
        plt.title(title)
        plt.legend()
        plt.show()

        x_plot = np.linspace(min(y), max(y), 200)
        y_plot = inverse_quadratic_model(x_plot, *popt)
        # Plot the original data points flipped
        plt.scatter(y, x, color="red", label="Data Points")
        # Plot sqrt approx
        plt.plot(x_plot, y_plot, color="blue")
        plt.xlabel("Force (Kg f)")
        plt.ylabel("PWM (µs)")
        title = "Inverse Quadratic Approx"
        plt.title(title)
        plt.legend()
        plt.show()

    return popt


def main():
    # Find coeffs for each voltage
    for volt, _ in COEFFS.items():
        COEFFS[volt] = tuple(find_coeffs(volt, plot=True))

    # Store coeffs in JSON file
    with open("../coeffs.json", "w") as f:
        json.dump(COEFFS, f)

    print(COEFFS)


if __name__ == "__main__":
    main()
