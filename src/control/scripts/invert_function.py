import sympy as sp
import numpy as np
import matplotlib.pyplot as plt

pwm = sp.symbols("pwm")
force = sp.symbols("force")

f = 3.23e-08 * pwm**3 - 0.0001421 * pwm**2 + 0.2139 * pwm - 110.1

inverse = sp.solve(f - force, pwm)
inverse_function = sp.lambdify(force, inverse)

# sp.plot(f, (pwm, 0, 2000))
x_values = np.linspace(0, 5)
y_values = np.array(inverse_function(x_values))
print(x_values.shape, y_values.shape)

plt.plot(x_values, y_values)
plt.xlabel("Force")
plt.ylabel("PWM")
plt.title("Force to PWM")


print(inverse)
