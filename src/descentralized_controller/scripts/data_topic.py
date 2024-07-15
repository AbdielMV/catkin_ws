#!/usr/bin/env python
# -*- coding: utf-8 -*-
import numpy as np
from scipy.interpolate import interp1d
import matplotlib.pyplot as plt

# Crear el intervalo de tiempo
t = np.arange(0, 30, 0.001)

# Valores de y correspondientes (de 0 a 45)
y = np.linspace(0, 45, len(t))

# Crear la interpolación cúbica
cubic_interp = interp1d(t, y, kind='cubic')

# Evaluar la spline en el intervalo de tiempo
y_spline = cubic_interp(t)

# Graficar la spline cúbica
plt.plot(t, y_spline, label='Spline cúbica')
plt.xlabel('Tiempo')
plt.ylabel('Valor de y')
plt.title('Spline Cúbica de y vs. Tiempo')
