import subprocess
import re
import matplotlib
matplotlib.rcParams['text.usetex'] = True
import matplotlib.pyplot as plt
import numpy as np


fig, ax = plt.subplots()
ax.set_title(r'popSize=1000, gridSize=100, $d=0.8$, startingCases=10, $\beta = 0.6$ \
            $\gamma \in [0.001, 0.06]$ with step=0.002, estimatesPerPoint=10')
#ax.set_xlabel(r'$\beta$')
#ax.set_ylabel(r'$k = frac{ \beta }{ \gamma } $')
ax.set_xlabel(r'$k = \beta / \gamma $')
ax.set_ylabel('final number of recovered')

fil = open("./data/SAVErecoveredPerGamma.csv", 'r')
infectionRate = 0.6
lines = fil.readlines()
recoveryRates = []
susceptibles = []
for line in lines:
    line = line.strip().split(",")
    susceptibles.append(float(line[0]))
    recoveryRates.append(float(line[1]))

recoveryRates = np.array(recoveryRates)
ks = infectionRate / recoveryRates
susceptibles = np.array(susceptibles)
ax.plot(ks, susceptibles, color='green')

fil.close()



plt.savefig('beta-gama_surface.png')
plt.show()
