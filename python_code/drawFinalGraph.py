import subprocess
import re
import matplotlib
matplotlib.rcParams['text.usetex'] = True
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

child = subprocess.Popen(['ls', './data'], stdout=subprocess.PIPE, stderr=subprocess.STDOUT)

files_in_datadir = child.stdout.read().decode('utf-8').split("\n")
regex = re.compile("SAVErecoveredPerGammaWithBeta.*")

dataFileNames = []

for  fil in files_in_datadir:
    rez = regex.search(fil)
    if rez != None:
        dataFileNames.append(rez.group())



fig = plt.figure()
ax = fig.gca(projection='3d')
ax.set_title('popSize=1000, gridSize=100, $d=0.8$, startingCases=10, \
            $\gamma \in [0.001, 0.1]$ with step=0.002, estimatesPerPoint=10')
ax.set_xlabel(r'$\beta$')
#ax.set_ylabel(r'$k = frac{ \beta }{ \gamma } $')
ax.set_ylabel(r'$k = \beta / \gamma $')
ax.set_zlabel('final number of recovered')

for dataFileName in dataFileNames:
    fil = open("./data/" + dataFileName, 'r')
    lines = fil.readlines()
    infectionRates = []
    recoveryRates = []
    susceptibles = []
    for line in lines:
        line = line.strip().split(",")
        infectionRates.append(float(line[0]))
        susceptibles.append(float(line[1]))
        recoveryRates.append(float(line[2]))

    infectionRates = np.array(infectionRates)
    recoveryRates = np.array(recoveryRates)
    recoveryRates = infectionRates[0] / recoveryRates
    susceptibles = np.array(susceptibles)
    ax.plot(infectionRates, recoveryRates, susceptibles, color='green')

    fil.close()



plt.savefig('beta-gama_surface.png')
plt.show()
