import subprocess
import re
import matplotlib
#matplotlib.rcParams['text.usetex'] = True
import matplotlib.pyplot as plt
import numpy as np

child = subprocess.Popen(['ls', './data2'], stdout=subprocess.PIPE, stderr=subprocess.STDOUT)

files_in_datadir = child.stdout.read().decode('utf-8').split("\n")
regex = re.compile(".*activeSwimming.*0\.001.*rest.*")
#regex = re.compile(".*activeSwimming.*0\.01.*rest.*")
#regex = re.compile(".*activeSwimming.*0\.04.*rest.*")

dataFileNames = []

for  fil in files_in_datadir:
    rez = regex.search(fil)
    if rez != None:
        #print(rez.string)
        dataFileNames.append(rez.string)


fig, ax = plt.subplots()
ax.set_title('Active swimming')
ax.set_xlabel('coef')
#ax.set_ylabel(r'$k = frac{ \beta }{ \gamma } $')
ax.set_ylabel('number of delivered targets')

finalNOfTargets = []
coefs = []

for dataFileName in dataFileNames:
    #print(dataFileName)
    fil = open("./data2/" + dataFileName, 'r')
    lines = fil.readlines()
    targetPerTime = []
    check = False
    for line in lines:
        if "ni " in line:
            line = line.strip().split(" ")
            c = float(line[1])
            if c not in coefs:
                coefs.append(c)
            continue
        if "nOfCollectedItemsPerTime:" in line:
            check = True
            continue
        if check == False:
            continue
        line = line.strip().split(",")
        targetPerTime.append([int(line[0]), int(line[1])])
    finalNOfTargets.append(targetPerTime[-1][-1])

#    print(dataFileName)
#    print(targetPerTime)
    fil.close()

print(coefs)
print(finalNOfTargets)
#coefs = np.array(coefs)
#coefs = np.sum(coefs.reshape((15, coefs.size / 15)), axis=0)
finalNOfTargets = np.array(finalNOfTargets)
finalNOfTargets = np.sum(finalNOfTargets.reshape((15, int(finalNOfTargets.size / 15))), axis=0) \
                    / 15
print(coefs)
print(finalNOfTargets)
plt.plot(coefs, finalNOfTargets)

#plt.savefig('beta-gama_surface.png')
plt.show()
