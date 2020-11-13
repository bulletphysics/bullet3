import pandas as pd
import numpy as np
import matplotlib
import matplotlib.pyplot as plt

content = []

with open("output/agent0_log.txt", "r") as f:
    content = f.readlines()

for i in range(len(content)):
    content[i] = ' '.join(content[i].split())
    content[i] = content[i].split(" ")

header = content[0]
df = pd.DataFrame(content[1:], columns=header, dtype=float)

tr_vals = df["Train_Return"].values
te_vals = df["Test_Return"].values
x = range(0, len(tr_vals)*400, 400)

fig, ax = plt.subplots()
ax.plot(x, tr_vals, label="Train")
ax.plot(x, te_vals, label="Test")
ax.set_xlim(0, (len(tr_vals)+1)*400)
ax.set_xlabel('Iterations')
ax.set_ylim(0, 100, 10)
ax.set_ylabel('Return')
ax.legend()
plt.savefig("output/rewards_log.png")
