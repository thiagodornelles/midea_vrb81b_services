import pandas as pd

# file = "stopped"
file = "back"
# file = "front"
# file = "left"
# file = "right"

path = "/content/drive/MyDrive/dados_acelerometro/{}.csv".format(file)
data = pd.read_csv(path, sep=";", header=None)
data = data.rename(columns={0: "Gx", 1: "Gy", 2: "Gz", 3: "Ax", 4: "Ay", 5: "Az"})
