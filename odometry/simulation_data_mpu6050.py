import pandas as pd

# file = "stopped"
file = "back"
# file = "front"
# file = "left"
# file = "right"

path = "dados_acelerometro/{}.csv".format(file)
columns = ["Gx", "Gy", "Gz", "Ax", "Ay", "Az"]
mpu_data = pd.read_csv(path, sep=";", header=None, names=columns)

# Simulating continuous incoming data from sensor...
for index, row in mpu_data.iterrows():
    print("Az:{:.5f} Gz:{:.5f}".format(row.Az, row.Gz))
    
