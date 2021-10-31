import pandas as pd

# file = "stopped"
# file = "front"
# file = "left"
# file = "right"
file = "imu_data"

path = "/Users/thiago/Desktop/spiral_2/{}.csv".format(file)
columns = ["Gx", "Gy", "Gz", "Ax", "Ay", "Az"]
mpu_data = pd.read_csv(path, sep=";", header=None, names=columns)
print(mpu_data.shape)
# spiral - 43625 and 823 frames
# spiral_2 - 63980 lines and 611 frames
# Simulating continuous incoming data from sensor...
for index, row in mpu_data.iterrows():
    print("Az:{:.5f} Gz:{:.5f}".format(row.Az, row.Gz))
    
