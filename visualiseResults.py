import matplotlib.pyplot as plt
data = open(r"C:\Coding\lqr_test\own\lqr_results.txt")
l1 = data.readline();
l2 = data.readline();
l3 = data.readline();
dt = float(l1.split("=")[1])
target = float(l2.split("=")[1])
init = float(l3.split("=")[1])
times = []
pos = []
for line in data:
    times.append(float(line.split(" ")[0])*dt)
    pos.append(float(line.split(" ")[1]))
plt.plot(times,pos)
plt.grid(True)
plt.title("LQR Controller")
plt.xlabel("Time (seconds)")
plt.ylabel("Offset")
plt.show()