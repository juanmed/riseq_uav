import pandas as pd
import matplotlib.pyplot as plt

fig = plt.figure()
ax = fig.add_subplot(1,1,1)

#cpu = 'Intel Core i5-8400 CPU @ 2.80GHz x 6'
#gpu = 'GeForcuree GTX 1050 Ti/PCIe/SSE2'

cpu = 'ARMv8'
gpu = 'NVIDIA Tegra X1'

gpu_times = pd.read_csv("~/Documents/jetsongputimes2.csv")
gpu_times.columns = ['{} time (ms)'.format(gpu)]
gpu_times['{} time (ms)'.format(gpu)] = gpu_times['{} time (ms)'.format(gpu)]/1000. 
gpu_times.plot.hist(ax = ax, bins = 400, logx = True, logy=True, title = 'GPU vs CPU Times', color = 'r')

cpu_times = pd.read_csv("~/Documents/jetsoncputimes2.csv")
cpu_times.columns = ['{} time (ms)'.format(cpu)]
#cpu_times['time (ms)'] = cpu_times['time (ms)'] 
cpu_times.plot.hist(ax = ax, bins = 400, logx = True, logy=True, color = 'b')

pcpu_times = pd.read_csv("~/Documents/jetsonpytorchcputimes2.csv")
pcpu_times.columns = ['Pytorch on {} time (ms)'.format(cpu)]
pcpu_times['Pytorch on {} time (ms)'.format(cpu)] = pcpu_times['Pytorch on {} time (ms)'.format(cpu)]/1000. 
pcpu_times.plot.hist(ax = ax, bins = 400, logx = True, logy=True,  color = 'g', alpha = 0.5)


plt.show()
