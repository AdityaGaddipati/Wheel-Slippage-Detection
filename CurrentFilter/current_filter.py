import numpy as np
import scipy.fftpack
from scipy.signal import butter, lfilter, lfilter_zi, freqz, filtfilt
import matplotlib.pyplot as plt
import csv
import rospy

# csv file name 
filename = "unfiltered_current.csv"

# initializing the titles and rows list 
fields = [] 
rows = []

Ileft = np.array([])
Iright = np.array([])

with open(filename, 'r') as csvfile:
	csvreader = csv.reader(csvfile)
	fields = csvreader.next()
	
	for row in csvreader:
		rows.append(row)
		Ileft = np.append(Ileft,float(row[5]))
		Iright = np.append(Iright,float(row[6]))

		#Ileft = np.append(Ileft,float(row[4]))
		#Iright = np.append(Iright,float(row[5]))

yf = scipy.fftpack.fft(Iright)

N = np.size(Iright)
T = 1.0/10
x = np.linspace(0.0, N*T, N)

xf = np.linspace(0.0, 1.0/(2.0*T), N/2)
#xf = np.linspace(0.0, 1.0, 61)

order = 6
fs = 10.0       # sample rate, Hz
cutoff = 0.5  # desired cutoff frequency of the filter, Hz

b, a = butter(order, cutoff/(0.5*fs), btype='low', analog=False)

Ileft_filtered1 = lfilter(b, a, Ileft)
Iright_filtered1 = lfilter(b, a, Iright)

Ileft_filtered2 = filtfilt(b, a, Ileft)
Iright_filtered2 = filtfilt(b, a, Iright)


plt.subplot(2, 2, 1)
plt.plot(x, Ileft)
plt.grid()
plt.subplot(2, 2, 3)
plt.plot(x, Ileft_filtered2)
plt.grid()
plt.subplot(2, 2, 2)
plt.plot(x, Iright)
plt.grid()
plt.subplot(2, 2, 4)
plt.plot(x, Iright_filtered2)
plt.grid()
plt.show()

'''
fig, ax = plt.subplots()
ax.plot(xf, 2.0/N * np.abs(yf[:N//2]))
#ax.plot(xf, 2.0/N * np.abs(yf[:61]))
#ax.plot(x, Ileft)
plt.grid()
plt.show()
'''

'''
w, h = freqz(b, a, worN=8000)
plt.subplot(2, 1, 1)
plt.plot(0.5*fs*w/np.pi, np.abs(h), 'b')
plt.plot(cutoff, 0.5*np.sqrt(2), 'ko')
plt.axvline(cutoff, color='k')
plt.xlim(0, 0.5*fs)
plt.title("Lowpass Filter Frequency Response")
plt.xlabel('Frequency [Hz]')
plt.grid()
plt.show()
'''
