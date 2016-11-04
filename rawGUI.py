#!/usr/bin/python3
import sys
import numpy as np
import matplotlib.pyplot as plt


class RawDataGUI:
    def __init__(self, inputFile):
        self.inputFile = inputFile
        self.allDataSets = []
        self.allHeaders  = []
        self.readFile()
        self.currentDataset = 0 
        self.iChannel = 3
        self.nDatasets = len(self.allDataSets)
        self.nChannels = len(self.channelNames)
        self.fgh       = plt.figure()
        self.ax        = plt.subplot(111)       
        self.cid       = self.fgh.canvas.mpl_connect('key_press_event', self.keyCallback)
        self.updateGUI()
        print('Shortcuts:\n  left / right : previous / next sequence \n  up / down    : previous / next channel')
        plt.show()
        
        
    def readFile(self):
        f = open(self.inputFile, 'r')

        iDataSet = 0
        currLine = f.readline()

        while (currLine != ""):
            self.allHeaders.append([])
            # firstLine = currLine
            while (currLine.find("nsamp") == -1):
                currLine = f.readline()
                if currLine == "":
                    break
                self.allHeaders[iDataSet].append(currLine)
        
            if currLine == "":
                break   
            nSamples = int(currLine[currLine.find("nsamp")+6:-1])
            
            currLine = f.readline()
            if currLine == "":
                break
            if iDataSet == 0:
                self.channelNames = currLine[:-1].split(" ")
            
            self.allDataSets.append( dict((key, np.zeros(nSamples))  for key in self.channelNames ))
            for iSample in range(nSamples):
                currLine = f.readline()
                if currLine == "":
                    break
                currStrValues = currLine[:-1].split()
                for iEntry,key in enumerate(self.channelNames):
                    self.allDataSets[iDataSet][key][iSample] = float(currStrValues[iEntry])
                    
            iDataSet += 1
        f.close()

    def keyCallback(self, event):
   #     print('you pressed', event.key, event.xdata, event.ydata)
        if event.key == 'right':
            self.currentDataset = (self.currentDataset+1) % self.nDatasets
            self.updateGUI()

        elif event.key == 'left':
            self.currentDataset = (self.currentDataset-1) % self.nDatasets
            self.updateGUI()
        elif event.key == 'up':
            self.iChannel = (self.iChannel+1) % self.nChannels
            self.updateGUI()
        elif event.key == 'down':
            self.iChannel = (self.iChannel-1) % self.nChannels
            self.updateGUI()
  #      print('{} / {}'.format(self.currentDataset, self.nDatasets))
        
    def updateGUI(self):
        self.ax.cla()
        self.ax.plot(self.allDataSets[self.currentDataset][self.channelNames[self.iChannel]], marker='x')
        self.ax.grid(True)
        self.ax.set_xlim((0,self.allDataSets[self.currentDataset][self.channelNames[self.iChannel]].shape[0]))
        plt.title('Sequence {} / {}       {}'.format(self.currentDataset, self.nDatasets, self.channelNames[self.iChannel]))
        plt.draw()


if len(sys.argv) == 1:
    import glob
    import os
    print('No input. Looking for newest file in /data/diagnostic_samples/ ...')    
    fileName = max(glob.iglob('/data/diagnostic_samples/*.txt.*'), key=os.path.getctime)
    print('found :{}'.format(fileName[25:]))
else:
    fileName = sys.argv[1]


# read file and open GUI
RawDataGUI(fileName)






