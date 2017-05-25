#!/usr/bin/python3
# 
# Function to show recorded diagnostic samples of control program
#
# usage:
# 
#  ./rawGUI.sh                     : load newest file in /data/diagnostic_samples/
#  ./rawGUI.sh bla.txt.a           : look for this file in /data/diagnostic_samples/
#  ./rawGUI.sh /a/path/bla.txt.a   : take this file


import sys
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import gridspec

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
        gs = gridspec.GridSpec(10,1)
        self.ax    = self.fgh.add_subplot(gs[0:8,:])
        self.axTxt = self.fgh.add_subplot(gs[9,:])
        self.axTxt.axis("off")   
        self.cid       = self.fgh.canvas.mpl_connect('key_press_event', self.keyCallback)
        self.updateGUI()
        print('Shortcuts:\n  left / right : previous / next sequence \n  up / down    : previous / next channel')
        mng = plt.get_current_fig_manager()
        mng.full_screen_toggle()
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

            while (currLine.find("Sequence: END") == -1):
                currLine = f.readline()
                if currLine == "":
                    break
                self.allHeaders[iDataSet].append(currLine)                     

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
        self.axTxt.cla()
        self.axTxt.axis("off")
        self.ax.plot(self.allDataSets[self.currentDataset][self.channelNames[self.iChannel]], marker='x')
        self.ax.set_xlim((0,self.allDataSets[self.currentDataset][self.channelNames[self.iChannel]].shape[0]))

        self.ax.set_title(self.inputFile)
        self.ax.set_xlabel("Samples")
        self.ax.set_ylabel(self.channelNames[self.iChannel] )
        self.axTxt.set_title('Dataset {} / {}'.format(self.currentDataset, self.nDatasets))
        self.axTxt.text(0, 1, "".join(self.allHeaders[self.currentDataset]), verticalalignment='top')
        self.ax.grid(True)
        self.ax.set_xlim((0,self.allDataSets[self.currentDataset][self.channelNames[self.iChannel]].shape[0]))
        plt.draw()


if len(sys.argv) == 1:
    import glob
    import os
    print('No input. Looking for newest file in /data/diagnostic_samples/ ...')    
    fileName = max(glob.iglob('/data/diagnostic_samples/*.txt.*'), key=os.path.getctime)
    print('found :{}'.format(fileName[25:]))
else:
    fileName = sys.argv[1]
    if fileName.find("/") == -1: # no path defined => assume /data/diagnostic_samples/
        fileName = "/data/diagnostic_samples/" + fileName

# read file and open GUI
RawDataGUI(fileName)






