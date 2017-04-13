#!/usr/bin/python3
"""
Created on Thu Nov 17 18:42:45 2016

@author: mguski
"""

# %%
from datetime import datetime
import matplotlib.pyplot as plt

# %% class stuff

class logEntries:
    def __init__(self,fileName=None):
        self.time  = []
        self.name  = []
        self.level = []
        self.msg   = []
        self.fileName = fileName
        if self.fileName != None:
            self.parseFile()
        
    def parseFile(self):
        with open(self.fileName) as f:
            for line in f:
                dateStr = line[:23]
                self.time.append(datetime.strptime(dateStr, '%Y-%m-%d %H:%M:%S,%f'))
                self.name.append(line[24:37].strip())
                self.level.append(line[37:46].strip())
                self.msg.append(line[46:-1].strip())
                        
    def copy(self):
        output = logEntries()
        output.time  = self.time
        output.name  = self.name
        output.level = self.level
        output.msg   = self.msg
        return output
    
    def remove(self, idx):
        del self.time[idx]
        del self.name[idx]
        del self.level[idx]
        del self.msg[idx]

class timeSpans():
    def __init__(self, logData):
        self.logData = logData.copy()
        self.startTime  = []
        self.stopTime   = []
        self.startLevel = []
        self.stopLevel  = []
        self.startName  = []
        self.stopName   = []
        self.msg        = []

        self.scanLogData()
        
    def scanLogData(self):
        data = self.logData.copy()
        currStartIdx = 0
        print('number of entries: {}'.format(len(data.msg)))
        while (currStartIdx < len(data.msg)):
            if data.msg[currStartIdx][:6] == "start ":
                currStopIdx = currStartIdx +1
                
                while (currStopIdx < len(data.msg)):
                    if data.msg[currStopIdx][:4] == "end ":
                        if data.msg[currStartIdx][6:].strip() == data.msg[currStopIdx][4:].strip():
                            self.startTime.append(data.time[currStartIdx])
                            self.stopTime.append(data.time[currStopIdx])
                            self.startLevel.append(data.level[currStartIdx])
                            self.stopLevel.append(data.level[currStopIdx])
                            self.startName.append(data.name[currStartIdx])
                            self.stopName.append(data.name[currStopIdx])
                            self.msg.append(data.msg[currStopIdx][4:].strip())  
                            data.remove(currStartIdx)
                            data.remove(currStopIdx-1)
                            currStartIdx -=1 
                            break
                    currStopIdx += 1
                currStartIdx +=1                
            else:
                currStartIdx +=1
        print('number pairs found: {}\n number remaining entries: {}'.format(len(self.msg), len(data.msg)))
        
        for idx in range(len(data.msg)):
            print(data.msg[idx])
        self.remainingLogEntries = data
        
        
# %% parse log files

server = logEntries('../log/server.log')          
timSpansServer = timeSpans(server)

# %% plot data

fig, ax = plt.subplots()
uniqueNames = list(set(timSpansServer.startName))
uniqueMessages = list(set(timSpansServer.msg))

t0 = timSpansServer.startTime[0]

for idx in range(len(timSpansServer.startTime)):

    # nameIdx = uniqueNames.index(timSpansServer.startName[idx])
    nameIdx = uniqueMessages.index(timSpansServer.msg[idx])

    startSec = (timSpansServer.startTime[idx] - t0).total_seconds()
    duration = (timSpansServer.stopTime[idx] - timSpansServer.startTime[idx]).total_seconds()
    ax.barh(nameIdx, duration, left= startSec, align='center')
    ax.text(startSec+duration/2, nameIdx, "{} ms".format(duration*1000), rotation=45, backgroundcolor='gray', alpha=0.5, ha='center', va='center' )

plt.xlabel('Time in s')

plt.yticks(range(len(uniqueMessages)), uniqueMessages)
ax.axis([0,20, -0.5,len(uniqueMessages)-0.5])

plt.grid(True)
plt.show()    


