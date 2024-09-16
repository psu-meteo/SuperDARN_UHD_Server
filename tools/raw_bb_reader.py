#!/usr/bin/python3
# function to plot raw bb samples exportet to disk by usrp_server

# -*- coding: utf-8 -*-
"""
Created on Sat Sep 23 17:13:42 2017

@author: mguski
"""

import os
import sys
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import gridspec
import matplotlib.cm as cm

# %% used hardcoded file and path
#raw_path = "/data/image_samples/bb_data/"
#fileName = '201709231930.1.iraw.d'
#file_with_path = os.path.join(raw_path, fileName)




# %% read file

def read_raw_file(file_with_path):
    #  read all data as int32
    rawFile = open(file_with_path , "rb")
    data = np.fromfile(rawFile, dtype=np.uint32)
    rawFile.close()
    
    
    
    # data structure:
    # HEADER for complete integration period
    # for each sequence:
    #   start time of sequnece (sec and usec)
    #   for each antenna:
    #     samples of this antenna for this sequnece
    #
    # and then comes the next integration period staring with HEADER
    
    all_periods = []
    curr_idx = 0
    while (curr_idx < len(data)):
        period_dict = {}
        period_dict['version'] = data[curr_idx]
        if period_dict['version'] != 3:
            print("Error: only Version 3 exports are supported!")
            return 0
        period_dict['year'] = data[curr_idx+1]
        period_dict['month'] = data[curr_idx+2]
        period_dict['day'] = data[curr_idx+3]
        period_dict['hour'] = data[curr_idx+4]
        period_dict['minute'] = data[curr_idx+5]
        period_dict['second'] = data[curr_idx+6]
        period_dict['microsecond'] = data[curr_idx+7]
        period_dict['nrang'] = data[curr_idx+8]
        period_dict['mpinc'] = data[curr_idx+9]
        period_dict['smsep'] = data[curr_idx+10]
        period_dict['lagfr'] = data[curr_idx+11]
        period_dict['pulseLength'] = data[curr_idx+12]
        period_dict['beam'] = data[curr_idx+13]
       
        period_dict['rfreq'] = data[curr_idx+14]
        period_dict['mppul'] = data[curr_idx+15]
        period_dict['ppat'] = data[curr_idx+16:curr_idx+16+period_dict['mppul']]
        curr_idx = curr_idx+16+period_dict['mppul']
       
        period_dict['nbaud'] = data[curr_idx]
        period_dict['pcode'] = data[curr_idx+1:curr_idx+period_dict['nbaud']+1]
        curr_idx += period_dict['nbaud']+1
       
        period_dict['nSamples'] = data[curr_idx]
        period_dict['nSeq'] = data[curr_idx+1]
        period_dict['nAntennas'] = data[curr_idx+2]
        period_dict['antenna_list']= data[curr_idx+3:curr_idx+3+period_dict['nAntennas']]
        curr_idx += period_dict['nAntennas']+3
        print("  Integration period: {} with {} sequences".format(len(all_periods)+1, period_dict['nSeq'] ))
        seq_list = []
        for iSeq in range(period_dict["nSeq"]):
            seq_dict = {}
            seq_dict["sequence_no_in_period"] = iSeq
            seq_dict["seq_start_time_sec"] = data[curr_idx]
            seq_dict["seq_start_time_usec"] = data[curr_idx+1]
            samples = []
            nSamples = period_dict['nSamples'] *2 # because we read as uint32 but one sample is complex64 (or two float32)
            for iAntenna in range(period_dict['nAntennas']):
               packed_data = data[curr_idx+2+iAntenna*nSamples:curr_idx+2+nSamples*(iAntenna+1) ]
               packed_data.dtype = "complex64"
               samples.append( packed_data )
            #   samples.append( np.int16(packed_data >> 16) + 1j* np.int16(packed_data % 2**16))
       
            seq_dict["samples"] = samples
            seq_list.append(seq_dict)
            curr_idx += 2+nSamples*period_dict['nAntennas']
         
        period_dict['seq_list'] = seq_list
        all_periods.append(period_dict)
        
    return all_periods



# %% plot GUI

class RawDataGUI:
    def __init__(self, inputFile):
        self.inputFile = inputFile
        self.data = read_raw_file(inputFile)
        self.iPeriod = 0 
        self.nPeriods = len(self.data)
        self.iSequence = "a"
        self.nSequences = (self.data[self.iPeriod]["nSeq"])
        
    def open(self):
        self.fgh       = plt.figure()        
        self.plot_db = False
        self.cid       = self.fgh.canvas.mpl_connect('key_press_event', self.keyCallback)
        self.updateGUI()
        print('\n\nShortcuts:\n  left / right : previous / next sequence \n  up / down    : previous / next period\n  d            : toggle lin / log plotting\n  a            : toggle all / one sequence')
     #   mng = plt.get_current_fig_manager()
     #   mng.full_screen_toggle()
        plt.show()
        
    def keyCallback(self, event):
       # print('you pressed', event.key, event.xdata, event.ydata)
        if event.key == 'right':
            if self.iSequence == "a":
                self.iSequence = 0
            else:
                self.iSequence = (self.iSequence+1) % self.nSequences
            self.updateGUI()

        elif event.key == 'left':
            if self.iSequence == "a":
                self.iSequence = self.nSequences-1
            else:
                self.iSequence = (self.iSequence-1) % self.nSequences
            self.updateGUI()
            
        elif event.key == 'a':
            if self.iSequence == "a":
                self.iSequence = 0
            else:
                self.iSequence = "a"
                
            self.updateGUI()            
            
            
        elif event.key == 'up':
            self.iPeriod = (self.iPeriod+1) % self.nPeriods
            self.nSequences = (self.data[self.iPeriod]["nSeq"])
            self.updateGUI()
        elif event.key == 'down':
            self.iPeriod = (self.iPeriod-1) % self.nPeriods
            self.nSequences = (self.data[self.iPeriod]["nSeq"])
            self.updateGUI()    
        elif event.key == 'd':
            self.plot_db = not self.plot_db
            self.updateGUI()    
        

            
    def updateGUI(self):
       # self.ax.cla()
       # self.axTxt.cla()
        self.fgh.clf()
        nAntennas = self.data[self.iPeriod]['nAntennas']
        nRows = int(np.ceil(nAntennas/2)+1)
        gs = gridspec.GridSpec(nRows, 2)
                 
        if self.iSequence == "a":
            plot_data = np.array(self.data[self.iPeriod]["seq_list"][0]['samples'])
            for iSequence in range(1,self.nSequences):
                plot_data = np.concatenate((plot_data,self.data[self.iPeriod]["seq_list"][iSequence]['samples']), axis=1)
            seq_text = "all {} ".format( self.nSequences)
        else:
            plot_data = np.array(self.data[self.iPeriod]["seq_list"][self.iSequence]['samples'])
            seq_text = "{}/{}".format(self.iSequence+1, self.nSequences)
        
        for iAntenna in range(nAntennas):
            ax  = self.fgh.add_subplot(gs[int(iAntenna/2),int(iAntenna%2)])
            yLabel = "ant {}".format(self.data[self.iPeriod]['antenna_list'][iAntenna])
            if iAntenna == 0:
               plt.title(self.inputFile)
            elif iAntenna == 1:
                plt.title('Period:{}/{}, Seq: {}'.format(self.iPeriod+1, self.nPeriods, seq_text))
                
            if self.plot_db:
                min_value = 0.5
                ref_value = 2**15

       #         plt.plot(20*np.log10(np.abs(np.real(self.data[self.iPeriod]["seq_list"][self.iSequence]['samples'][iAntenna])+min_value)/ref_value))
       #         plt.plot(20*np.log10(np.abs(np.imag(self.data[self.iPeriod]["seq_list"][self.iSequence]['samples'][iAntenna])+min_value)/ref_value))
                plot_values = 20*np.log10(np.abs(plot_data[iAntenna]+min_value)/ref_value)
                plt.plot(plot_values)

                ax.set_ylim([-92,10])
                yLabel += " (dB)"
            else:
                plt.plot(np.real(plot_data[iAntenna]))
                plt.plot(np.imag(plot_data[iAntenna]))
                
            plt.xlim([0, plot_data.shape[1]])
            plt.grid(True)
            ax.set_ylabel(yLabel)
            

        axTxt = self.fgh.add_subplot(gs[nRows-1,0])
        axTxt.axis("off")   

    #    self.ax.plot(self.allDataSets[self.currentDataset][self.channelNames[self.iChannel]], marker='x')
    #    self.ax.set_xlim((0,self.allDataSets[self.currentDataset][self.channelNames[self.iChannel]].shape[0]))

    #    self.ax.set_title(self.inputFile)
    #    self.ax.set_xlabel("Samples")
    #    self.ax.set_ylabel(self.channelNames[self.iChannel] )
        
        par_list = self.data[self.iPeriod].keys()
        dont_print_list = ["antenna_list","seq_list", "pcode", "year", 'minute', 'second', 'month', 'day', 'hour', 'microsecond', "beam", "rfreq"]
        par_txt = "\n{:04d}-{:02d}-{:02d} {:02d}:{:02d}:{:02d}.{:02d}\n".format(self.data[self.iPeriod]["year"], self.data[self.iPeriod]["month"], self.data[self.iPeriod]["day"], self.data[self.iPeriod]["hour"], self.data[self.iPeriod]["minute"], self.data[self.iPeriod]["second"], self.data[self.iPeriod]["microsecond"])
        par_txt += "beam {}, rfreq = {} kHz \n".format(self.data[self.iPeriod]["beam"], self.data[self.iPeriod]["rfreq"])

        print_par = [par for par in par_list if par not in dont_print_list]

        for iPar,par in enumerate(print_par):
            if ((iPar+1) % 3):
                par_txt += "{}={},  ".format(par, self.data[self.iPeriod][par])
            else:
                par_txt += "{}={},\n".format(par, self.data[self.iPeriod][par])

        axTxt.text(0, 1, par_txt, verticalalignment='top')
    #    self.ax.grid(True)
      #  self.ax.set_xlim((0,self.allDataSets[self.currentDataset][self.channelNames[self.iChannel]].shape[0]))
        
        ax = self.fgh.add_subplot(gs[nRows-1,1])
        #ax = plt.subplot(212)
        
        pulse_idx_list = [0, 1, 2, 3, 4]
        colorList = cm.rainbow(np.linspace(0,1,len(pulse_idx_list)))
        for iPulse, plotColor in zip(pulse_idx_list, colorList):
            iSample = int(np.round(self.data[self.iPeriod]['ppat'][iPulse] * self.data[self.iPeriod]['mpinc'] / self.data[self.iPeriod]['smsep']))
            exampleTXsamples = [plot_data[iAnt][iSample] for iAnt in range(nAntennas)]       
            arrayAngle = np.angle(exampleTXsamples, deg=True)
            arrayAngle = arrayAngle - arrayAngle[0]
            plt.scatter(np.array(self.data[self.iPeriod]['antenna_list'])+(np.random.rand(1)-0.5)/5, arrayAngle % 360, s=5*np.log(np.abs(exampleTXsamples)+0.01), color=plotColor)
               
        beamsep = 3.24 
        delta_x = 15.24
        nBeams = 16
        max_antenna_idx = 16  # max(self.data[self.iPeriod]['antenna_list'])

        plot_all_beams = False
        if plot_all_beams:
            for iBeam in range(16): 
               alpha = beamsep * (iBeam - (nBeams -1) /2 )
               phaseDiff_per_ant = - delta_x / 3e8 * self.data[self.iPeriod]["rfreq"] *1000 * np.sin(alpha/180*np.pi)  *360
               plt.plot(np.arange(max_antenna_idx), [phaseDiff_per_ant*iAnt % 360  for iAnt in range(max_antenna_idx)], ":")
                       
                       
        for iChannel in range(1): 
           alpha = beamsep * (self.data[self.iPeriod]['beam'] - (nBeams -1) /2 )
           phaseDiff_per_ant = - delta_x / 3e8 * self.data[self.iPeriod]["rfreq"] *1000 * np.sin(alpha/180*np.pi)  *360
           plt.plot(np.arange(max_antenna_idx), [phaseDiff_per_ant*(iAnt-1) % 360  for iAnt in range(max_antenna_idx)], linewidth=2)
           plt.plot(np.arange(max_antenna_idx), [(phaseDiff_per_ant*(iAnt-1)+180) % 360  for iAnt in range(max_antenna_idx)], "--", linewidth=1)

           
        plt.grid(True)
        plt.ylabel("phase difference in deg")
        plt.xlabel("antenna number")
        plt.axis([-0, max_antenna_idx, 0, 360])
        #legendList = ["Measured sample {} (ch 0)".format(idx) for idx in idx2checkVec] 
        
        plt.draw()            

    

# %% 
if __name__ == '__main__':
   # %% try input arguments ...
   raw_path = "/data/image_samples/bb_data/"
   
   if len(sys.argv) == 1:
       import glob
       print('No input. Looking for newest file in {} ...'.format(raw_path))
       file_with_path = max(glob.iglob('{}*.iraw.*'.format(raw_path)), key=os.path.getctime)
       fileName = file_with_path[len(raw_path):]
       print('found :{}'.format(fileName))
   else:
       fileName = sys.argv[1]
       if fileName.find("/") == -1: # no path defined => assume /data/diagnostic_samples/
           print('Only filename without path. looking for file in {} ...'.format(raw_path))
           file_with_path = os.path.join(raw_path, fileName)
       else:
           file_with_path = fileName
   
   x = RawDataGUI(file_with_path)
   x.open()
