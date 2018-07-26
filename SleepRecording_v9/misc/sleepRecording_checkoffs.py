"""
Author: Justin Baik
Date created: 11/28/2017
Title: run_experiment with GUI control
Notes:

This is a GUI program used to do the following:
- Keep track of mice information (one system can run upto 4 mice at the same time). The GUI defines how many mice are run and creates ID for each mouse.
- Start and stop EEG and EMG recording (using intan RHD2000) with user defined inputs. User can set the duration of the experiment and also set
the delay to start after a certain time.
- Run on 2 different modes: open loop and closed loop. Open loop mode sends signals to either Arduino or RPi to trigger the laser. Closed loop mode
works only with the raspberry pi using socket communication. Because pi is prone to C related crashes, only using arduino for now (on open loop). 
- creates and move info txt file that is unique to the experiment with the user input settings. This txt later gets used to post-process the raw data.
- does real-time spectral analysis of the EEG signal by reading off from the raw data that gets updated upon the start of an experiment. This spectral
data gets plotted on a second window.


This program is sort of a wrapper for the DAQ software that came with the Intan system. This program talks to the DAQ board via arduino or pi by sending
serial commands. The DAQ board has a trigger input that starts recording when it has a high input and stops when the signal shuts down. The DAQ software
also has a GUI to display the input signals from amplifiers / digital inputs / analog inputs. Right now, I am only using the amplifier signals for the
EEG and EMG, and the digital inputs for getting the camera strobe signals, laser trigger signals, the onoff signal that triggers DAQ recording.

This program has a lot of room for improvement, but because it's a bit messy, it has become increasingly inefficient to add features.
"""

# from PyQt5 import QtCore, QtGui, QtWidgets
# from PyQt5.QtCore import QObject, QThread, pyqtSignal, pyqtSlot, QTimer
# from PyQt5.QtWidgets import QFileDialog, QInputDialog, QLineEdit
# import pyqtgraph as pg
# from run_GUI_v7 import Ui_MainWindow
# import sleepy
# import socket
# import numpy as np
# import scipy
# import PyCapture2
# import cv2
# import serial
# import serial.tools.list_ports as com_read
# import time
# import sys
# import shutil
# import os
# import glob

# ################### Functions used for the spectral analysis #######################

# def power_spectrum(data, length, dt):
#     """
#     scipy's implementation of Welch's method using hanning window to estimate
#     the power spectrum
#     @Parameters
#         data    -   time series; float vector!
#         length  -   length of hanning window, even integer!
    
#     @Return:
#         powerspectrum, frequencies
#     """
#     f, pxx = scipy.signal.welch(data, fs=1/dt, window='hanning', nperseg=int(length), noverlap=int(length/2))
#     return pxx, f


# def smooth_data(x, sig):
#     """
#     y = smooth_data(x, sig)
#     smooth data vector @x with gaussian kernal
#     with standard deviation $sig
#     """
#     sig = float(sig)
#     if sig == 0.0:
#         return x
        
#     # gaussian:
#     gauss = lambda (x, sig) : (1/(sig*np.sqrt(2.*np.pi)))*np.exp(-(x*x)/(2.*sig*sig))

#     p = 1000000000
#     L = 10.
#     while (p > p):
#         L = L+10
#         p = gauss((L, sig))

#     F = map(lambda (x): gauss((x, sig)), np.arange(-L, L+1.))
#     F = F / np.sum(F)
    
#     return scipy.signal.fftconvolve(x, F, 'same')


# def recursive_spectrogram(EEG, EMG, sf=0.3, alpha=0.3):

#     len_eeg = len(EEG)
#     fdt = 2.5
#     SR = 1000.0
#     swin = int(np.round(SR)*5.0)
#     swinh = int(swin/2.0)
#     fft_win = int(swin/5.0)
#     spoints = int(np.floor(len_eeg/swinh))

#     # SE = np.zeros((fft_win/2+1, spoints))
#     # SM = np.zeros((fft_win/2+1, spoints))


#     x_e = EEG[0:swin]
#     [p_e,f_e] = power_spectrum(x_e.astype('float'), fft_win, 1.0/SR)
#     p_e = smooth_data(p_e,sf)
#     # SE[:,i] = alpha*p + (1-alpha) * SE[:,i-1]

#     x_m = EMG[0:swin]
#     [p_m,f_m] = power_spectrum(x_m.astype('float'), fft_win, 1.0/SR)
#     p_m = smooth_data(p_m,sf)
#     # SM[:,i] = alpha*p + (1-alpha) * SM[:,i-1]

#     return p_e, p_m, f_e, f_m





################################################################################################


####################################### QObjects for Threads  ##################################
"""
There are a total of 4 multithreads that run in this Qt program. Normally 3 but, if there is a
REM threshold txt file for each mouse that is listed in the experiment, the last thread also 
turns on. In another version that works, this spectrogram plotting works without the txt file, 
but these different versions haven't been merged. The other version doesn't have the 2 camera
feature, but can run spectrogram without the REM threshold values.
"""

# class TotTime_counter(QObject):
#     """
#     QThread object that runs timer for the experiment. 
#     This object is crucial for displaying the time count down until the experiment finished
#     Initiated with start button (connected to start_clicked).

#     On Delay:
#         Time ticks until delay is finished, then goes into the actual count down.
#         This function allows the actual data acquisition to start after a certain delay
#     When Delay is finished (or if delay is set as 0):
#         Still goes through the same process of finishing (ptimeDone) the delay mode,
#         then goes into the count down

#     """

#     signal_TimeRem = pyqtSignal(float, str)
#     signal_done = pyqtSignal()
#     signal_delayDone = pyqtSignal()
#     signal_ptimeDone = pyqtSignal()
#     signal_delayDone_wop = pyqtSignal()


#     def __init__(self, maxpg = 25, p_enabled = 2, t_start = 0, expdur = 0, delay = 0):
#         super(TotTime_counter, self).__init__()
#         self.timer_on = 0
#         self._maxpg = maxpg
#         self._start_time = t_start
#         self._expdur = expdur
#         self._delay = delay
#         self._p_on = p_enabled
#         self.delay_finished = 0

#     @pyqtSlot()
#     def run_count(self):
#         while self.timer_on == 1:
#             time_rem = (self._expdur*60*60 + self._delay*60) - (time.time()-self._start_time)
#             del_rem = (self._delay*60) - (time.time()-self._start_time)
#             if time_rem < self._maxpg * 60 * 1.5:
#                 self.signal_ptimeDone.emit()
#             if time_rem < 0.001:
#                 self.signal_done.emit()
#             if self._p_on == 2:
#                 if time_rem - self._expdur*60*60 < 0.001 and self.delay_finished == 0:
#                     self.delay_finished = 1
#                     self.signal_delayDone.emit()
#             else:
#                 if time_rem - self._expdur*60*60 < 0.001 and self.delay_finished == 0:
#                     self.delay_finished = 1
#                     self.signal_delayDone_wop.emit()

#             # If Delay is set
#             if self.delay_finished == 1:
#                 tm,ts = np.divmod(int(time_rem), 60)
#                 th,tm = np.divmod(tm,60)
#                 if len(str(tm)) == 1:
#                     minute = "0" + str(tm)
#                 else:
#                     minute = str(tm)    
#                 if len(str(ts)) == 1:
#                     second = "0" + str(ts)
#                 else:
#                     second = str(ts)
#                 time_display = str(th) + ":" + minute + ":" + second
#                 self.signal_TimeRem.emit(time_rem, time_display)

#             # If Delay time has ended
#             else:
#                 tm,ts = np.divmod(int(del_rem), 60)
#                 th,tm = np.divmod(tm,60)
#                 if len(str(tm)) == 1:
#                     minute = "0" + str(tm)
#                 else:
#                     minute = str(tm)    
#                 if len(str(ts)) == 1:
#                     second = "0" + str(ts)
#                 else:
#                     second = str(ts)
#                 del_display = str(th) + ":" + minute + ":" + second
#                 self.signal_TimeRem.emit(time_rem, "D: " + del_display)

#             time.sleep(0.2)

# class PTime_counter(QObject):
#     """
#     QThread object that run when the open-loop mode has been turned on. This keeps track of
#     time gaps between pulse trains (properties of the pulse trains are set using GUI inputs).
#     Works very similarly to TotTime_counter, but this one runs a separate counter to count down
#     time for the open-loop pulses.

#     This object is supposed to close some time before the experiment ends (i.e. when the
#     stop_clicked function is run to stop the recording)
#     """

#     signal_PTimeRem = pyqtSignal(float, str)
#     signal_firePulses = pyqtSignal()

#     def __init__(self, expdur = 0 , puldur = 0, pt_start = 0, minpg = 15, maxpg = 25):
#         super(PTime_counter, self).__init__()
#         self.ptimer_on = 0
#         self._pstart_time = pt_start
#         self._puldur = puldur
#         self._minpg = minpg
#         self._maxpg = maxpg
#         if minpg*60 < puldur:
#             self.pulGap = int(np.random.uniform(puldur, maxpg*60))
#         else:
#             self.pulGap = int(np.random.uniform(minpg, maxpg)*60)
#         self.writer = None

#     def run_pcount(self):
#         while self.ptimer_on == 1:
#             ptime_rem = self.pulGap - (time.time()-self._pstart_time)
#             if ptime_rem < 0.001:
#                 self.signal_firePulses.emit()

#                 ptime_rem = self.pulGap - (time.time()-self._pstart_time)
#                 self._pstart_time = time.time()
#                 if self._minpg*60 < self._puldur:
#                     self.pulGap = int(np.random.uniform(self._puldur, self._maxpg*60))
#                 else:
#                     self.pulGap = int(np.random.uniform(self._minpg, self._maxpg)*60)

#             pm,ps = np.divmod(int(ptime_rem), 60)
#             ph,pm = np.divmod(pm,60)
#             if len(str(pm)) == 1:
#                 minute = "0" + str(pm)
#             else:
#                 minute = str(pm)    
#             if len(str(ps)) == 1:
#                 second = "0" + str(ps)
#             else:
#                 second = str(ps)
#             ptime_display = str(ph) + ":" + minute + ":" + second
#             self.signal_PTimeRem.emit(ptime_rem, ptime_display)

#             time.sleep(0.2)

# class worker_camera(QObject):
#     """
#     QThread object that runs the camera. Its signals display camera frames on the GUI and it also writes
#     video frames to an MKV stack.

#     One recently aadded feature is the self.f and self.f2 that writes timestamps for the video frames in 
#     a txt file.

#     Because of the way PyCapture2 package generates the image buffers (into a long list of pixel values), there is a bunch of numpy flipping
#     and transposing to get the image shape write to display it properly. I am guessing this is where the
#     inefficiency is coming from as each line requires new memory.

#     Also, there are two cameras in the same QObject that run at the same time. There should really be one object for
#     each camera. The logic behind defining this class as such was that I tried doing so, but failed to create two
#     individual video files. This was probably because I didn't use the right fourcc (some of the writer codecs can only be used one at a time. For example, XVID and H264.
#     The only ones that worked were for writing from two cameras were MJPG and DIVX). It should work now with using the write codec, but needs to be revised.
#     """

#     signal_camFrame = pyqtSignal(str, np.ndarray, int)

#     def __init__(self, preview_or_record, cam1, cam2 = 0):
#         super(worker_camera, self).__init__()
#         self.PoR = preview_or_record
#         self.camera_on = 0
#         self._cam1 = cam1
#         self._cam2 = cam2
#         self.writer = 0
#         self.writer2 = 0
#         self.prev_f1 = 0
#         self.prev_f2 = 0
        

#     @pyqtSlot()
#     def run_camera(self):
#         if self.PoR == 'R':
#             self.f = open("cam1_timestamps.txt","w+")
#             self._cam1.writeRegister(0x1508, 0x82000000)
#             try:
#                 self._cam2.writeRegister(0x1508, 0x82000000)
#                 self.f2 = open("cam2_timestamps.txt","w+")
#             except:
#                 pass
        # else:
        #     self._cam1.writeRegister(0x1508, 0x80000000)
        #     try:
        #         self._cam2.writeRegister(0x1508, 0x80000000)
        #     except:
        #         pass

        # while self.camera_on == 1:
        #     try:
        #         image = self._cam1.retrieveBuffer()
        #         if self._cam2 != 0:
        #             image2 = self._cam2.retrieveBuffer()
        #     except PyCapture2.Fc2error as fc2Err:
        #         print "Error retrieving buffer : ", fc2Err
        #         continue
        #     self.imgt1 = image.getTimeStamp()
        #     if self.PoR == 'R':
        #         if len(str(self.imgt1.microSeconds)) == 6:
        #             self.f.write(str(self.imgt1.seconds)[-6:] + '.' + str(self.imgt1.microSeconds) + '\r\n')
        #         else:
        #             self.f.write(str(self.imgt1.seconds)[-6:] + '.0' + str(self.imgt1.microSeconds) + '\r\n')
        #     imgdat = image.getData()
        #     if self.prev_f1 != self.imgt1:
        #         imgnp = np.array(imgdat).reshape(image.getRows(),image.getCols())
        #         imgnp = np.transpose(imgnp)
        #         imgnp = np.flip(imgnp, 1)
        #         imgnp = np.flip(imgnp, 0)
        #         self.prev_f1 = self.imgt1
        #         t1 = 1
        #     else:
        #         t1 = 0

        #     if self._cam2 != 0:
        #         self.imgt2 = image2.getTimeStamp()
        #         if self.PoR == 'R':
        #             if len(str(self.imgt2.microSeconds)) == 6:
        #                 self.f2.write(str(self.imgt2.seconds)[-6:] + '.' + str(self.imgt2.microSeconds) + '\r\n')
        #             else:
        #                 self.f2.write(str(self.imgt2.seconds)[-6:] + '.0' + str(self.imgt2.microSeconds) + '\r\n')
        #         imgdat2 = image2.getData()
        #         if self.prev_f2 != self.imgt2:
        #             imgnp2 = np.array(imgdat2).reshape(image2.getRows(),image2.getCols())
        #             imgnp2 = np.transpose(imgnp2)
        #             imgnp2 = np.flip(imgnp2, 1)
        #             imgnp2 = np.flip(imgnp2, 0)
        #             self.prev_f2 = self.imgt2
        #             t2 = 1
        #         else:
        #             t2 = 0

        #     if self.PoR == 'R':
        #         if t1 == 1:
        #             self.writer.write(np.uint8(imgnp))
        #         if self._cam2 != 0:
        #             if t2 == 1:
        #                 self.writer2.write(np.uint8(imgnp2))
        #     if t1 == 1:
        #         self.signal_camFrame.emit(self.PoR, imgnp, 1)

        #     if self._cam2 != 0:
        #         if t2 == 1:
        #             self.signal_camFrame.emit(self.PoR, imgnp2, 2)
            
        #     time.sleep(0.01)

# class worker_getdata(QObject):
#     """
#     Thread object that retrieves data as it gets written from the DAQ board. Each loop cuts the
#     data into 2.5 second bits (sampling rate runs on 1000 SPS), then sends this to a function to calculate
#     spectrograms and to plot on another window. There is also a relay function that takes the data and identifies the 
#     signal for each mouse. 

#     Perhaps np.append is not the most efficient function to use here? 
#     """

#     signal_new5sec = pyqtSignal(dict, dict)

#     def __init__(self, od, eidx, midx, switch = 0):
#         super(worker_getdata, self).__init__()
#         self.odir = od
#         self.fid = open(self.odir + '/amplifier.dat', 'rb')
#         self.tid = open(self.odir + '/time.dat', 'r')
#         self.switch = switch
#         self.new_data = np.fromfile(self.fid, dtype='int16')
#         self.new_time = np.fromfile(self.tid, dtype='int32')
#         self.eidx = eidx
#         self.midx = midx
#         self.numchans = 0
#         for mouse in self.eidx:
#             self.numchans += 4


#         self.buffer_e = {k: np.ndarray(0) for k in eidx}
#         self.tmp_key = self.buffer_e.keys()[0]
#         self.buffer_m = {k: np.ndarray(0) for k in midx}


#         # we need: eeg_idx, emg_idx, new5_e, new5_m, 
#     def run_it(self):

#         while self.switch == 1:
#             self.new_data = np.fromfile(self.fid, dtype='int16')

#             if len(self.new_data) > 1:
#                 for mouse in self.eidx:
#                     self.buffer_e[mouse] = np.append(self.buffer_e[mouse], self.new_data[self.eidx[mouse]::self.numchans])
#                     self.buffer_m[mouse] = np.append(self.buffer_m[mouse], self.new_data[self.midx[mouse]::self.numchans])

#                 if len(self.buffer_e[self.tmp_key]) >= 2500:
#                     buffer_emitE = {}
#                     buffer_emitM = {}
#                     for mouse in self.eidx:
#                         buffer_emitE[mouse] = self.buffer_e[mouse][:2500]
#                         buffer_emitM[mouse] = self.buffer_m[mouse][:2500]
#                     self.signal_new5sec.emit(buffer_emitE, buffer_emitM)
                    
#                     for mouse in self.eidx:
#                         self.buffer_e[mouse] = self.buffer_e[mouse][2500:]
#                         self.buffer_m[mouse] = self.buffer_m[mouse][2500:]

#             time.sleep(0.01)
            
#################################################### Overall GUI settings and functions ##############################################

class controlBoard(QtWidgets.QMainWindow):
    """
    This class contains literally everything from thread objects to individual handles on GUI inputs.

    This class initiates variables to be used during the time that the program is running. Note that some of the variables defined 
    don't get used and they need to be cleaned up.

    """

    def __init__(self):

    # Setting GUI properties (Presetting)

        # super(controlBoard, self).__init__()
        # self.ui = Ui_MainWindow()
        # self.ui.setupUi(self)
        # self.setWindowIcon(QtGui.QIcon('sleepm.png'))
        # self.ui.pulseonbutton.setDisabled(True)
        # self.ui.pulseoffbutton.setDisabled(True)
        # self.ui.entercomment.setDisabled(True)
        # self.ui.t_rem.setAlignment(QtCore.Qt.AlignCenter)
        # self.ui.p_rem.setAlignment(QtCore.Qt.AlignCenter)
        # self.ui.pul_enable.setChecked(True)
        # self.ardorrasp = ''

        # Setup REM detection stuff
        self.originalDir = os.getcwd()
        # self.running = 0
        # self.SR = 1000
        # self.ui.graphicsPlot.showMaximized()

        # Setting core attributes for the Main and QThreads to use as references

        # self.start_on = 0
        # self.preview_on = 0
        # self.cam_running = 0
        # self.cam_connected_LED = 0
        # self.ard_connected_LED = 0
        # self.timer_obj = TotTime_counter()
        # self.cam1on = 0
        # self.cam2on = 0

        # # Create error dialog handle to be used throughout the code

        # self.error_dialog = QtWidgets.QMessageBox()
        # self.error_dialog.setIcon(QtWidgets.QMessageBox.Critical)
        # self.error_dialog.setWindowTitle("Error")
        # self.error_dialog.setWindowIcon(QtGui.QIcon('sleepm.png'))

        # Setup QT Signal and Slot Arrangement

        # self.ui.startbutton.clicked.connect(self.start_clicked)
        # self.ui.stopbutton.clicked.connect(self.stop_clicked)
        # self.ui.cont.clicked.connect(self.disableCustomTime)
        # self.ui.preview.clicked.connect(self.preview_clicked)
        # self.ui.entercomment.clicked.connect(self.submittxt)
        # self.ui.pulseonbutton.clicked.connect(self.arduinoRsignal_C)
        # self.ui.pulseoffbutton.clicked.connect(self.pulseoff_clicked)
        # self.ui.arduino_btn.clicked.connect(self.a_clicked)
        # self.ui.cam_btn.clicked.connect(self.c_clicked)
        self.ui.pul_enable.clicked.connect(self.disablePulses)
        # self.ui.cl_enable.clicked.connect(self.closed_loopinit)

        # self.load_dir = str(QFileDialog.getExistingDirectory(self, "Select the directory containing DAT files"))
        # self.configpath = str(QFileDialog.getExistingDirectory(self, "Select path to mouse_rem.txt files"))

    # controlBoard methods to respond to inputs

    # @pyqtSlot()
    # def closed_loopinit(self):
    #     if self.ui.cl_enable.checkState() == 2:
    #         self.ui.pul_enable.setChecked(False)
    #         self.disablePulses()
    #         self.ui.pul_enable.setDisabled(True)
    #     else:
    #         self.ui.pul_enable.setChecked(True)
    #         self.disablePulses()
    #         self.ui.pul_enable.setDisabled(False)


    @pyqtSlot()
    def start_clicked(self):

    # Checks if both the camera and arduino are connected

        # if self.ard_connected_LED == 0:
        #     self.error_dialog.setText("You must connect the arduino first!")
        #     self.error_dialog.show()

        # Checks that the conditions are met for the program to start

        elif self.start_on == 0 and self.preview_on == 0:
            # self.start_on = 1

            # Serial communication with the Arduino to change the default values
            # if self.ardorrasp == 'a':
            #     QTimer.singleShot(100, lambda: self.comPort.write(bytearray(b'H' + str(self.ui.hi.value()) + '\n')))
            #     QTimer.singleShot(100, lambda: self.comPort.write(bytearray(b'L' + str(self.ui.lo.value()) + '\n')))
            #     QTimer.singleShot(100, lambda: self.comPort.write(bytearray(b'D' + str(self.ui.pulsedur.value()) + '\n')))

            
            # self.ui.startbutton.setStyleSheet("background-color: red")
            # self.dur_time = int(self.ui.expdur.value()*60*60 + self.ui.delay.value()*60)

            # self.mouselist = []

            # if self.ui.m1chk.checkState() == 2:
            #     self.mouselist.append(self.ui.id1.text())
            # if self.ui.m2chk.checkState() == 2:
            #     self.mouselist.append(self.ui.id2.text())
            # if self.ui.m3chk.checkState() == 2:
            #     self.mouselist.append(self.ui.id3.text())
            # if self.ui.m4chk.checkState() == 2:
            #     self.mouselist.append(self.ui.id4.text())

            # # Disable all parameter inputs during the experiment

            # self.ui.ch1.setDisabled(True)
            # self.ui.id1.setDisabled(True)
            # self.ui.l1.setDisabled(True)
            # self.ui.m1chk.setDisabled(True)
            # self.ui.ch2.setDisabled(True)
            # self.ui.id2.setDisabled(True)
            # self.ui.l2.setDisabled(True)
            # self.ui.m2chk.setDisabled(True)
            # self.ui.ch3.setDisabled(True)
            # self.ui.id3.setDisabled(True)
            # self.ui.l3.setDisabled(True)
            # self.ui.m3chk.setDisabled(True)
            # self.ui.ch4.setDisabled(True)
            # self.ui.id4.setDisabled(True)
            # self.ui.l4.setDisabled(True)
            # self.ui.m4chk.setDisabled(True)
            
            # self.ui.sr.setDisabled(True)
            # self.ui.delay.setDisabled(True)
            # self.ui.expdur.setDisabled(True)
            # self.ui.pulsedur.setDisabled(True)
            # self.ui.minPG.setDisabled(True)
            # self.ui.maxPG.setDisabled(True)
            # self.ui.hi.setDisabled(True)
            # self.ui.lo.setDisabled(True)
            # self.ui.lid1.setDisabled(True)
            # self.ui.dialnum1.setDisabled(True)
            # self.ui.lid2.setDisabled(True)
            # self.ui.dialnum2.setDisabled(True)

            # self.ui.cam_btn.setDisabled(True)
            # self.ui.arduino_btn.setDisabled(True)
            # self.ui.preview.setDisabled(True)
            # self.ui.entercomment.setDisabled(False)
            # self.ui.pulseonbutton.setDisabled(False)
            # self.ui.pulseoffbutton.setDisabled(False)
            # self.ui.pul_enable.setDisabled(True)

            # time_init = time.localtime()
            # self.timetxt = str(time_init.tm_hour) + ':' + str(time_init.tm_min) + ':' + str(time_init.tm_sec)

            # self.setupNotes()

            # Activates GUI features to run during the experiment
            # if self.ui.pul_enable.checkState() == 2:
            #     if self.ui.delay.value() != 0:
            #         self.ui.p_rem.setText(QtCore.QCoreApplication.translate("MainWindow", "On Delay"))


            # self.timer_thread = QThread(self)
            # self.timer_obj = TotTime_counter(self.ui.maxPG.value(), self.ui.pul_enable.checkState(), 0, self.ui.expdur.value(), self.ui.delay.value())
            # self.timer_obj.moveToThread(self.timer_thread)
            # self.timer_obj.timer_on = 1
            # self.timer_obj.signal_TimeRem.connect(self.update_TimeRem)
            # self.timer_obj.signal_done.connect(self.stop_clicked)
            # self.timer_obj.signal_delayDone.connect(self.initpTime)
            # self.timer_obj.signal_delayDone_wop.connect(self.s_recording)
            # self.timer_obj.signal_ptimeDone.connect(self.endpTime)
            # self.timer_thread.started.connect(self.timer_obj.run_count)


            # if self.cam_connected_LED == 1:
            #     self.camera_thread = QThread(self)
            #     if self.cam2on == 1:
            #         self.camera_obj = worker_camera('P', self.c, self.c2)
            #     else:
            #         self.camera_obj = worker_camera('P', self.c)
            #     self.camera_obj.moveToThread(self.camera_thread)
            #     self.camera_obj.camera_on = 1
            #     self.camera_obj.signal_camFrame.connect(self.process_vid)
            #     self.camera_thread.started.connect(self.camera_obj.run_camera)
            #     self.c.startCapture()
            #     if self.cam2on == 1:
            #         self.c2.startCapture() 

                # Setup for video writing in cv (currently taking only 2 video outs)

                # self.fourcc = cv2.VideoWriter_fourcc(*'DIVX')
                # self.fourcc2 = cv2.VideoWriter_fourcc(*'DIVX')
                # self.writer1 = cv2.VideoWriter(self.todayis + "_" + self.ui.id1.text() + self.ui.id2.text() +"_vid.mkv", self.fourcc, self.frame_rate, (482, 644), isColor=0) ################
                # self.camera_obj.writer = self.writer1

                # if self.cam2on == 1:
                #     self.writer2 = cv2.VideoWriter(self.todayis + "_" + self.ui.id3.text() + self.ui.id4.text() +"_vid.mkv", self.fourcc2, self.frame_rate, (482, 644), isColor=0) ################
                #     self.camera_obj.writer2 = self.writer2


                # start_time = time.time()
                # self.timer_obj._start_time = start_time
                # self.timer_thread.start()
            #     self.camera_thread.start()
            
            # else:
            #     start_time = time.time()
            #     self.timer_obj._start_time = start_time
                
            #     self.timer_thread.start()

    def run_spectral(self):

        try:
            # # rem state on/off
            # if self.ardorrasp == 'r':
            #     self.s.send('START')

            #     for mouse in self.mouselist:
            #         self.s.send(mouse)
            #         time.sleep(0.1)

                    
            #     self.s.send('CountDone')

            #     if self.ui.cl_enable.checkState() == 2:
            #         time.sleep(0.1)
            #         self.s.send('cl')
            #     elif self.ui.pul_enable.checkState() == 2:
            #         time.sleep(0.1)
            #         self.s.send('ol')
            #         time.sleep(0.1)
            #         self.s.send(str(self.ui.hi.value()))
            #         time.sleep(0.1)
            #         self.s.send(str(self.ui.lo.value()))
            #         time.sleep(0.1)
            #         self.s.send(str(self.ui.pulsedur.value()))
            #     else:
            #         time.sleep(0.1)
            #         self.s.send('nn')

            # self.prem = {k: 0 for k in self.mouselist}
            # self.pow_muh = {k: [] for k in self.mouselist}
            # self.rem_hist = {k: [] for k in self.mouselist}
            # self.past_len = {k: int(120/2.5) for k in self.mouselist}
            # self.num_iter = {k: 0 for k in self.mouselist}

            # # Range values for each power
            # self.r_delta = [0.5, 4]
            # self.r_theta = [5,12]
            # self.r_mu = [300, 500]

            # self.alpha = 0.3

            # # self.win_len = len(np.zeros(self.SR*10))
            # self.EEG_5 = {k: np.ndarray(0) for k in self.mouselist}
            # self.EMG_5 = {k: np.ndarray(0) for k in self.mouselist}

            # self.spectro_buf1 = {k: np.zeros((240,501)) for k in self.mouselist}
            # self.spectro_buf2 = {k: np.zeros((240,501)) for k in self.mouselist}

            # self.thr_delta = {}
            # self.thr_deltap = {}
            # self.thr_th_delta1 = {}
            # self.thr_th_delta1p = {}
            # self.thr_th_delta2 = {}
            # self.thr_th_delta2p = {}
            # self.thr_mu = {}    
            # self.thr_mup = {}       
            # self.channels = {}
            # self.bern_rem = {}
            # self.bern_counter = {}

            # for mouse in self.mouselist:
            #     params = sleepy.load_sleep_params(self.configpath, mouse + '_rem.txt')
            #     self.thr_delta[mouse] = params['THR_DELTA'][0]
            #     self.thr_deltap[mouse] = np.ones(240)*self.thr_delta[mouse]
            #     self.thr_th_delta1[mouse] = params['THR_TH_DELTA'][0]
            #     self.thr_th_delta1p[mouse] = np.ones(240)*self.thr_th_delta1[mouse]
            #     self.thr_th_delta2[mouse] = params['THR_TH_DELTA'][1]
            #     self.thr_th_delta2p[mouse] = np.ones(240)*self.thr_th_delta2[mouse]
            #     self.thr_mu[mouse] = params['THR_MU'][0]
            #     self.thr_mup[mouse] = np.ones(240)*self.thr_mu[mouse]
            #     self.channels[mouse] = params['ch_alloc'][0]
            

            # counter = 0
            # self.eeg_idx = {}
            # self.emg_idx = {}
            # for mouse in self.mouselist:
            #     self.eeg_idx[mouse] = self.channels[mouse].find('E') + counter
            #     self.emg_idx[mouse] = self.channels[mouse].find('M') + counter
            #     counter += 4


            # self.delta_h = {k: list(np.zeros(240)) for k in self.mouselist}
            # self.mu_h = {k: list(np.zeros(240)) for k in self.mouselist}
            # self.theta_h = {k: list(np.zeros(240)) for k in self.mouselist}
            # self.rem_h = {k: list(np.zeros(240)) for k in self.mouselist}
            # self.th_delta_h = {k: list(np.zeros(240)) for k in self.mouselist}


            # pos = np.array([0., 0.05, .2, .4, .6, .9])
            # color = np.array([[0, 0, 0, 255], [0,0,255,255], [0,255,0,255], [255,255,0, 255], (255,165,0,255), (255,0,0, 255)], dtype=np.ubyte)
            # cmap = pg.ColorMap(pos, color)
            # self.lut = cmap.getLookupTable(0.0, 1.0, 256)

    #         self.image_fft1 = {k: pg.ImageItem() for k in self.mouselist}
            
    #         self.allplots = {}
    #         for i in range(len(self.mouselist)):
    #             self.allplots[self.mouselist[i]] = [self.ui.graphicsPlot.addPlot(title = 'Delta Threshold', row = 1, col = i+1)]
    #             self.allplots[self.mouselist[i]].append(self.ui.graphicsPlot.addPlot(title = 'Theta/Delta Threshold', row = 2, col = i+1))
    #             self.allplots[self.mouselist[i]].append(self.ui.graphicsPlot.addPlot(title = 'Mu Power Threshold', row = 3, col = i+1))
    #             self.allplots[self.mouselist[i]].append(self.ui.graphicsPlot.addPlot(title = 'EEG Spectrogram', row = 4, col = i+1))

    #             spec_plot = self.allplots[self.mouselist[i]][-1]
    #             spec_plot.clear()
    #             spec_plot.addItem(self.image_fft1[self.mouselist[i]])
    #             ax = spec_plot.getAxis(name='left')
    #             ax.setTicks([[(0, '0'), (10, '10'), (20, '20')]])

    #         for mouse in self.mouselist:
    #             for plot in self.allplots[mouse]:
    #                 plot.setMouseEnabled(x = False)
    #             try:
    #                 self.bern_rem[mouse] = np.random.binomial(1,params['Bern'][0],1000)
    #                 self.bern_counter[mouse] = 1
    #             except KeyError:
    #                 pass

    #         self.running = 1
    #         self.folderlist = glob.glob(self.load_dir + '/*')
    #         self.datdir = max(self.folderlist, key=os.path.getctime)
    #         self.updater_thread = QThread(self)
    #         self.data_getter = worker_getdata(self.datdir, self.eeg_idx, self.emg_idx, switch = 1)
    #         self.data_getter.moveToThread(self.updater_thread)
    #         self.data_getter.signal_new5sec.connect(self.update_master)
    #         self.updater_thread.started.connect(self.data_getter.run_it)
    #         self.updater_thread.start()

    #     except AttributeError:
    #         print('Choose a proper directory')
    #     except IOError:
    #         self.error_dialog.setText("rem txt file does not exist - continuing without spectrogram")
    #         self.error_dialog.show()


    # @pyqtSlot(dict, dict)
    # def update_master(self, new_deeg, new_demg):
    #     for mouse in self.mouselist:
    #         self.EEG_5[mouse] = np.append(self.EEG_5[mouse], new_deeg[mouse])
    #         self.EMG_5[mouse] = np.append(self.EMG_5[mouse], new_demg[mouse])
    #         try:
    #             self.update_spec(mouse, self.EEG_5[mouse], self.EMG_5[mouse])
    #         except TypeError:
    #             print('four inputs')

    #         if len(self.EEG_5[mouse]) >= 5000:
    #             self.EEG_5[mouse] = new_deeg[mouse]
    #             self.EMG_5[mouse] = new_demg[mouse]

    # def update_spec(self, mousename, data_eeg, data_emg):

    #     if len(data_eeg) >= 5000:
    #         self.p_e, self.p_m, self.f_e, self.f_m = recursive_spectrogram(data_eeg, data_emg)

    #         for plots in self.allplots[mousename]:
    #             plots.clear()

    #         self.SE = self.alpha*self.p_e + (1-self.alpha) * self.spectro_buf1[mousename][-1,:]
    #         self.SM = self.alpha*self.p_m + (1-self.alpha) * self.spectro_buf2[mousename][-1,:]

    #         # power calculation

    #         i_delta = np.where((self.f_e >= self.r_delta[0]) & (self.f_e <= self.r_delta[1]))[0]
    #         i_theta = np.where((self.f_e >= self.r_theta[0]) & (self.f_e <= self.r_theta[1]))[0]
    #         i_mu = np.where((self.f_m >= self.r_mu[0]) & (self.f_m <= self.r_mu[1]))[0]
            
    #         pow_delta = np.sum(self.SE[i_delta])
    #         self.delta_h[mousename].append(pow_delta)
    #         self.delta_h[mousename] = self.delta_h[mousename][-240:]
    #         self.allplots[mousename][0].plot(self.delta_h[mousename])
    #         self.allplots[mousename][0].plot(self.thr_deltap[mousename], pen = pg.mkPen('r'))

    #         pow_theta = np.sum(self.SE[i_theta])
    #         self.theta_h[mousename].append(pow_theta)
    #         self.theta_h[mousename] = self.theta_h[mousename][-240:]

    #         th_delta = np.divide(pow_theta, pow_delta)
    #         self.th_delta_h[mousename].append(th_delta)
    #         self.th_delta_h[mousename] = self.th_delta_h[mousename][-240:]
    #         self.allplots[mousename][1].plot(self.th_delta_h[mousename])
    #         self.allplots[mousename][1].plot(self.thr_th_delta1p[mousename], pen = pg.mkPen('r'))
    #         self.allplots[mousename][1].plot(self.thr_th_delta2p[mousename], pen = pg.mkPen('b'))

    #         pow_mu = np.sum(self.SM[i_mu])
    #         self.pow_muh[mousename].append(pow_mu)
    #         self.mu_h[mousename].append(pow_mu)
    #         self.mu_h[mousename] = self.mu_h[mousename][-240:]
    #         self.allplots[mousename][2].plot(self.mu_h[mousename])
    #         self.allplots[mousename][2].plot(self.thr_mup[mousename], pen = pg.mkPen('r'))


    #         # determine rem state

    #         if (self.prem[mousename] == 0 and pow_delta < self.thr_delta[mousename] and pow_mu < self.thr_mu[mousename]):
    #         ### could be REM
            
    #             if (th_delta > self.thr_th_delta1[mousename]):
    #             ### we are potentially entering REM
    #                 if (self.past_len[mousename] < self.num_iter[mousename]):
    #                     past_len = self.past_len[mousename]
    #                 else:
    #                     past_len = self.num_iter[mousename]

    #                 # count the percentage of brainstate bins with elevated EMG power
    #                 if past_len != 0:
    #                     c_mu = np.sum(np.where(self.pow_muh[mousename][(past_len*-1):]>self.thr_mu[mousename])[0] ) / past_len

    #                     if c_mu < 0.2: # 0.2 = past_mu previously
    #                     ### we are in REM
    #                         self.prem[mousename] = 1  # turn laser on

    #         # We are currently in REM; do we stay there?
    #         if self.prem[mousename] == 1:
    #             ### REM continues, if theta/delta is larger than soft threshold and if there's
    #             ### no EMG activation
    #             if ((th_delta > self.thr_th_delta2[mousename]) and (pow_mu < self.thr_mu[mousename])):
    #                 self.prem[mousename] = 1
    #             else:
    #                 self.prem[mousename] = 0 #turn laser off
    #                 self.bern_counter[mousename] += 1


    #         try:
    #             if self.prem[mousename] == 1:
    #                 try:
    #                     if self.ui.cl_enable.checkState() == 2:
    #                         if self.bern_rem[mousename][self.bern_counter[mousename]] == 1:
    #                             self.s.send(mousename + '2')
    #                             self.prem[mousename] += 1 # bump up prem to 2 if actually on vs. 1 when it is randomly off
    #                         else:
    #                             self.s.send(mousename + '1')
    #                 except KeyError:
    #                     print('laser on at 100%' + ' rate')
    #                     if self.ui.cl_enable.checkState() == 2:
    #                         self.s.send(mousename + '2')
    #             else:
    #                 if self.ui.cl_enable.checkState() == 2:
    #                     self.s.send(mousename + '0')
    #         except socket.error:
    #             print('error')

    #         self.rem_hist[mousename].append(self.prem[mousename])
    #         self.rem_h[mousename].append(self.prem[mousename])
    #         self.rem_h[mousename] = self.rem_h[mousename][-240:]
    #         self.allplots[mousename][1].plot(self.rem_h[mousename], pen = pg.mkPen('g'))

    #         if self.prem[mousename] == 2:
    #             self.prem[mousename] = 1

    #         # Buffer add

    #         self.num_iter[mousename] += 1


    #         self.spectro_buf1[mousename] = np.vstack([self.spectro_buf1[mousename], [self.SE]])[1:,:]
    #         self.spectro_buf2[mousename] = np.vstack([self.spectro_buf2[mousename], [self.SM]])[1:,:]

    #         x = pg.makeARGB(self.spectro_buf1[mousename], levels = [0,45000], lut = self.lut)[0]
    #         y = pg.makeARGB(self.spectro_buf2[mousename], levels = [0,45000], lut = self.lut)[0]

    #         self.allplots[mousename][3].addItem(self.image_fft1[mousename])
    #         # self.P6.addItem(self.image_fft2)
    #         self.image_fft1[mousename].setImage(x)
    #         # self.image_fft2.setImage(y)

    # def setupNotes(self):
    # # Create prefix for all files using time library. self.todayis = "YearMonthDate_HrMinSec"

    #     self.todayis = str(time.localtime().tm_year)[2:4]
    #     self.dateNote = ''

    #     if len(str(time.localtime().tm_mon)) < 2:
    #         self.todayis += '0' + str(time.localtime().tm_mon)
    #         self.dateNote += '0' + str(time.localtime().tm_mon) + '/'
    #     else:
    #         self.todayis += str(time.localtime().tm_mon)
    #         self.dateNote += str(time.localtime().tm_mon) + '/'

    #     if len(str(time.localtime().tm_mday)) < 2:
    #         self.todayis += '0' + str(time.localtime().tm_mday)
    #         self.dateNote += '0' + str(time.localtime().tm_mday) + '/'
    #     else:
    #         self.todayis += str(time.localtime().tm_mday)
    #         self.dateNote += str(time.localtime().tm_mday) + '/'

    #     self.dateNote += str(time.localtime().tm_year)[2:4]

    #     self.todayis += str('_')
    #     if len(str(time.localtime().tm_hour)) < 2:
    #         self.todayis += '0' + str(time.localtime().tm_hour)
    #     else:
    #         self.todayis += str(time.localtime().tm_hour)

    #     if len(str(time.localtime().tm_min)) < 2:
    #         self.todayis += '0' + str(time.localtime().tm_min)
    #     else:
    #         self.todayis += str(time.localtime().tm_min)
    #     if len(str(time.localtime().tm_sec)) < 2:
    #         self.todayis += '0' + str(time.localtime().tm_sec)
    #     else:
    #         self.todayis += str(time.localtime().tm_sec)

    #     # Writes a txt file to include all experiment setting and variables

    #     self.f = open(self.todayis + "_notes.txt","w+")
    #     self.f.write('date:\t' + self.dateNote + '\r\n')
    #     self.f.write('time:\t' + self.timetxt + '\r\n')
    #     self.f.write('amplifier:\t' + 'intan' + '\r\n')
    #     self.f.write('SR:\t' + str(self.ui.sr.value()) + '\r\n')
    #     self.f.write('delay:\t' + str(self.ui.delay.value()) + '\r\n')
    #     if self.ui.cl_enable.checkState() == 2:
    #         self.f.write('mode:\t' + 'cl\r\n')
    #     elif self.ui.pul_enable.checkState() == 2:
    #         self.f.write('mode:\t' + 'ol\r\n')
    #     else:
    #         self.f.write('mode:\t' + 'none\r\n')
    #     if self.ui.pul_enable.checkState() == 2:
    #         self.f.write('laser_dur:\t' + str(self.ui.pulsedur.value()) + '\r\n')
    #         self.f.write('laser_hi:\t' + str(self.ui.hi.value()) + '\r\n')
    #         self.f.write('laser_lo:\t' + str(self.ui.lo.value()) + '\r\n')
    #         self.f.write('stim_freq:\t' + str(int(1.0/(self.ui.hi.value()+self.ui.lo.value())/0.001)) + '\r\n')
    #     self.f.write('exp_dur:\t' + str(self.ui.expdur.value()) + '\r\n')
    #     self.f.write('mouse_ID:\t')
    #     if self.ui.m1chk.checkState() == 2:
    #         self.f.write(self.ui.id1.text() + '\t')
    #     else:
    #         self.f.write('\t ')
    #     if self.ui.m2chk.checkState() == 2:
    #         self.f.write(self.ui.id2.text() + '\t')
    #     else:
    #         self.f.write('\t ')
    #     if self.ui.m3chk.checkState() == 2:
    #         self.f.write(self.ui.id3.text() + '\t')
    #     else:
    #         self.f.write('\t ')
    #     if self.ui.m4chk.checkState() == 2:
    #         self.f.write(self.ui.id4.text())
    #     else:
    #         self.f.write('\t ')
    #     self.f.write('\r\n')
    #     self.f.write('ch_alloc:\t')
    #     if self.ui.m1chk.checkState() == 2:
    #         self.f.write(self.ui.ch1.text() + '\t')
    #     if self.ui.m2chk.checkState() == 2:
    #         self.f.write(self.ui.ch2.text() + '\t')
    #     if self.ui.m3chk.checkState() == 2:
    #         self.f.write(self.ui.ch3.text() + '\t')
    #     if self.ui.m4chk.checkState() == 2:
    #         self.f.write(self.ui.ch4.text())
    #     self.f.write('\r\n')
    #     self.f.write('laser_used:\t')
    #     if self.ui.m1chk.checkState() == 2:
    #         self.f.write(self.ui.l1.text() + '\t')
    #     if self.ui.m2chk.checkState() == 2:
    #         self.f.write(self.ui.l2.text() + '\t')
    #     if self.ui.m3chk.checkState() == 2:
    #         self.f.write(self.ui.l3.text() + '\t')
    #     if self.ui.m4chk.checkState() == 2:
    #         self.f.write(self.ui.l4.text())
    #     self.f.write('\r\n')
    #     self.f.write('#Notes:\t')
    #     self.f.close()


    # @pyqtSlot(float, str)
    # def update_TimeRem(self, time_rem, new_time):
    #     self.time_rem = time_rem
    #     if self.timer_obj.timer_on == 1:
    #         self.ui.t_rem.setText(QtCore.QCoreApplication.translate("MainWindow", new_time))
    #     else:
    #         self.ui.t_rem.setText(QtCore.QCoreApplication.translate("MainWindow", "STOP"))
    
    @pyqtSlot()
    def s_recording(self):

        if self.cam_connected_LED == 1:
            self.camera_obj._cam1.writeRegister(0x1508, 0x82000000)
            self.camera_obj.PoR = "R"
            self.camera_obj.f = open("cam1_timestamps.txt","w+")
            try:
                self.camera_obj._cam2.writeRegister(0x1508, 0x82000000)
                self.camera_obj.f2 = open("cam2_timestamps.txt","w+")
            except:
                pass

        # if self.ardorrasp == 'a':
        #     self.comPort.write(bytearray(b'S\n'))

        QTimer.singleShot(100, self.run_spectral)

    # @pyqtSlot()
    # def initpTime(self):

        # if self.cam_connected_LED == 1:
        #     self.camera_obj._cam1.writeRegister(0x1508, 0x82000000)
        #     self.camera_obj.f = open("cam1_timestamps.txt","w+")
        #     try:
        #         self.camera_obj._cam2.writeRegister(0x1508, 0x82000000)
        #         self.camera_obj.f2 = open("cam2_timestamps.txt","w+")
        #     except:
        #         pass
        #     self.camera_obj.PoR = "R"

        # if self.ardorrasp == 'a':
        #     self.comPort.write(bytearray(b'S\n'))
            
        # self.ptimer_thread = QThread(self)
        # pstart_time = time.time()
        # self.ptimer_obj = PTime_counter(self.ui.expdur.value(), self.ui.pulsedur.value(), pstart_time, self.ui.minPG.value(), self.ui.maxPG.value())
        # self.ptimer_obj.moveToThread(self.ptimer_thread)
        # self.ptimer_obj.ptimer_on = 1
        # self.ptimer_obj.signal_PTimeRem.connect(self.update_PTimeRem)
        # self.ptimer_obj.signal_firePulses.connect(self.arduinoRsignal)
        # self.ptimer_thread.started.connect(self.ptimer_obj.run_pcount)
        # self.ptimer_thread.start()

        QTimer.singleShot(100, self.run_spectral)

    # @pyqtSlot()
    # def endpTime(self):
    #     try: 
    #         self.ptimer_obj.ptimer_on = 0
    #         self.ptimer_thread.quit()
    #         self.ptimer_thread.wait()
    #         if self.ui.pul_enable.checkState() == 2:
    #             self.ui.p_rem.setText(QtCore.QCoreApplication.translate("MainWindow", "STOP"))
    #     except AttributeError:
    #         pass

    @pyqtSlot()
    def arduinoRsignal(self):
        # if self.ardorrasp == 'a':
        #     self.comPort.write(bytearray(b'R\n'))
        elif self.ardorrasp == 'r':
            self.s.send('r')

        # self.ui.pulseonbutton.setDisabled(True)
        # self.ui.pulseoffbutton.setDisabled(True)
        # self.ui.pulseopen_label.setText(QtCore.QCoreApplication.translate("MainWindow", "Pulse in Use"))
        # self.ui.pul_open.setPixmap(QtGui.QPixmap("green-led-on.png"))
        
        # def enableAll():
        #     self.ui.pulseonbutton.setDisabled(False)
        #     self.ui.pulseoffbutton.setDisabled(False)
        #     self.ui.pulseopen_label.setText(QtCore.QCoreApplication.translate("MainWindow", "Pulse Open"))
        #     self.ui.pul_open.setPixmap(QtGui.QPixmap("led-red-on.png"))

        # QTimer.singleShot(self.ui.pulsedur.value()*1000, enableAll)


    # @pyqtSlot(float, str)
    # def update_PTimeRem(self, ptime_rem, new_ptime):
    #     if self.timer_obj.timer_on == 1 and self.ptimer_obj.ptimer_on == 1:
    #         self.ui.p_rem.setText(QtCore.QCoreApplication.translate("MainWindow", new_ptime))
    #     else:
    #         self.ui.p_rem.setText(QtCore.QCoreApplication.translate("MainWindow", "STOP"))


    # @pyqtSlot(str, np.ndarray, int)
    # def process_vid(self, PoR, frame, camid):
    #     if camid == 1:
    #         self.ui.graphicsView.setImage(frame)
    #     if camid == 2:
    #         self.ui.graphicsView_2.setImage(frame)


    @pyqtSlot()
    def stop_clicked(self):
        # Terminates all ongoing processes

        # self.ui.graphicsPlot.clear()

        # if self.start_on == 1 and self.preview_on == 0:
            # self.start_on = 0
            # self.ui.startbutton.setStyleSheet(self.ui.stopbutton.styleSheet())
            # self.ui.p_rem.setText(QtCore.QCoreApplication.translate("MainWindow", "STOP"))
            # if self.ardorrasp == 'a':
            #     self.comPort.write(bytearray(b'E\n'))

            # try:
            #     self.endpTime()
            # except AttributeError:
            #     pass

            # self.timer_obj.timer_on = 0
            # self.timer_thread.quit()
            # self.timer_thread.wait()

            # if self.cam_connected_LED == 1:
            #     self.camera_obj.camera_on = 0
            #     self.c.stopCapture()
            #     self.c.writeRegister(self.pin2_strobecnt, self.StrobeOff)

            #     if self.cam2on == 1:
            #         self.c2.stopCapture()
            #         self.c2.writeRegister(self.pin2_strobecnt, self.StrobeOff)
                    
            #     self.camera_thread.quit()
            #     self.camera_thread.wait()

            #     self.writer1.release()
            #     if self.cam2on == 1:
            #         self.writer2.release()




            # self.ui.ch1.setDisabled(False)
            # self.ui.id1.setDisabled(False)
            # self.ui.l1.setDisabled(False)
            # self.ui.m1chk.setDisabled(False)
            # self.ui.ch2.setDisabled(False)
            # self.ui.id2.setDisabled(False)
            # self.ui.l2.setDisabled(False)
            # self.ui.m2chk.setDisabled(False)
            # self.ui.ch3.setDisabled(False)
            # self.ui.id3.setDisabled(False)
            # self.ui.l3.setDisabled(False)
            # self.ui.m3chk.setDisabled(False)
            # self.ui.ch4.setDisabled(False)
            # self.ui.id4.setDisabled(False)
            # self.ui.l4.setDisabled(False)
            # self.ui.m4chk.setDisabled(False) 
    
            # self.ui.sr.setDisabled(False)
            # self.ui.delay.setDisabled(False)
            # self.ui.expdur.setDisabled(False)
            # self.ui.pulsedur.setDisabled(False)
            # self.ui.minPG.setDisabled(False)
            # self.ui.maxPG.setDisabled(False)
            # self.ui.hi.setDisabled(False)
            # self.ui.lo.setDisabled(False)
            # self.ui.lid1.setDisabled(False)
            # self.ui.dialnum1.setDisabled(False)
            # self.ui.lid2.setDisabled(False)
            # self.ui.dialnum2.setDisabled(False)

            # self.ui.cam_btn.setDisabled(False)
            # self.ui.arduino_btn.setDisabled(False)
            # self.ui.preview.setDisabled(False)
            # self.ui.entercomment.setDisabled(True)
            # self.ui.pulseonbutton.setDisabled(True)
            # self.ui.pulseoffbutton.setDisabled(True)
            # self.ui.pul_enable.setDisabled(False)

            # self.disablePulses()

            # self.f = open(self.todayis + "_notes.txt", 'a')
            # self.f.write('\r\n')
            # self.tot_time = self.dur_time - self.time_rem
            # self.mr,self.srem = np.divmod(self.tot_time, 60)
            # self.hr,self.mr = np.divmod(self.mr,60)
            # self.f.write('actual_duration:\t'+ str(int(self.hr)) + 'h:' + str(int(self.mr)) + 'm:' + str(int(self.srem)) + 's')
            # self.f.close()

            # try:
            #     self.camera_obj.f.close()
            #     self.camera_obj.f2.close()
            # except:
            #     pass

            if self.ardorrasp == 'r':
                try:
                    self.s.send('STOP')
                    # self.data_getter.switch = 0
                    # self.data_getter.fid.close()
                    # self.data_getter.tid.close()
                    # self.updater_thread.quit()
                    # self.updater_thread.wait()
                except AttributeError:
                    pass

            # self.cdir = os.getcwd()
            # self.save_dir = str(QFileDialog.getExistingDirectory(self, "Select Directory"))
            # for filename in os.listdir('.'):
            #     if self.todayis in filename:
            #         shutil.move(self.cdir + "\\" + filename, self.save_dir + "\\" + filename)
            #     if "timestamp" in filename:
            #         shutil.move(self.cdir + "\\" + filename, self.save_dir + "\\" + filename)

            

    # @pyqtSlot()
    # def disableCustomTime(self):
    #     if self.ui.cont.checkState() == 2:
    #         self.ui.cpulsedur.setDisabled(True)
    #     else:
    #         self.ui.cpulsedur.setDisabled(False)

    # @pyqtSlot()
    # def disablePulses(self):
    #     if self.ui.pul_enable.checkState() != 2:
    #         self.ui.pulsedur.setDisabled(True)
    #         self.ui.minPG.setDisabled(True)
    #         self.ui.maxPG.setDisabled(True)
    #         self.ui.hi.setDisabled(True)
    #         self.ui.lo.setDisabled(True)
    #         self.ui.lid1.setDisabled(True)
    #         self.ui.dialnum1.setDisabled(True)
    #         self.ui.lid2.setDisabled(True)
    #         self.ui.dialnum2.setDisabled(True)
    #         self.ui.p_rem.setText(QtCore.QCoreApplication.translate("MainWindow", "Disabled"))
    #     else:
    #         self.ui.pulsedur.setDisabled(False)
    #         self.ui.minPG.setDisabled(False)
    #         self.ui.maxPG.setDisabled(False)
    #         self.ui.hi.setDisabled(False)
    #         self.ui.lo.setDisabled(False)
    #         self.ui.lid1.setDisabled(False)
    #         self.ui.dialnum1.setDisabled(False)
    #         self.ui.lid2.setDisabled(False)
    #         self.ui.dialnum2.setDisabled(False)
    #         self.ui.p_rem.setText(QtCore.QCoreApplication.translate("MainWindow", ''))

    # @pyqtSlot()
    # def preview_clicked(self):
    #     if self.cam_connected_LED == 0:
    #         self.error_dialog.setText("You must connect the camera first!")
    #         self.error_dialog.show()

    #     elif self.start_on == 0 and self.preview_on == 0:
    #         self.ui.preview.setStyleSheet("background-color: green")
    #         self.preview_on = 1
    #         self.ui.cam_btn.setDisabled(True)
    #         self.ui.arduino_btn.setDisabled(True)
    #         self.ui.startbutton.setDisabled(True)
    #         self.ui.stopbutton.setDisabled(True)
    #         self.ui.entercomment.setDisabled(True)
    #         self.ui.pulseonbutton.setDisabled(False)
    #         self.ui.pulseoffbutton.setDisabled(False)

    #         if self.cam2on != 1:
    #             self.camera_thread = QThread(self)
    #             self.camera_obj = worker_camera('P', self.c)
    #             self.camera_obj.moveToThread(self.camera_thread)
    #             self.camera_obj.camera_on = 1
    #             self.camera_obj.signal_camFrame.connect(self.process_vid)
    #             self.camera_thread.started.connect(self.camera_obj.run_camera)
    #             self.c.startCapture()
    #             self.camera_thread.start()

    #         else:
    #             self.camera_thread = QThread(self)
    #             self.camera_obj = worker_camera('P', self.c, self.c2)
    #             self.camera_obj.moveToThread(self.camera_thread)
    #             self.camera_obj.camera_on = 1
    #             self.camera_obj.signal_camFrame.connect(self.process_vid)
    #             self.camera_thread.started.connect(self.camera_obj.run_camera)
    #             self.c.startCapture()
    #             self.c2.startCapture()
    #             self.camera_thread.start()


    #     elif self.start_on == 0 and self.preview_on == 1:
    #         self.ui.preview.setStyleSheet(self.ui.stopbutton.styleSheet())
    #         self.preview_on = 0
    #         self.ui.cam_btn.setDisabled(False)
    #         self.ui.arduino_btn.setDisabled(False)
    #         self.ui.startbutton.setDisabled(False)
    #         self.ui.stopbutton.setDisabled(False)
    #         self.ui.pulseonbutton.setDisabled(True)
    #         self.ui.pulseoffbutton.setDisabled(True)

    #         if self.cam2on != 1:
    #             self.camera_obj.camera_on = 0
    #             self.c.stopCapture()
    #             self.c.writeRegister(self.pin2_strobecnt, self.StrobeOff)
    #             self.camera_thread.quit()
    #             self.camera_thread.wait()

    #         else:
    #             self.camera_obj.camera_on = 0
    #             self.c.stopCapture()
    #             self.c2.stopCapture()
    #             self.c.writeRegister(self.pin2_strobecnt, self.StrobeOff)
    #             self.c2.writeRegister(self.pin2_strobecnt, self.StrobeOff)
    #             self.camera_thread.quit()
    #             self.camera_thread.wait()

    # @pyqtSlot()
    # def submittxt(self):
    #     self.f = open(self.todayis + "_notes.txt", 'a')
    #     self.f.write("\r\n//")
    #     self.f.write(time.asctime()[11:19] + " ")
    #     self.f.write(self.ui.commentbox.toPlainText())
    #     self.ui.commentbox.clear()
    #     self.f.close()


    # def pulseon_clicked(self):
    #     self.ui.pulseopen_label.setText(QtCore.QCoreApplication.translate("MainWindow", "Pulse in Use"))
    #     self.ui.pul_open.setPixmap(QtGui.QPixmap("green-led-on.png"))

    # def pulseoff_clicked(self):
    #     self.ui.pulseopen_label.setText(QtCore.QCoreApplication.translate("MainWindow", "Pulse Open"))
    #     self.ui.pul_open.setPixmap(QtGui.QPixmap("led-red-on.png"))


    # Current version of a_clicked only checks for one arduino.
    # Next version should add:
    #   - account for multiple arduinos 
    #   - check if the arduino is slave or servant

    @pyqtSlot()
    def a_clicked(self):

        # if self.ard_connected_LED == 0:

            # establish connection with RPi
            # self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            # text, okPressed = QInputDialog.getText(self, "Connecting to RPi","Type the IP address of your RPi (ie. 10.103.215.103) or type n to disregard", QLineEdit.Normal, "")
            # if okPressed and text != '':
            #     self.port = 5010
            #     self.host = text
            # else:
            #     self.host = 'n'
            # if self.host != 'n':
            #     try:
            #         self.s.connect((self.host,self.port))
            #         self.ui.arduino_on.setPixmap(QtGui.QPixmap("green-led-on.png"))
            #         self.ard_connected_LED = 1
            #         self.ardorrasp = 'r'
            #     except socket.error:
            #         print('connection not established. trying to connect to Arduino')
            #         if self.ard_connected_LED == 0:
            #             port_avail = list(com_read.comports())
            #             for p in port_avail:
            #                 if "Ard" in p[1] or "USB" in p[1]:
            #                     COM = p[0]

                        # try:
                        #     self.comPort = serial.Serial(COM)
                        #     self.ui.arduino_on.setPixmap(QtGui.QPixmap("green-led-on.png"))
                        #     self.ard_connected_LED = 1
                        #     self.ui.startbutton.setDisabled(True)
                        #     QTimer.singleShot(1200, lambda: self.ui.startbutton.setDisabled(False))
                        #     self.ardorrasp = 'a'
                        # except serial.SerialException:
                        #     self.error_dialog.setText("There is a problem connecting to the ComPort. Try reconnecting the USB")
                        #     self.error_dialog.show()
                        # except NameError:
                        #     self.error_dialog.setText("There is no arduino available")
                        #     self.error_dialog.show()
            # else:
            #     port_avail = list(com_read.comports())
            #     for p in port_avail:
            #         if "Ard" in p[1] or "USB" in p[1]:
            #             COM = p[0]

                # try:
                #     self.comPort = serial.Serial(COM)
                #     self.ui.arduino_on.setPixmap(QtGui.QPixmap("green-led-on.png"))
                #     self.ard_connected_LED = 1
                #     self.ui.startbutton.setDisabled(True)
                #     QTimer.singleShot(1200, lambda: self.ui.startbutton.setDisabled(False))
                #     self.ardorrasp = 'a'
                # except serial.SerialException:
                #     self.error_dialog.setText("There is a problem connecting to the ComPort. Try reconnecting the USB")
                #     self.error_dialog.show()
                # except NameError:
                #     self.error_dialog.setText("There is no arduino available")
                #     self.error_dialog.show()

        else:
            try:
                if self.ardorrasp == 'r':
                    self.s.send('END')
                    # self.s.close()
                    # self.ui.arduino_on.setPixmap(QtGui.QPixmap("led-red-on.png"))
                    # self.ard_connected_LED = 0
                # else:
                #     self.comPort.close()
                #     self.ui.arduino_on.setPixmap(QtGui.QPixmap("led-red-on.png"))
                #     self.ard_connected_LED = 0
            # except NameError:
            #     print('self.comPort not created yet')
            #     self.ui.arduino_on.setPixmap(QtGui.QPixmap("led-red-on.png"))
            #     self.ard_connected_LED = 0
            # except AttributeError:
            #     print('self.comPort not created yet')
            #     self.ui.arduino_on.setPixmap(QtGui.QPixmap("led-red-on.png"))
            #     self.ard_connected_LED = 0




    # Current version of c_clicked only connects  one camera.
    # Next version should add:
    #   - account for 2 cameras 

    # @pyqtSlot()
    # def c_clicked(self):

    #     if self.cam_connected_LED == 0:

            # def temp_enablebtns():
            #     self.ui.startbutton.setDisabled(False)
            #     self.ui.preview.setDisabled(False)

            # try:
                # # Setting Video Mode
                # vidMOD = 0x604
                # MODE_0 = 0x00000000 #Full resolution
                # MODE_1 = 0x20000000 #Half resolution
                # MODE_5 = 0xA0000000 #Quarter resolution

                #  # Switching on Strobe Mode
                # GPIO_pin2_setting = 0x1130
                # GPIO_pin2_outVal = 0x80080001
                # self.pin2_strobecnt = 0x1508

                # self.StrobeOff = 0x80000000
                # self.StrobeOn = 0x82000000

            #     self.bus = PyCapture2.BusManager()
            #     self.c = PyCapture2.Camera()
            #     self.c.connect(self.bus.getCameraFromIndex(0))

            #     self.ui.startbutton.setDisabled(True)
            #     self.ui.preview.setDisabled(True)
            #     # Setting Frame Rate
            #     self.c.setProperty(type = PyCapture2.PROPERTY_TYPE.FRAME_RATE, autoManualMode = False)
            #     self.c.setProperty(type = PyCapture2.PROPERTY_TYPE.FRAME_RATE, absControl = True)
            #     self.c.setProperty(type = PyCapture2.PROPERTY_TYPE.FRAME_RATE, absValue = 5.0)

            #     self.fRateProp = self.c.getProperty(PyCapture2.PROPERTY_TYPE.FRAME_RATE)
            #     self.frame_rate = self.fRateProp.absValue

            #     self.c.writeRegister(vidMOD, MODE_1)

            #     self.c.writeRegister(GPIO_pin2_setting, GPIO_pin2_outVal)
            #     self.c.writeRegister(self.pin2_strobecnt, self.StrobeOff)

            #     QTimer.singleShot(1200, temp_enablebtns)

            #     self.ui.cam_on.setPixmap(QtGui.QPixmap("green-led-on.png"))
            #     self.cam_connected_LED = 1
            #     self.cam1on = 1


            # except PyCapture2.Fc2error:
            #     self.error_dialog.setText("There is a problem connecting to the Camera. Try reconnecting the USB")
            #     self.error_dialog.show()


            # Camera 2 connection attempt
        #     try:
        #         self.c2 = PyCapture2.Camera()
        #         self.c2.connect(self.bus.getCameraFromIndex(1))

        #         self.c2.setProperty(type = PyCapture2.PROPERTY_TYPE.FRAME_RATE, autoManualMode = False)
        #         self.c2.setProperty(type = PyCapture2.PROPERTY_TYPE.FRAME_RATE, absControl = True)
        #         self.c2.setProperty(type = PyCapture2.PROPERTY_TYPE.FRAME_RATE, absValue = 5.0)
        #         self.c2.writeRegister(vidMOD, MODE_1)

        #         self.c2.writeRegister(GPIO_pin2_setting, GPIO_pin2_outVal)
        #         self.c2.writeRegister(self.pin2_strobecnt, self.StrobeOff)

        #         self.cam2on = 1

        #     except PyCapture2.Fc2error:
        #         self.error_dialog.setText("There is a problem connecting to the Camera 2. Try reconnecting the USB")
        #         self.error_dialog.show()


        # else:
        #     try:
        #         if self.cam1on == 1:
        #             self.c.disconnect()
        #             self.cam1on = 0
        #         if self.cam2on == 1:
        #             self.c2.disconnect()
        #             self.cam2on = 0
        #         self.ui.cam_on.setPixmap(QtGui.QPixmap("led-red-on.png"))
        #         self.cam_connected_LED = 0
        #     except PyCapture2.Fc2error:
        #         self.error_dialog.setText("The camera is currently in use")
        #         self.error_dialog.show()


    # def closeEvent(self, *args, **kwargs):
    #     super(QtGui.QMainWindow, self).closeEvent(*args, **kwargs)
    #     try:
    #         self.comPort.close()
    #     except AttributeError:
    #         pass
    #     try:
    #         self.c.disconnect()
    #         self.c2.disconnect()
    #     except AttributeError:
    #         pass
    #     except PyCapture2.Fc2error:
    #         print("There was a problem disconnecting the camera")

    #     self.ui.graphicsPlot.close()





if __name__ == "__main__":

    # Create the QT application
    app = QtWidgets.QApplication(sys.argv)

    # Create the main window
    win = controlBoard()

    # Show window
    win.show()

    # Disable window resizing
    win.setFixedSize(win.size())

    # Start QT application
    sys.exit(app.exec_())


