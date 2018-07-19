"""
Author: Justin Baik
Date created: 11/28/2017
Title: run_experiment with GUI control
Notes:

This is a GUI program used to do the following:
- keeps track of mice information (one system can run upto 4 mice at the same time). The GUI defines how many mice are run and creates ID for each mouse.
- start and stop EEG and EMG recording (using intan RHD2000) with user defined inputs. User can set the duration of the experiment and also set
the delay to start after a certain time.
- runs on different modes: open loop and closed loop. Open loop mode sends signals to either Arduino or RPi to trigger the laser. Closed loop mode
works only with the raspberry pi using socket communication. Because pi is prone to C related crashes, only using arduino for now (on open loop). 
- creates and move info txt file that is unique to the experiment with the user input settings. This txt later gets used to post-process the raw data.
- real-time spectral analysis of the EEG signal by reading off from the raw data that gets updated upon the start of an experiment. This spectral
data gets plotted on a second window.


This program is sort of a wrapper for the DAQ software that came with the Intan system. This program talks to the DAQ board via arduino or pi by sending
serial commands. The DAQ board has a trigger input that starts recording when it has a high input and stops when the signal shuts down. The DAQ software
also has a GUI to display the input signals from amplifiers / digital inputs / analog inputs. Right now, I am only using the amplifier signals for the
EEG and EMG, and the digital inputs for getting the camera strobe signals, laser trigger signals, the onoff signal that triggers DAQ recording.
"""

from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtCore import QObject, QThread, pyqtSignal, pyqtSlot, QTimer
from PyQt5.QtWidgets import QFileDialog, QInputDialog, QLineEdit, QMessageBox
import pyqtgraph as pg
from run_GUI_v9 import Ui_MainWindow
import sleepy
import socket
import numpy as np
import scipy
import PyCapture2
import cv2
import serial
import serial.tools.list_ports as com_read
import time
import sys
import shutil
import os
import glob
import pdb

######## functions used for the real time spectral analysis of EEG/EMG signals ########

def power_spectrum(data, length, dt):
    """
    scipy's implementation of Welch's method using hanning window to estimate
    the power spectrum
    @Parameters
        data    -   time series; float vector!
        length  -   length of hanning window, even integer!
    
    @Return:
        powerspectrum, frequencies
    """
    f, pxx = scipy.signal.welch(data, fs=1/dt, window='hanning', nperseg=int(length), noverlap=int(length/2))
    return pxx, f

def smooth_data(x, sig):
    """
    y = smooth_data(x, sig)
    smooth data vector @x with gaussian kernal
    with standard deviation $sig
    """
    sig = float(sig)
    if sig == 0.0:
        return x
        
    # gaussian:
    gauss = lambda (x, sig) : (1/(sig*np.sqrt(2.*np.pi)))*np.exp(-(x*x)/(2.*sig*sig))

    p = 1000000000
    L = 10.
    while (p > p):
        L = L+10
        p = gauss((L, sig))

    F = map(lambda (x): gauss((x, sig)), np.arange(-L, L+1.))
    F = F / np.sum(F)
    
    return scipy.signal.fftconvolve(x, F, 'same')

def recursive_spectrogram(EEG, EMG, sf=0.3, alpha=0.3):

    len_eeg = len(EEG)
    fdt = 2.5
    SR = 1000.0
    swin = int(np.round(SR)*5.0)
    swinh = int(swin/2.0)
    fft_win = int(swin/5.0)
    spoints = int(np.floor(len_eeg/swinh))

    # SE = np.zeros((fft_win/2+1, spoints))
    # SM = np.zeros((fft_win/2+1, spoints))


    x_e = EEG[0:swin]
    [p_e,f_e] = power_spectrum(x_e.astype('float'), fft_win, 1.0/SR)
    p_e = smooth_data(p_e,sf)
    # SE[:,i] = alpha*p + (1-alpha) * SE[:,i-1]

    x_m = EMG[0:swin]
    [p_m,f_m] = power_spectrum(x_m.astype('float'), fft_win, 1.0/SR)
    p_m = smooth_data(p_m,sf)
    # SM[:,i] = alpha*p + (1-alpha) * SM[:,i-1]

    return p_e, p_m, f_e, f_m





######## Main QThread objects and functions ########

def time2display(time_rem):
    tm,ts = np.divmod(int(time_rem), 60)
    th,tm = np.divmod(tm,60)
    if len(str(tm)) == 1:
        minute = "0" + str(tm)
    else:
        minute = str(tm)    
    if len(str(ts)) == 1:
        second = "0" + str(ts)
    else:
        second = str(ts)
    time_display = str(th) + ":" + minute + ":" + second
    return time_display

class worker_countdown(QObject):
    """
    QThread object that runs timer for the experiment. 
    This object is crucial for displaying the time count down until the experiment finished
    Initiated with start button (connected to start_clicked).

    On Delay:
        Time ticks until delay is finished, then goes into the actual count down.
        This function allows the actual data acquisition to start after a certain delay
    When Delay is finished (or if delay is set as 0):
        Still goes through the same process of finishing (ptimeDone) the delay mode,
        then goes into the count down

    """

    signal_countdownDisplay = pyqtSignal(float, str)
    signal_done = pyqtSignal()
    signal_delayDone = pyqtSignal()


    def __init__(self, start_time = 0, expdur = 0, delay = 0):
        super(worker_countdown, self).__init__()
        self.countdown_on = False
        self.start_time = start_time
        self.expdur = expdur
        self.delay = delay
        self.delay_finished = False


    @pyqtSlot()
    def run_countdown(self):
        while self.countdown_on:
            time_rem = (self.expdur*60*60 + self.delay*60) - (time.time()-self.start_time)
            delay_rem = (self.delay*60) - (time.time()-self.start_time)

            # send delay done signal without ol mode
            if time_rem - self.expdur*60*60 < 0.001 and not self.delay_finished:
                self.delay_finished = True
                self.signal_delayDone.emit()

            # Emit signal for time remaining to be displayed
            if self.delay_finished:
                self.signal_countdownDisplay.emit(time_rem, time2display(time_rem))

            else:
                self.signal_countdownDisplay.emit(time_rem, "D: " + time2display(delay_rem))

            # Finish countdown
            if time_rem < 0.001:
                self.signal_done.emit()

            time.sleep(0.1)

class worker_pcountdown(QObject):
    """
    QThread object that run when the open-loop mode has been turned on. This keeps track of
    time gaps between pulse trains (properties of the pulse trains are set using GUI inputs).
    Works very similarly to TotTime_counter, but this one runs a separate counter to count down
    time for the open-loop pulses.

    This object is supposed to close some time before the experiment ends (i.e. when the
    stop_clicked function is run to stop the recording)
    """

    signal_pcountdownDisplay = pyqtSignal(float, str)
    signal_firePulses = pyqtSignal()

    def __init__(self, expdur = 0 , puldur = 0, pt_start = 0, minpg = 15, maxpg = 25):
        super(worker_pcountdown, self).__init__()
        self.pcountdown_on = False
        self.pstart_time = pt_start
        self.puldur = puldur
        self.minpg = minpg
        self.maxpg = maxpg
        

    def setPulgap(self):
        if self.minpg*60 < self.puldur:
            pulGap = int(np.random.uniform(self.puldur+10, self.maxpg*60))
        elif self.maxpg*60 < self.puldur:
            pulGap = int(np.random.uniform(self.puldur+10, self.puldur+60))
        else:
            pulGap = int(np.random.uniform(self.minpg, self.maxpg)*60)
        return pulGap


    def run_pcount(self):
        self.pulGap = self.setPulgap()
        while self.pcountdown_on:
            ptime_rem = self.pulGap - (time.time()-self.pstart_time)
            if ptime_rem < 0.001:
                self.signal_firePulses.emit()
                # reset pulse gap
                self.pstart_time = time.time()
                self.pulGap = self.setPulgap()

            self.signal_pcountdownDisplay.emit(ptime_rem, time2display(ptime_rem))

            time.sleep(0.2)

class worker_camera(QObject):
    """
    QThread object that runs the camera. Its signals display camera frames on the GUI and it also writes
    video frames to an MKV stack.

    One recently aadded feature is the self.f and self.f2 that writes timestamps for the video frames in 
    a txt file.

    Because of the way PyCapture2 package generates the image buffers (into a long list of pixel values), there is a bunch of numpy flipping
    and transposing to get the image shape write to display it properly. I am guessing this is where the
    inefficiency is coming from as each line requires new memory.

    Also, there are two cameras in the same QObject that run at the same time. There should really be one object for
    each camera. The logic behind defining this class as such was that I tried doing so, but failed to create two
    individual video files. This was probably because I didn't use the right fourcc (some of the writer codecs can only be used one at a time. For example, XVID and H264.
    The only ones that worked were for writing from two cameras were MJPG and DIVX). It should work now with using the write codec, but needs to be revised.
    """

    signal_camFrame = pyqtSignal(str, np.ndarray, str, int)

    def __init__(self, preview_or_record, cam, cam_num):
        super(worker_camera, self).__init__()
        self.preview_or_record = preview_or_record
        self.cam = cam
        self.cam_num = cam_num
        self.camera_on = False
        self.writer = 0
        self.prev_timestamp = 0
        

    @pyqtSlot()
    def run_camera(self):
        """
        Commented areas for start_clicked
        """

        if self.preview_or_record == 'R':
            # self.f = open("cam1_timestamps.txt","w+")
            self.cam.writeRegister(0x1508, 0x82000000) #Turns strobe on

        elif self.preview_or_record == 'P':
            self.cam.writeRegister(0x1508, 0x80000000) #Turns strobe off

        while self.camera_on:
            try:
                image = self.cam.retrieveBuffer()
            except PyCapture2.Fc2error as fc2Err:
                print "Error retrieving buffer : ", fc2Err
                continue

            self.img_timestamp = image.getTimeStamp()
            if len(str(self.img_timestamp.microSeconds)) == 6:
                self.str_timestamp = str(self.img_timestamp.seconds)[-6:] + '.' + str(self.img_timestamp.microSeconds) + '\r\n'
            else:
                self.str_timestamp = str(self.img_timestamp.seconds)[-6:] + '.0' + str(self.img_timestamp.microSeconds) + '\r\n'

            imgdat = image.getData()
            imgnp = np.transpose(np.array(imgdat).reshape(image.getRows(), image.getCols()))
            self.prev_timestamp = self.img_timestamp

            self.signal_camFrame.emit(self.preview_or_record, imgnp, self.str_timestamp, self.cam_num)
            
            time.sleep(0.6)

class worker_buffer(QObject):
    """
    Thread object that retrieves data as it gets written from the DAQ board. Each loop cuts the
    data into 2.5 second bits (sampling rate runs on 1000 SPS), then sends this to a function to calculate
    spectrograms and to plot on another window. There is also a relay function that takes the data and identifies the 
    signal for each mouse. 

    Perhaps np.append is not the most efficient function to use here? 
    """

    signal_new5sec = pyqtSignal(dict, dict)

    def __init__(self, od, eidx, midx, switch = 0):
        super(worker_getdata, self).__init__()
        self.odir = od
        self.fid = open(self.odir + '/amplifier.dat', 'rb')
        self.tid = open(self.odir + '/time.dat', 'r')
        self.switch = switch
        self.new_data = np.fromfile(self.fid, dtype='int16')
        self.new_time = np.fromfile(self.tid, dtype='int32')
        self.eidx = eidx
        self.midx = midx
        self.numchans = 0
        for mouse in self.eidx:
            self.numchans += 4


        self.buffer_e = {k: np.ndarray(0) for k in eidx}
        self.tmp_key = self.buffer_e.keys()[0]
        self.buffer_m = {k: np.ndarray(0) for k in midx}


        # we need: eeg_idx, emg_idx, new5_e, new5_m, 
    def run_it(self):

        while self.switch == 1:
            self.new_data = np.fromfile(self.fid, dtype='int16')

            if len(self.new_data) > 1:
                for mouse in self.eidx:
                    self.buffer_e[mouse] = np.append(self.buffer_e[mouse], self.new_data[self.eidx[mouse]::self.numchans])
                    self.buffer_m[mouse] = np.append(self.buffer_m[mouse], self.new_data[self.midx[mouse]::self.numchans])

                if len(self.buffer_e[self.tmp_key]) >= 2500:
                    buffer_emitE = {}
                    buffer_emitM = {}
                    for mouse in self.eidx:
                        buffer_emitE[mouse] = self.buffer_e[mouse][:2500]
                        buffer_emitM[mouse] = self.buffer_m[mouse][:2500]
                    self.signal_new5sec.emit(buffer_emitE, buffer_emitM)
                    
                    for mouse in self.eidx:
                        self.buffer_e[mouse] = self.buffer_e[mouse][2500:]
                        self.buffer_m[mouse] = self.buffer_m[mouse][2500:]

            time.sleep(0.01)



######## Core GUI class ########

class main(QtWidgets.QMainWindow):
    """
    This class contains every bit of information that the GUI needs to process information.
    """

    def __init__(self):

        super(main, self).__init__()
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)
        self.setWindowIcon(QtGui.QIcon('sleepm.png'))
        self.define_variables()
        self.connect_signals()

        #### Start from default setup ####
        self.disable_comment(True)
        self.user_inputSetup()

    def define_variables(self):
        #### GUI placeholder variables
        self.cam1 = None
        self.cam2 = None

        #### Flir Camera settings
        self.bus = PyCapture2.BusManager()
        # Video Modes
        self.vidMOD = 0x604
        self.MODE_0 = 0x00000000 #Full resolution
        self.MODE_1 = 0x20000000 #Half resolution
        self.MODE_5 = 0xA0000000 #Quarter resolution
        # Switching on Strobe Mode
        self.GPIO_pin2_setting = 0x1130
        self.GPIO_pin2_outVal = 0x80080001
        self.pin2_strobecnt = 0x1508
        self.StrobeOff = 0x80000000
        self.StrobeOn = 0x82000000

        #### GUI status variables
        self.rpi_on = False
        self.ard_on = False
        self.cam1_connected = False
        self.cam2_connected = False
        self.start_on = False
        self.preview_on = False
        self.trigger_on = False

        #### GUI input variable dictionaries ####
        ## Protocol ##
        ## QRadioButton
        # setChecked() - mark checked 
        # isChecked() - checks state and returns True or False
        self.protocols = {}
        self.protocols['option_default'] = self.ui.option_default
        self.protocols['option_ol'] = self.ui.option_ol
        self.protocols['option_cl'] = self.ui.option_cl
        self.protocols['option_remdep'] = self.ui.option_remdep
        self.protocols['option_nremdep'] = self.ui.option_nremdep
        
        ## Parameters ##
        ## QSpinBox / QDoubleSpinBox (integer / double)
        # value() - returns the value of the spinbox input
        self.parameters = {}
        self.parameters['sr'] = self.ui.sr
        self.parameters['delay'] = self.ui.delay
        self.parameters['expdur'] = self.ui.expdur
        self.parameters['pulsedur'] = self.ui.pulsedur
        self.parameters['hi'] = self.ui.hi
        self.parameters['lo'] = self.ui.lo
        self.parameters['maxPG'] = self.ui.maxPG
        self.parameters['minPG'] = self.ui.minPG

        ## Mouse ID ##
        ## QCheckBox
        # checkState() - returns 2 if true (different from QRadioButton)
        # setChecked() - mark checked
        ## QLineEdit
        # text() - returns the input text
        # clear() - clears the input text
        self.mouseIDs = {}
        self.mouseIDs['m1chk'] = self.ui.m1chk
        self.mouseIDs['m2chk'] = self.ui.m2chk
        self.mouseIDs['m3chk'] = self.ui.m3chk
        self.mouseIDs['m4chk'] = self.ui.m4chk
        self.mouseIDs['id1'] = self.ui.id1
        self.mouseIDs['id2'] = self.ui.id2
        self.mouseIDs['id3'] = self.ui.id3
        self.mouseIDs['id4'] = self.ui.id4
        self.mouseIDs['ch1'] = self.ui.ch1
        self.mouseIDs['ch2'] = self.ui.ch2
        self.mouseIDs['ch3'] = self.ui.ch3
        self.mouseIDs['ch4'] = self.ui.ch4
        self.mouseIDs['l1'] = self.ui.l1
        self.mouseIDs['l2'] = self.ui.l2
        self.mouseIDs['l3'] = self.ui.l3
        self.mouseIDs['l4'] = self.ui.l4

        ## Connect Devices ##
        self.devices = {}
        self.devices['connect_controller'] = self.ui.connect_controller
        self.devices['connect_c1'] = self.ui.connect_c1
        self.devices['connect_c2'] = self.ui.connect_c2
        self.devices['led_controller'] = self.ui.led_controller
        self.devices['led_cam1'] = self.ui.led_cam1
        self.devices['led_cam2'] = self.ui.led_cam2

        ## Control ##
        self.controls = {}
        self.controls['preview'] = self.ui.preview
        self.controls['startbutton'] = self.ui.startbutton
        self.controls['stopbutton'] = self.ui.stopbutton

        ## Pulse Control ##
        self.pulseControls = {}
        self.pulseControls['hi_c'] = self.ui.hi_c
        self.pulseControls['lo_c'] = self.ui.lo_c
        self.pulseControls['dur_c'] = self.ui.dur_c
        self.pulseControls['continuous'] = self.ui.continuous
        self.pulseControls['c_on'] = self.ui.c_on
        self.pulseControls['c_off'] = self.ui.c_off

        ## Comments ##
        ## QTextEdit
        # toPlainText() - returns the text in string
        # clear() - clears the input text
        self.commentItems = {}
        self.commentItems['commentHist'] = self.ui.commentHist
        self.commentItems['commentbox'] = self.ui.commentbox
        self.commentItems['entercomment'] = self.ui.entercomment

        ## Cameras ## 
        ## pg.GraphicsView
        ## pg.GraphicsLayoutWidget
        # setImage(buffer)
        self.cam1view = self.ui.cam1view
        self.cam2view = self.ui.cam2view
        self.graphicsView = self.ui.graphicsView

        ## Thresholds ##
        self.thresholds = {}
        self.thresholds['threshDelta1'] = self.ui.threshDelta1
        self.thresholds['threshDelta2'] = self.ui.threshDelta2
        self.thresholds['threshDelta3'] = self.ui.threshDelta3
        self.thresholds['threshDelta4'] = self.ui.threshDelta4

        self.thresholds['threshUDelta1'] = self.ui.threshUDelta1
        self.thresholds['threshUDelta2'] = self.ui.threshUDelta2
        self.thresholds['threshUDelta3'] = self.ui.threshUDelta3
        self.thresholds['threshUDelta4'] = self.ui.threshUDelta4

        self.thresholds['threshLDelta1'] = self.ui.threshLDelta1
        self.thresholds['threshLDelta2'] = self.ui.threshLDelta2
        self.thresholds['threshLDelta3'] = self.ui.threshLDelta3
        self.thresholds['threshLDelta4'] = self.ui.threshLDelta4

        self.thresholds['threshThd1'] = self.ui.threshThd1
        self.thresholds['threshThd2'] = self.ui.threshThd2
        self.thresholds['threshThd3'] = self.ui.threshThd3
        self.thresholds['threshThd4'] = self.ui.threshThd4

        self.thresholds['threshThdl1'] = self.ui.threshThdl1
        self.thresholds['threshThdl2'] = self.ui.threshThdl2
        self.thresholds['threshThdl3'] = self.ui.threshThdl3
        self.thresholds['threshThdl4'] = self.ui.threshThdl4

        self.thresholds['threshMu1'] = self.ui.threshMu1
        self.thresholds['threshMu2'] = self.ui.threshMu2
        self.thresholds['threshMu3'] = self.ui.threshMu3
        self.thresholds['threshMu4'] = self.ui.threshMu4


        #### Error Dialog Handle ####
        self.error_dialog = QMessageBox()
        self.error_dialog.setIcon(QtWidgets.QMessageBox.Critical)
        self.error_dialog.setWindowTitle("Error")
        self.error_dialog.setWindowIcon(QtGui.QIcon('sleepm.png'))

    def connect_signals(self):
        #### Connecting Signals and Slots ####
        for protocol in self.protocols:
            self.protocols[protocol].clicked.connect(self.user_inputSetup)
        self.devices['connect_controller'].clicked.connect(self.comm_connect)
        self.devices['connect_c1'].clicked.connect(self.cam1_setup)
        self.devices['connect_c2'].clicked.connect(self.cam2_setup)
        self.controls['preview'].clicked.connect(self.preview_clicked)
        self.controls['startbutton'].clicked.connect(self.start_clicked)
        self.controls['stopbutton'].clicked.connect(self.stop_clicked)
        # self.pulseControls['c_on'].clicked.connect(self.pulon_clicked)
        # self.pulseControls['c_off'].clicked.connect(self.puloff_clicked)
        self.commentItems['entercomment'].clicked.connect(self.submit_comment)

    def disable_customPulse(self, disable):
        for item in self.pulseControls:
            self.pulseControls[item].setDisabled(disable)

    @pyqtSlot()
    def user_inputSetup(self):
        """Disable settings used for other protocols"""
        if self.protocols['option_default'].isChecked():
            self.parameters['pulsedur'].setDisabled(True)
            self.parameters['hi'].setDisabled(True)
            self.parameters['lo'].setDisabled(True)
            self.parameters['maxPG'].setDisabled(True)
            self.parameters['minPG'].setDisabled(True)
            for item in self.thresholds:
                self.thresholds[item].setDisabled(True)
            self.disable_customPulse(False)

        elif self.protocols['option_ol'].isChecked():
            self.parameters['pulsedur'].setDisabled(False)
            self.parameters['hi'].setDisabled(False)
            self.parameters['lo'].setDisabled(False)
            self.parameters['maxPG'].setDisabled(False)
            self.parameters['minPG'].setDisabled(False)
            for item in self.thresholds:
                self.thresholds[item].setDisabled(True)
            self.disable_customPulse(True)
        elif self.protocols['option_cl'].isChecked():
            self.parameters['pulsedur'].setDisabled(False)
            self.parameters['hi'].setDisabled(False)
            self.parameters['lo'].setDisabled(False)
            self.parameters['maxPG'].setDisabled(True)
            self.parameters['minPG'].setDisabled(True)
            for item in self.thresholds:
                self.thresholds[item].setDisabled(False)
            self.disable_customPulse(False)
        else:
            self.parameters['pulsedur'].setDisabled(True)
            self.parameters['hi'].setDisabled(True)
            self.parameters['lo'].setDisabled(True)
            self.parameters['maxPG'].setDisabled(True)
            self.parameters['minPG'].setDisabled(True)
            for item in self.thresholds:
                self.thresholds[item].setDisabled(False)
            self.disable_customPulse(False)







    def arduino_mode_on(self):
        port_avail = list(com_read.comports())
        for p in port_avail:
            if "Ard" in p[1] or "USB" in p[1]:
                COM = p[0]
        try:
            self.comPort = serial.Serial(COM)
            self.devices['led_controller'].setPixmap(QtGui.QPixmap("led_on"))
            self.ard_on = True
            self.controls['startbutton'].setDisabled(True)
            QTimer.singleShot(1200, lambda: self.controls['startbutton'].setDisabled(False))
        except ValueError:
            self.error_dialog.setText("There is a problem connecting to the ComPort. Try reconnecting the USB")
            self.error_dialog.show()
        except serial.SerialException:
            self.error_dialog.setText("There is a problem connecting to the ComPort. Try reconnecting the USB")
            self.error_dialog.show()
        except NameError:
            self.error_dialog.setText("There is no arduino available")
            self.error_dialog.show()

    def arduino_disconnect(self):
        if self.ard_on:
            self.comPort.close()
            self.devices['led_controller'].setPixmap(QtGui.QPixmap("led_off"))
            self.ard_on = False

    # def raspi_mode_on(self, ip):

    # def raspi_disconnect(self):

    @pyqtSlot()
    def comm_connect(self):
        if not self.ard_on and not self.rpi_on:
            ip_address, okPressed = QInputDialog.getText(self, "Connecting to RPi","Type the IP address of your RPi (ie. 10.103.215.103) or skip to disregard", QLineEdit.Normal, "")
            if okPressed:
                if ip_address == '':
                    self.arduino_mode_on()
                else:
                    self.raspi_mode_on(ip_address)
                    
        else:
            if self.ard_on:
                self.arduino_disconnect()
            elif self.rpi_on:
                self.raspi_disconnect()







    @pyqtSlot()
    def cam1_setup(self):
        if not self.cam1_connected:
            self.cam1 = PyCapture2.Camera()
            connection = self.cam_connect(0, self.cam1)
            if connection == 'success':
                self.devices['led_cam1'].setPixmap(QtGui.QPixmap("led_on"))
                self.cam1_connected = True
        else:
            try:
                self.cam1.disconnect()
                self.devices['led_cam1'].setPixmap(QtGui.QPixmap("led_off"))
                self.cam1_connected = False
            except:
                print("There was a problem disconnecting the camera 1")
                pass

    @pyqtSlot()
    def cam2_setup(self):
        if not self.cam2_connected:
            self.cam2 = PyCapture2.Camera()
            connection = self.cam_connect(1, self.cam2)
            if connection == 'success':
                self.devices['led_cam2'].setPixmap(QtGui.QPixmap("led_on"))
                self.cam2_connected = True
        else:
            try:
                self.cam2.disconnect()
                self.devices['led_cam2'].setPixmap(QtGui.QPixmap("led_off"))
                self.cam2_connected = False
            except:
                print("There was a problem disconnecting the camera 2")
                pass

    def cam_connect(self, cam_num, cam):

        def temp_enablebtns():
            self.controls['startbutton'].setDisabled(False)
            self.controls['stopbutton'].setDisabled(False)
            self.controls['preview'].setDisabled(False)

        try:
            
            cam.connect(self.bus.getCameraFromIndex(cam_num))

            self.controls['startbutton'].setDisabled(True)
            self.controls['stopbutton'].setDisabled(True)
            self.controls['preview'].setDisabled(True)
            # Setting Frame Rate
            cam.setProperty(type = PyCapture2.PROPERTY_TYPE.FRAME_RATE, autoManualMode = False)
            cam.setProperty(type = PyCapture2.PROPERTY_TYPE.FRAME_RATE, absControl = True)
            cam.setProperty(type = PyCapture2.PROPERTY_TYPE.FRAME_RATE, absValue = 5.0)

            self.fRateProp = cam.getProperty(PyCapture2.PROPERTY_TYPE.FRAME_RATE)
            self.frame_rate = self.fRateProp.absValue

            cam.writeRegister(self.vidMOD, self.MODE_1)

            cam.writeRegister(self.GPIO_pin2_setting, self.GPIO_pin2_outVal)
            cam.writeRegister(self.pin2_strobecnt, self.StrobeOff)

            QTimer.singleShot(1200, temp_enablebtns)

            return 'success'

        except PyCapture2.Fc2error:
            self.error_dialog.setText("There is a problem connecting to the Camera. Try reconnecting the USB in the right order.")
            self.error_dialog.show()

    def cams_disconnect(self):
        try:
            self.cam1.disconnect()
            self.cam2.disconnect()
            self.cam1_connected = False
            self.cam2_connected = False
        except AttributeError:
            pass

    def start_cameraThread(self, cam, record_or_preview, cam_num):
        camera_thread = QThread(self)
        camera_obj = worker_camera(record_or_preview, cam, cam_num)
        camera_obj.moveToThread(camera_thread)
        camera_obj.camera_on = True
        camera_obj.signal_camFrame.connect(self.process_vid)
        camera_thread.started.connect(camera_obj.run_camera)
        cam.startCapture()
        camera_thread.start()

        return camera_obj, camera_thread

    def stop_cameraThread(self, cam, cam_obj, cam_thread):
        cam.writeRegister(self.pin2_strobecnt, self.StrobeOff)
        cam_obj.camera_on = False
        cam.stopCapture()
        cam_thread.quit()
        cam_thread.wait()

    @pyqtSlot(str, np.ndarray, str, int)
    def process_vid(self, PoR, frame, timestamps, cam_num):

        if PoR == 'R':
            if cam_num == 1:
                if self.mouseIDs['m1chk'].checkState() == 2:
                    self.mkv_writer1.write(np.uint8(frame[0:322,:]))
                if self.mouseIDs['m2chk'].checkState() == 2:
                    self.mkv_writer2.write(np.uint8(frame[322:,:]))
            if cam_num == 2:
                if self.mouseIDs['m3chk'].checkState() == 2:
                    self.mkv_writer3.write(np.uint8(frame[0:322,:]))
                if self.mouseIDs['m4chk'].checkState() == 2:
                    self.mkv_writer4.write(np.uint8(frame[322:,:]))

        if cam_num == 1:
            self.cam1view.setImage(frame)
        if cam_num == 2:
            self.cam2view.setImage(frame)

    @pyqtSlot()
    def preview_clicked(self):
        if not self.cam1_connected and not self.cam2_connected:
            self.error_dialog.setText("You must connect the camera first!")
            self.error_dialog.show()

        elif not self.start_on and not self.preview_on:
            self.controls['preview'].setStyleSheet("background-color: green")
            self.preview_on = True

            for buttons in self.devices:
                if 'led' not in buttons:
                    self.devices[buttons].setDisabled(True)
            for buttons in self.controls:
                if 'preview' not in buttons:
                    self.controls[buttons].setDisabled(True)

            if self.cam1_connected:
                self.cam_obj1, self.cam_thread1 = self.start_cameraThread(self.cam1, 'P', 1)

            if self.cam2_connected:
                self.cam_obj2, self.cam_thread2 = self.start_cameraThread(self.cam2, 'P', 2)


        elif not self.start_on and self.preview_on:
            self.controls['preview'].setStyleSheet(self.controls['stopbutton'].styleSheet())
            self.preview_on = False

            for buttons in self.devices:
                self.devices[buttons].setDisabled(False)
            for buttons in self.controls:
                self.controls[buttons].setDisabled(False)

            if self.cam1_connected:
                self.stop_cameraThread(self.cam1, self.cam_obj1, self.cam_thread1)
                QTimer.singleShot(100, lambda: self.cam1view.clear())

            if self.cam2_connected:
                self.stop_cameraThread(self.cam2, self.cam_obj2, self.cam_thread2)
                QTimer.singleShot(100, lambda: self.cam2view.clear())

    def start_videoRec(self):
        self.fourcc = cv2.VideoWriter_fourcc(*'DIVX')
        if self.cam1_connected:
            if self.mouseIDs['m1chk'].checkState() == 2:
                self.mkv_writer1 = cv2.VideoWriter(self.ui.id1.text() + "_cam_1.mkv", self.fourcc, self.frame_rate, (482, 322), isColor=0)
            if self.mouseIDs['m2chk'].checkState() == 2:
                self.mkv_writer2 = cv2.VideoWriter(self.ui.id2.text() + "_cam_2.mkv", self.fourcc, self.frame_rate, (482, 322), isColor=0)
            self.cam_obj1, self.cam_thread1 = self.start_cameraThread(self.cam1, 'R', 1)

        if self.cam2_connected:
            if self.mouseIDs['m3chk'].checkState() == 2:
                self.mkv_writer3 = cv2.VideoWriter(self.ui.id3.text() + "_cam_3.mkv", self.fourcc, self.frame_rate, (482, 322), isColor=0)
            if self.mouseIDs['m4chk'].checkState() == 2:
                self.mkv_writer4 = cv2.VideoWriter(self.ui.id4.text() + "_cam_4.mkv", self.fourcc, self.frame_rate, (482, 322), isColor=0)
            self.cam_obj2, self.cam_thread2 = self.start_cameraThread(self.cam2, 'R', 2)

    def end_videoRec(self):
        if self.cam1_connected:
            self.stop_cameraThread(self.cam1, self.cam_obj1, self.cam_thread1)
            if self.mouseIDs['m1chk'].checkState() == 2:
                self.mkv_writer1.release()
            if self.mouseIDs['m2chk'].checkState() == 2:
                self.mkv_writer2.release()
            QTimer.singleShot(100, lambda: self.cam1view.clear())

        if self.cam2_connected:
            self.stop_cameraThread(self.cam2, self.cam_obj2, self.cam_thread2)
            if self.mouseIDs['m3chk'].checkState() == 2:
                self.mkv_writer3.release()
            if self.mouseIDs['m4chk'].checkState() == 2:
                self.mkv_writer4.release()
            QTimer.singleShot(100, lambda: self.cam2view.clear())

   




    def create_mouselist(self):
        mouselist = []
        if self.mouseIDs['m1chk'].checkState() == 2:
            mouselist.append(self.mouseIDs['id1'].text())
        if self.mouseIDs['m2chk'].checkState() == 2:
            mouselist.append(self.mouseIDs['id2'].text())
        if self.mouseIDs['m3chk'].checkState() == 2:
            mouselist.append(self.mouseIDs['id3'].text())
        if self.mouseIDs['m4chk'].checkState() == 2:
            mouselist.append(self.mouseIDs['id4'].text())
        return mouselist, not bool(mouselist)

    def disable_input(self, disable):
            for protocol in self.protocols:
                self.protocols[protocol].setDisabled(disable)
            for connection in self.devices:
                if 'led' not in connection:
                    self.devices[connection].setDisabled(disable)
            for param in self.parameters:
                self.parameters[param].setDisabled(disable)
            for prop in self.mouseIDs:
                self.mouseIDs[prop].setDisabled(disable)
            if not disable:
                self.user_inputSetup()

    def getTime(self):

        # Create prefix for all files using time library. date_time = "YearMonthDate_HrMinSec", date = "YearMonthDate"

        date_time = str(time.localtime().tm_year)[2:4]
        date = ''
        

        if len(str(time.localtime().tm_mon)) < 2:
            date_time += '0' + str(time.localtime().tm_mon)
            date += '0' + str(time.localtime().tm_mon) + '/'
        else:
            date_time += str(time.localtime().tm_mon)
            date += str(time.localtime().tm_mon) + '/'

        if len(str(time.localtime().tm_mday)) < 2:
            date_time += '0' + str(time.localtime().tm_mday)
            date += '0' + str(time.localtime().tm_mday) + '/'
        else:
            date_time += str(time.localtime().tm_mday)
            date += str(time.localtime().tm_mday) + '/'

        date += str(time.localtime().tm_year)[2:4]

        date_time += str('_')
        if len(str(time.localtime().tm_hour)) < 2:
            date_time += '0' + str(time.localtime().tm_hour)
        else:
            date_time += str(time.localtime().tm_hour)

        if len(str(time.localtime().tm_min)) < 2:
            date_time += '0' + str(time.localtime().tm_min)
        else:
            date_time += str(time.localtime().tm_min)
        if len(str(time.localtime().tm_sec)) < 2:
            date_time += '0' + str(time.localtime().tm_sec)
        else:
            date_time += str(time.localtime().tm_sec)

        time_init = time.localtime()
        curr_time = str(time_init.tm_hour) + ':' + str(time_init.tm_min) + ':' + str(time_init.tm_sec)

        return curr_time, date, date_time

    def setupNotes(self):

        self.timetxt, self.todate, self.todate_full = self.getTime()

        # Writes a txt file to include all experiment setting and variables

        self.txt = open(self.todate_full + "_notes.txt","w+")
        self.txt.write('date:\t' + self.todate + '\r\n')
        self.txt.write('time:\t' + self.timetxt + '\r\n')
        self.txt.write('amplifier:\t' + 'intan' + '\r\n')
        self.txt.write('SR:\t' + str(self.parameters['sr'].value()) + '\r\n')
        self.txt.write('delay:\t' + str(self.parameters['delay'].value()) + '\r\n')
        if self.protocols['option_ol'].isChecked():
            self.txt.write('mode:\t' + 'ol\r\n')
        elif self.protocols['option_cl'].isChecked():
            self.txt.write('mode:\t' + 'cl\r\n')
        elif self.protocols['option_remdep'].isChecked():
            self.txt.write('mode:\t' + 'remdep\r\n')
        elif self.protocols['option_nremdep'].isChecked():
            self.txt.write('mode:\t' + 'nremdep\r\n')
        else:
            self.txt.write('mode:\t' + 'default\r\n')

        if self.protocols['option_ol'].isChecked() or self.protocols['option_cl'].isChecked():
            self.txt.write('laser_hi:\t' + str(self.parameters['hi'].value()) + '\r\n')
            self.txt.write('laser_lo:\t' + str(self.parameters['lo'].value()) + '\r\n')
            self.txt.write('stim_freq:\t' + str(int(1.0/(self.parameters['hi'].value()+self.parameters['lo'].value())/0.001)) + '\r\n')
            if self.protocols['option_ol'].isChecked():
                self.txt.write('laser_dur:\t' + str(self.parameters['pulsedur'].value()) + '\r\n')
        self.txt.write('exp_dur:\t' + str(self.parameters['expdur'].value()) + '\r\n')
        self.txt.write('mouse_ID:\t')
        if self.mouseIDs['m1chk'].checkState() == 2:
            self.txt.write(self.mouseIDs['id1'].text() + '\t')
        else:
            self.txt.write('\t ')
        if self.mouseIDs['m2chk'].checkState() == 2:
            self.txt.write(self.mouseIDs['id2'].text() + '\t')
        else:
            self.txt.write('\t ')
        if self.mouseIDs['m3chk'].checkState() == 2:
            self.txt.write(self.mouseIDs['id3'].text() + '\t')
        else:
            self.txt.write('\t ')
        if self.mouseIDs['m4chk'].checkState() == 2:
            self.txt.write(self.mouseIDs['id4'].text() + '\t')
        else:
            self.txt.write('\t ')

        self.txt.write('\r\n')
        self.txt.write('ch_alloc:\t')
        if self.mouseIDs['m1chk'].checkState() == 2:
            self.txt.write(self.mouseIDs['ch1'].text() + '\t')
        if self.mouseIDs['m2chk'].checkState() == 2:
            self.txt.write(self.mouseIDs['ch2'].text() + '\t')
        if self.mouseIDs['m3chk'].checkState() == 2:
            self.txt.write(self.mouseIDs['ch3'].text() + '\t')
        if self.mouseIDs['m4chk'].checkState() == 2:
            self.txt.write(self.mouseIDs['ch4'].text() + '\t')
        
        self.txt.write('\r\n')
        self.txt.write('laser_used:\t')
        if self.mouseIDs['m1chk'].checkState() == 2:
            self.txt.write(self.mouseIDs['l1'].text() + '\t')
        if self.mouseIDs['m2chk'].checkState() == 2:
            self.txt.write(self.mouseIDs['l2'].text() + '\t')
        if self.mouseIDs['m3chk'].checkState() == 2:
            self.txt.write(self.mouseIDs['l3'].text() + '\t')
        if self.mouseIDs['m4chk'].checkState() == 2:
            self.txt.write(self.mouseIDs['l4'].text() + '\t')

        self.txt.write('\r\n')
        self.txt.write('#Notes:\t')
        self.txt.close()

    def endNotes(self):
        self.txt = open(self.todate_full + "_notes.txt", 'a')
        self.txt.write('\r\n')
        self.totRec_time = int(self.ui.expdur.value()*60*60) - self.time_rem
        if self.totRec_time > 0:
            self.mr,self.srem = np.divmod(self.totRec_time, 60)
            self.hr,self.mr = np.divmod(self.mr,60)
            self.txt.write('actual_duration:\t'+ str(int(self.hr)) + 'h:' + str(int(self.mr)) + 'm:' + str(int(self.srem)) + 's')
            self.txt.close()
        else:
            self.txt.write('actual_duration:\t'+ '0h:00m:00s')
            self.txt.close()








    def setup_countdown(self):
        timer_thread = QThread(self)
        timer_obj = worker_countdown(0, self.ui.expdur.value(), self.ui.delay.value())
        timer_obj.moveToThread(timer_thread)
        timer_obj.signal_countdownDisplay.connect(self.update_countdown)
        timer_obj.signal_done.connect(self.stop_clicked)
        timer_obj.signal_delayDone.connect(self.beginProtocol)
        timer_thread.started.connect(timer_obj.run_countdown)
        return timer_obj, timer_thread

    def end_countdown(self):
        self.countdown_obj.countdown_on = False
        self.countdown_thread.quit()
        self.countdown_thread.wait()
        self.update_countdown()

        if self.trigger_on:
            self.trigger_on = False
            self.comPort.write(bytearray(b'E\n'))

    @pyqtSlot(float, str)
    def update_countdown(self, time_rem = 0, time_display = '0'):
        self.time_rem = time_rem
        if self.countdown_obj.countdown_on:
            self.ui.t_rem.setText(QtCore.QCoreApplication.translate("MainWindow", time_display))
        else:
            self.ui.t_rem.setText(QtCore.QCoreApplication.translate("MainWindow", "IDLE"))

    def setup_pcountdown(self):
        ptimer_thread = QThread(self)
        ptimer_obj = worker_pcountdown(self.ui.expdur.value(), self.ui.pulsedur.value(), time.time(), self.ui.minPG.value(), self.ui.maxPG.value())
        ptimer_obj.moveToThread(ptimer_thread)
        ptimer_obj.pcountdown_on = True
        ptimer_obj.signal_pcountdownDisplay.connect(self.update_pcountdown)
        ptimer_obj.signal_firePulses.connect(self.ol_firePulses)
        ptimer_thread.started.connect(ptimer_obj.run_pcount)
        ptimer_thread.start()
        return ptimer_obj, ptimer_thread

    def end_pcountdown(self):
        self.pcountdown_obj.pcountdown_on = False
        self.pcountdown_thread.quit()
        self.pcountdown_thread.wait()
        self.update_pcountdown()

    def ol_firePulses(self):
        if self.ard_on:
            self.comPort.write(bytearray(b'R\n'))
        elif self.rpi_on:
            # add lines here #
            pass

    @pyqtSlot(float, str)
    def update_pcountdown(self, ptime_rem = 0, ptime_display = '0'):
        if self.pcountdown_obj.pcountdown_on and self.time_rem < self.ui.maxPG.value()*60 + 20:
            self.end_pcountdown()
        if self.pcountdown_obj.pcountdown_on:
            self.ui.p_rem.setText(QtCore.QCoreApplication.translate("MainWindow", ptime_display))
        else:
            self.ui.p_rem.setText(QtCore.QCoreApplication.translate("MainWindow", "IDLE"))







    def setupProtocol(self):
        if self.ard_on and self.protocols['option_ol'].isChecked:
            try:
                self.comPort.write(bytearray(b'H' + str(self.parameters['hi'].value()) + '\n'))
                self.comPort.write(bytearray(b'L' + str(self.parameters['lo'].value()) + '\n'))
                self.comPort.write(bytearray(b'D' + str(self.parameters['pulsedur'].value()) + '\n'))
                if self.parameters['delay'].value() != 0:
                    self.ui.p_rem.setText(QtCore.QCoreApplication.translate("MainWindow", "On Delay"))
            except:
                self.error_dialog.setText("Pulse information tranfer failed")
                self.error_dialog.show()

        # elif self.rpi_on:

        self.countdown_obj, self.countdown_thread = self.setup_countdown()
        self.countdown_obj.countdown_on = True
        self.countdown_obj.start_time = time.time()
        self.countdown_thread.start()

    @pyqtSlot()
    def beginProtocol(self):
        
        self.run_default()

        if self.protocols['option_ol'].isChecked():
            self.pcountdown_obj, self.pcountdown_thread = self.setup_pcountdown()
        elif self.protocols['option_cl'].isChecked():
            self.run_cl()
        elif self.protocols['option_remdep'].isChecked():
            self.run_cl()
        elif self.protocols['option_nremdep'].isChecked():
            self.run_cl()

    def run_default(self):
        if self.ard_on:
            self.comPort.write(bytearray(b'S\n'))
            self.trigger_on = True
        elif self.rpi_on:
            # add lines here for rpi #
            pass
        
        
    # def run_cl(self):

    @pyqtSlot()
    def endProtocol(self):

        self.end_countdown()

        if self.protocols['option_ol'].isChecked():
            self.end_pcountdown()
        elif self.protocols['option_cl'].isChecked():
            self.end_cl()
        elif self.protocols['option_remdep'].isChecked():
            self.end_dep()
        elif self.protocols['option_nremdep'].isChecked():
            self.end_dep()


    # def end_cl():


    def run_spectral(self):

        try:
            # rem state on/off
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

            self.prem = {k: 0 for k in self.mouselist}
            self.pow_muh = {k: [] for k in self.mouselist}
            self.rem_hist = {k: [] for k in self.mouselist}
            self.past_len = {k: int(120/2.5) for k in self.mouselist}
            self.num_iter = {k: 0 for k in self.mouselist}

            # Range values for each power
            self.r_delta = [0.5, 4]
            self.r_theta = [5,12]
            self.r_mu = [300, 500]

            self.alpha = 0.3

            self.win_len = len(np.zeros(self.SR*10))
            self.EEG_5 = {k: np.ndarray(0) for k in self.mouselist}
            self.EMG_5 = {k: np.ndarray(0) for k in self.mouselist}

            self.spectro_buf1 = {k: np.zeros((240,501)) for k in self.mouselist}
            self.spectro_buf2 = {k: np.zeros((240,501)) for k in self.mouselist}

            self.thr_delta = {}
            self.thr_deltap = {}
            self.thr_th_delta1 = {}
            self.thr_th_delta1p = {}
            self.thr_th_delta2 = {}
            self.thr_th_delta2p = {}
            self.thr_mu = {}    
            self.thr_mup = {}       
            self.channels = {}
            self.bern_rem = {}
            self.bern_counter = {}

            for mouse in self.mouselist:
                params = sleepy.load_sleep_params(self.configpath, mouse + '_rem.txt')
                self.thr_delta[mouse] = params['THR_DELTA'][0]
                self.thr_deltap[mouse] = np.ones(240)*self.thr_delta[mouse]
                self.thr_th_delta1[mouse] = params['THR_TH_DELTA'][0]
                self.thr_th_delta1p[mouse] = np.ones(240)*self.thr_th_delta1[mouse]
                self.thr_th_delta2[mouse] = params['THR_TH_DELTA'][1]
                self.thr_th_delta2p[mouse] = np.ones(240)*self.thr_th_delta2[mouse]
                self.thr_mu[mouse] = params['THR_MU'][0]
                self.thr_mup[mouse] = np.ones(240)*self.thr_mu[mouse]
                self.channels[mouse] = params['ch_alloc'][0]
            

            counter = 0
            self.eeg_idx = {}
            self.emg_idx = {}
            for mouse in self.mouselist:
                self.eeg_idx[mouse] = self.channels[mouse].find('E') + counter
                self.emg_idx[mouse] = self.channels[mouse].find('M') + counter
                counter += 4


            self.delta_h = {k: list(np.zeros(240)) for k in self.mouselist}
            self.mu_h = {k: list(np.zeros(240)) for k in self.mouselist}
            self.theta_h = {k: list(np.zeros(240)) for k in self.mouselist}
            self.rem_h = {k: list(np.zeros(240)) for k in self.mouselist}
            self.th_delta_h = {k: list(np.zeros(240)) for k in self.mouselist}


            pos = np.array([0., 0.05, .2, .4, .6, .9])
            color = np.array([[0, 0, 0, 255], [0,0,255,255], [0,255,0,255], [255,255,0, 255], (255,165,0,255), (255,0,0, 255)], dtype=np.ubyte)
            cmap = pg.ColorMap(pos, color)
            self.lut = cmap.getLookupTable(0.0, 1.0, 256)

            self.image_fft1 = {k: pg.ImageItem() for k in self.mouselist}
            
            self.allplots = {}
            for i in range(len(self.mouselist)):
                self.allplots[self.mouselist[i]] = [self.ui.graphicsPlot.addPlot(title = 'Delta Threshold', row = 1, col = i+1)]
                self.allplots[self.mouselist[i]].append(self.ui.graphicsPlot.addPlot(title = 'Theta/Delta Threshold', row = 2, col = i+1))
                self.allplots[self.mouselist[i]].append(self.ui.graphicsPlot.addPlot(title = 'Mu Power Threshold', row = 3, col = i+1))
                self.allplots[self.mouselist[i]].append(self.ui.graphicsPlot.addPlot(title = 'EEG Spectrogram', row = 4, col = i+1))

                spec_plot = self.allplots[self.mouselist[i]][-1]
                spec_plot.clear()
                spec_plot.addItem(self.image_fft1[self.mouselist[i]])
                ax = spec_plot.getAxis(name='left')
                ax.setTicks([[(0, '0'), (10, '10'), (20, '20')]])

            for mouse in self.mouselist:
                for plot in self.allplots[mouse]:
                    plot.setMouseEnabled(x = False)
                try:
                    self.bern_rem[mouse] = np.random.binomial(1,params['Bern'][0],1000)
                    self.bern_counter[mouse] = 1
                except KeyError:
                    pass

            self.running = 1
            self.folderlist = glob.glob(self.load_dir + '/*')
            self.datdir = max(self.folderlist, key=os.path.getctime)
            self.updater_thread = QThread(self)
            self.data_getter = worker_getdata(self.datdir, self.eeg_idx, self.emg_idx, switch = 1)
            self.data_getter.moveToThread(self.updater_thread)
            self.data_getter.signal_new5sec.connect(self.update_master)
            self.updater_thread.started.connect(self.data_getter.run_it)
            self.updater_thread.start()

        except AttributeError:
            print('Choose a proper directory')
        except IOError:
            self.error_dialog.setText("rem txt file does not exist - continuing without spectrogram")
            self.error_dialog.show()


    @pyqtSlot(dict, dict)
    def update_master(self, new_deeg, new_demg):
        for mouse in self.mouselist:
            self.EEG_5[mouse] = np.append(self.EEG_5[mouse], new_deeg[mouse])
            self.EMG_5[mouse] = np.append(self.EMG_5[mouse], new_demg[mouse])
            try:
                self.update_spec(mouse, self.EEG_5[mouse], self.EMG_5[mouse])
            except TypeError:
                print('four inputs')

            if len(self.EEG_5[mouse]) >= 5000:
                self.EEG_5[mouse] = new_deeg[mouse]
                self.EMG_5[mouse] = new_demg[mouse]

    def update_spec(self, mousename, data_eeg, data_emg):

        if len(data_eeg) >= 5000:
            self.p_e, self.p_m, self.f_e, self.f_m = recursive_spectrogram(data_eeg, data_emg)

            for plots in self.allplots[mousename]:
                plots.clear()

            self.SE = self.alpha*self.p_e + (1-self.alpha) * self.spectro_buf1[mousename][-1,:]
            self.SM = self.alpha*self.p_m + (1-self.alpha) * self.spectro_buf2[mousename][-1,:]

            # power calculation

            i_delta = np.where((self.f_e >= self.r_delta[0]) & (self.f_e <= self.r_delta[1]))[0]
            i_theta = np.where((self.f_e >= self.r_theta[0]) & (self.f_e <= self.r_theta[1]))[0]
            i_mu = np.where((self.f_m >= self.r_mu[0]) & (self.f_m <= self.r_mu[1]))[0]
            
            pow_delta = np.sum(self.SE[i_delta])
            self.delta_h[mousename].append(pow_delta)
            self.delta_h[mousename] = self.delta_h[mousename][-240:]
            self.allplots[mousename][0].plot(self.delta_h[mousename])
            self.allplots[mousename][0].plot(self.thr_deltap[mousename], pen = pg.mkPen('r'))

            pow_theta = np.sum(self.SE[i_theta])
            self.theta_h[mousename].append(pow_theta)
            self.theta_h[mousename] = self.theta_h[mousename][-240:]

            th_delta = np.divide(pow_theta, pow_delta)
            self.th_delta_h[mousename].append(th_delta)
            self.th_delta_h[mousename] = self.th_delta_h[mousename][-240:]
            self.allplots[mousename][1].plot(self.th_delta_h[mousename])
            self.allplots[mousename][1].plot(self.thr_th_delta1p[mousename], pen = pg.mkPen('r'))
            self.allplots[mousename][1].plot(self.thr_th_delta2p[mousename], pen = pg.mkPen('b'))

            pow_mu = np.sum(self.SM[i_mu])
            self.pow_muh[mousename].append(pow_mu)
            self.mu_h[mousename].append(pow_mu)
            self.mu_h[mousename] = self.mu_h[mousename][-240:]
            self.allplots[mousename][2].plot(self.mu_h[mousename])
            self.allplots[mousename][2].plot(self.thr_mup[mousename], pen = pg.mkPen('r'))


            # determine rem state

            if (self.prem[mousename] == 0 and pow_delta < self.thr_delta[mousename] and pow_mu < self.thr_mu[mousename]):
            ### could be REM
            
                if (th_delta > self.thr_th_delta1[mousename]):
                ### we are potentially entering REM
                    if (self.past_len[mousename] < self.num_iter[mousename]):
                        past_len = self.past_len[mousename]
                    else:
                        past_len = self.num_iter[mousename]

                    # count the percentage of brainstate bins with elevated EMG power
                    if past_len != 0:
                        c_mu = np.sum(np.where(self.pow_muh[mousename][(past_len*-1):]>self.thr_mu[mousename])[0] ) / past_len

                        if c_mu < 0.2: # 0.2 = past_mu previously
                        ### we are in REM
                            self.prem[mousename] = 1  # turn laser on

            # We are currently in REM; do we stay there?
            if self.prem[mousename] == 1:
                ### REM continues, if theta/delta is larger than soft threshold and if there's
                ### no EMG activation
                if ((th_delta > self.thr_th_delta2[mousename]) and (pow_mu < self.thr_mu[mousename])):
                    self.prem[mousename] = 1
                else:
                    self.prem[mousename] = 0 #turn laser off
                    self.bern_counter[mousename] += 1


            try:
                if self.prem[mousename] == 1:
                    try:
                        if self.ui.cl_enable.checkState() == 2:
                            if self.bern_rem[mousename][self.bern_counter[mousename]] == 1:
                                self.s.send(mousename + '2')
                                self.prem[mousename] += 1 # bump up prem to 2 if actually on vs. 1 when it is randomly off
                            else:
                                self.s.send(mousename + '1')
                    except KeyError:
                        print('laser on at 100%' + ' rate')
                        if self.ui.cl_enable.checkState() == 2:
                            self.s.send(mousename + '2')
                else:
                    if self.ui.cl_enable.checkState() == 2:
                        self.s.send(mousename + '0')
            except socket.error:
                print('error')

            self.rem_hist[mousename].append(self.prem[mousename])
            self.rem_h[mousename].append(self.prem[mousename])
            self.rem_h[mousename] = self.rem_h[mousename][-240:]
            self.allplots[mousename][1].plot(self.rem_h[mousename], pen = pg.mkPen('g'))

            if self.prem[mousename] == 2:
                self.prem[mousename] = 1

            # Buffer add

            self.num_iter[mousename] += 1


            self.spectro_buf1[mousename] = np.vstack([self.spectro_buf1[mousename], [self.SE]])[1:,:]
            self.spectro_buf2[mousename] = np.vstack([self.spectro_buf2[mousename], [self.SM]])[1:,:]

            x = pg.makeARGB(self.spectro_buf1[mousename], levels = [0,45000], lut = self.lut)[0]
            y = pg.makeARGB(self.spectro_buf2[mousename], levels = [0,45000], lut = self.lut)[0]

            self.allplots[mousename][3].addItem(self.image_fft1[mousename])
            # self.P6.addItem(self.image_fft2)
            self.image_fft1[mousename].setImage(x)
            # self.image_fft2.setImage(y)





    @pyqtSlot()
    def start_clicked(self):
        if not self.start_on and not self.preview_on:
            if self.ard_on or self.rpi_on:
                self.mouselist, mouselist_empty = self.create_mouselist()
                if mouselist_empty:
                    self.error_dialog.setText("There are no mice selected")
                    self.error_dialog.show() 
                    return

                self.start_on = True
                self.disable_input(True)
                self.disable_comment(False)
                self.setupNotes()
                self.setupProtocol()
                self.start_videoRec()
                self.controls['startbutton'].setStyleSheet("background-color: red")

            else:
                self.error_dialog.setText("You must connect the controller first!")
                self.error_dialog.show()


    @pyqtSlot()
    def stop_clicked(self):

        if self.start_on and not self.preview_on:
            self.start_on = False
            self.graphicsView.clear()
            self.disable_input(False)
            self.disable_comment(True)
            self.endNotes()
            self.endProtocol()
            self.end_videoRec()
            self.commentItems['commentHist'].clear()
            self.commentItems['commentHist'].appendPlainText("Comment history:")
            self.ui.startbutton.setStyleSheet(self.ui.stopbutton.styleSheet())



    # @pyqtSlot()
    # def pulon_clicked(self):

    # @pyqtSlot()
    # def puloff_clicked(self):

    def disable_comment(self, disable):

        self.commentItems['entercomment'].setDisabled(disable)

    @pyqtSlot()
    def submit_comment(self):
        # Writing comment to the txt file
        self.txt = open(self.todate_full + "_notes.txt", 'a')
        self.txt.write("\r\n//")
        self.txt.write(time.asctime()[11:19] + " ")
        self.txt.write(self.commentItems['commentbox'].toPlainText())
        self.txt.close()

        # Writing comment to the history
        current_txt = self.commentItems['commentHist'].toPlainText()
        new_txt = self.commentItems['commentbox'].toPlainText()
        self.commentItems['commentHist'].appendPlainText("\r\n" + time.asctime()[11:19] + " " + new_txt)
        self.commentItems['commentbox'].clear()

    def closeEvent(self, *args, **kwargs):
        super(QtGui.QMainWindow, self).closeEvent(*args, **kwargs)
        self.stop_clicked()
        self.cams_disconnect()
        self.arduino_disconnect()
        



if __name__ == "__main__":

    # Create the QT application
    app = QtWidgets.QApplication(sys.argv)

    # Create the main window
    win = main()

    # Show window
    win.show()

    # Disable window resizing
    win.setFixedSize(win.size())

    # Start QT application
    sys.exit(app.exec_())


















