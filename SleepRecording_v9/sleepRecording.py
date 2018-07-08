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


        #### Connecting Signals and Slots ####
        for protocol in self.protocols:
            self.protocols[protocol].clicked.connect(self.protocol_setup)
        self.devices['connect_controller'].clicked.connect(self.comm_connect)
        
        self.devices['connect_c1'].clicked.connect(self.cam1_setup)
        self.devices['connect_c2'].clicked.connect(self.cam2_setup)
        self.controls['preview'].clicked.connect(self.preview_clicked)
        # self.controls['startbutton'].clicked.connect(self.start_clicked)
        # self.controls['stopbutton'].clicked.connect(self.stop_clicked)

        # self.pulseControls['c_on'].clicked.connect(self.pulon_clicked)
        # self.pulseControls['c_off'].clicked.connect(self.puloff_clicked)

        # self.commentItems['entercomment'].clicked.connect(self.submit_comment)

        #### Start from default setup ####
        self.disable_comment()
        self.default_setup()

    @pyqtSlot()
    def protocol_setup(self):
        if self.protocols['option_default'].isChecked():
            # Disable settings used for other protocols
            self.parameters['maxPG'].setDisabled(True)
            self.parameters['minPG'].setDisabled(True)
            for item in self.thresholds:
                self.thresholds[item].setDisabled(True)

        elif self.protocols['option_ol'].isChecked():
            self.parameters['maxPG'].setDisabled(False)
            self.parameters['minPG'].setDisabled(False)
            for item in self.thresholds:
                self.thresholds[item].setDisabled(True)

        else:
            self.parameters['maxPG'].setDisabled(True)
            self.parameters['minPG'].setDisabled(True)
            for item in self.thresholds:
                self.thresholds[item].setDisabled(False)



    @pyqtSlot()
    def run_default(self):
        
    @pyqtSlot()
    def run_ol(self):
        
    @pyqtSlot()
    def run_cl(self):
        


    # def arduino_mode(self):

    # def arduino_disconnect(self):

    # def raspi_mode(self, ip):

    # def raspi_disconnect(self):

    @pyqtSlot()
    def comm_connect(self):
        if not self.ard_on and not self.rpi_on:
            ip_address, okPressed = QInputDialog.getText(self, "Connecting to RPi","Type the IP address of your RPi (ie. 10.103.215.103) or skip to disregard", QLineEdit.Normal, "")
            if okPressed:
                if ip_address != '':
                    self.raspi_mode(ip_address)
                else:
                    self.arduino_mode()
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

    
    def cams_disconnect(self):
        try:
            self.cam1.disconnect()
            self.cam2.disconnect()
            self.cam1_connected = False
            self.cam2_connected = False
        except AttributeError:
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



    @pyqtSlot()
    def preview_clicked(self):
        if not self.cam1_connected and not self.cam2_connected:
            self.error_dialog.setText("You must connect the camera first!")
            self.error_dialog.show()

        elif not self.start_on and not self.preview_on:
            self.controls['preview'].setStyleSheet("background-color: green")
            self.preview_on = True

            for buttons in self.devices:
                self.devices[buttons].setDisabled(True)
            for buttons in self.controls:
                if 'preview' not in buttons:
                    self.controls[buttons].setDisabled(True)

            # if self.cam2on != 1:
            #     self.camera_thread = QThread(self)
            #     self.camera_obj = worker_camera('P', self.c)
            #     self.camera_obj.moveToThread(self.camera_thread)
            #     self.camera_obj.camera_on = 1
            #     self.camera_obj.signal_camFrame.connect(self.process_vid)
            #     self.camera_thread.started.connect(self.camera_obj.run_camera)
            #     self.c.startCapture()
            #     self.camera_thread.start()

            # else:
            #     self.camera_thread = QThread(self)
            #     self.camera_obj = worker_camera('P', self.c, self.c2)
            #     self.camera_obj.moveToThread(self.camera_thread)
            #     self.camera_obj.camera_on = 1
            #     self.camera_obj.signal_camFrame.connect(self.process_vid)
            #     self.camera_thread.started.connect(self.camera_obj.run_camera)
            #     self.c.startCapture()
            #     self.c2.startCapture()
            #     self.camera_thread.start()


        elif not self.start_on and self.preview_on:
            self.controls['preview'].setStyleSheet(self.ui.stopbutton.styleSheet())
            self.preview_on = False

            for buttons in self.devices:
                self.devices[buttons].setDisabled(False)
            for buttons in self.controls:
                self.controls[buttons].setDisabled(False)

            # if self.cam2on != 1:
            #     self.camera_obj.camera_on = 0
            #     self.c.stopCapture()
            #     self.c.writeRegister(self.pin2_strobecnt, self.StrobeOff)
            #     self.camera_thread.quit()
            #     self.camera_thread.wait()

            # else:
            #     self.camera_obj.camera_on = 0
            #     self.c.stopCapture()
            #     self.c2.stopCapture()
            #     self.c.writeRegister(self.pin2_strobecnt, self.StrobeOff)
            #     self.c2.writeRegister(self.pin2_strobecnt, self.StrobeOff)
            #     self.camera_thread.quit()
            #     self.camera_thread.wait()

    # @pyqtSlot()
    # def start_clicked(self):

    def enable_comment(self):
        self.commentItems['entercomment'].setDisabled(False)

    # @pyqtSlot()
    # def stop_clicked(self):

    def disable_comment(self):
        self.commentItems['entercomment'].setDisabled(True)

    # @pyqtSlot()
    # def pulon_clicked(self):

    # @pyqtSlot()
    # def puloff_clicked(self):

    @pyqtSlot()
    def submit_comment(self):
        self.f = open(self.todayis + "_notes.txt", 'a')
        self.f.write("\r\n//")
        self.f.write(time.asctime()[11:19] + " ")
        self.f.write(self.commentItems['commentbox'].toPlainText())
        self.commentItems['commentbox'].clear()
        self.f.close()

    @pyqtSlot()
    def submit_comment(self):
        # Writing comment to the txt file
        self.f = open(self.todayis + "_notes.txt", 'a')
        self.f.write("\r\n//")
        self.f.write(time.asctime()[11:19] + " ")
        self.f.write(self.commentItems['commentbox'].toPlainText())
        self.f.close()

        # Writing comment to the history
        current_txt = self.commentItems['commentHist'].toPlainText()
        new_txt = self.commentItems['commentbox'].toPlainText()
        self.commentItems['commentHist'].appendPlainText("\r\n" + time.asctime()[11:19] + " " + new_txt)
        self.commentItems['commentbox'].clear()


    def closeEvent(self, *args, **kwargs):
        super(QtGui.QMainWindow, self).closeEvent(*args, **kwargs)

        self.cams_disconnect()



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


















