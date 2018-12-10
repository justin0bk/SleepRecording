import sys
import socket
import RPi.GPIO as GPIO
from threading import Thread
import time

def get_ip_address():
    ip_address = '';
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.connect(("8.8.8.8",80))
    ip_address = s.getsockname()[0]
    s.close()
    return ip_address

GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)

pin_onoff = 36
pin_ol = 37
pin_cl1 = 15
fpin_cl1 = 16
pin_cl2 = 18
fpin_cl2 = 22
pin_cl3 = 29
fpin_cl3 = 31
pin_cl4 =32
fpin_cl4 = 33


GPIO.setup(pin_onoff, GPIO.OUT, initial = 0)
GPIO.setup(pin_ol, GPIO.OUT, initial = 0)
GPIO.setup(pin_cl1, GPIO.OUT, initial = 0)
GPIO.setup(fpin_cl1, GPIO.OUT, initial = 0)
GPIO.setup(pin_cl2, GPIO.OUT, initial = 0)
GPIO.setup(fpin_cl2, GPIO.OUT, initial = 0)
GPIO.setup(pin_cl3, GPIO.OUT, initial = 0)
GPIO.setup(fpin_cl3, GPIO.OUT, initial = 0)
GPIO.setup(pin_cl4, GPIO.OUT, initial = 0)
GPIO.setup(fpin_cl4, GPIO.OUT, initial = 0)

def set_hi_lo(hi, lo):
    hi = float(hi)
    lo = float(lo)
    freq = 1000/(hi+lo)
    dutcy = (hi)/(hi+lo)*100
    return freq, dutcy

def start_cycle():
    
    global server
    global client
    hi = 5 #default hi
    lo = 95 #default lo
    freq, dutcy = set_hi_lo(hi, lo)
    puldur = 120
    print("by default, hi=5, lo=95, freq=10, dutcy = 5%, puldur = 120")

    p_cl1 = GPIO.PWM(pin_cl1, freq)
    p_cl2 = GPIO.PWM(pin_cl2, freq)
    p_cl3 = GPIO.PWM(pin_cl3, freq)
    p_cl4 = GPIO.PWM(pin_cl4, freq)
    fp_cl1 = GPIO.PWM(fpin_cl1, freq)
    fp_cl2 = GPIO.PWM(fpin_cl2, freq)
    fp_cl3 = GPIO.PWM(fpin_cl3, freq)
    fp_cl4 = GPIO.PWM(fpin_cl4, freq)
    
    p_cl1 = GPIO.PWM(pin_cl1, freq)
    fp_cl1 = GPIO.PWM(fpin_cl1, freq)
    p_cl1.start(0)
    fp_cl1.start(0)

    p_cl2 = GPIO.PWM(pin_cl2, freq)
    fp_cl2 = GPIO.PWM(fpin_cl2, freq)
    p_cl2.start(0)
    fp_cl2.start(0)

    p_cl3 = GPIO.PWM(pin_cl3, freq)
    fp_cl3 = GPIO.PWM(fpin_cl3, freq)
    p_cl3.start(0)
    fp_cl3.start(0)

    p_cl4 = GPIO.PWM(pin_cl4, freq)
    fp_cl4 = GPIO.PWM(fpin_cl4, freq)
    p_cl4.start(0)
    fp_cl4.start(0)

    while True:
        msg = client.recv(8)
        print(msg)
        if msg == 'STAT0000':
            GPIO.output(pin_onoff, 0)
            GPIO.output(pin_ol, 0)
            p_cl1.ChangeDutyCycle(0)
            fp_cl1.ChangeDutyCycle(0)
            p_cl2.ChangeDutyCycle(0)
            fp_cl2.ChangeDutyCycle(0)
            p_cl3.ChangeDutyCycle(0)
            fp_cl3.ChangeDutyCycle(0)
            p_cl4.ChangeDutyCycle(0)
            fp_cl4.ChangeDutyCycle(0)
            
        elif msg == 'STAT0001':
            GPIO.output(pin_onoff, 1)
        elif msg == 'STAT0101':
            p = GPIO.PWM(pin_ol, freq)
            p.start(dutcy)
            time.sleep(puldur-1)
            p.stop()
        elif msg == 'STAT0201':
            fp_cl1.ChangeFrequency(freq)
            fp_cl1.ChangeDutyCycle(dutcy)
        elif msg == 'STAT0211':
            p_cl1.ChangeFrequency(freq)
            fp_cl1.ChangeFrequency(freq)
            p_cl1.ChangeDutyCycle(dutcy)
            fp_cl1.ChangeDutyCycle(dutcy)
        elif msg == 'STAT0200':
            p_cl1.ChangeDutyCycle(0)
            fp_cl1.ChangeDutyCycle(0)
        elif msg == 'STAT0301':
            fp_cl2.ChangeFrequency(freq)
            fp_cl2.ChangeDutyCycle(dutcy)
        elif msg == 'STAT0311':
            p_cl2.ChangeFrequency(freq)
            fp_cl2.ChangeFrequency(freq)
            p_cl2.ChangeDutyCycle(dutcy)
            fp_cl2.ChangeDutyCycle(dutcy)
        elif msg == 'STAT0300':
            p_cl2.ChangeDutyCycle(0)
            fp_cl2.ChangeDutyCycle(0)
        elif msg == 'STAT0401':
            fp_cl3.ChangeFrequency(freq)
            fp_cl3.ChangeDutyCycle(dutcy)
        elif msg == 'STAT0411':
            p_cl3.ChangeFrequency(freq)
            fp_cl3.ChangeFrequency(freq)
            p_cl3.ChangeDutyCycle(dutcy)
            fp_cl3.ChangeDutyCycle(dutcy)
        elif msg == 'STAT0400':
            p_cl3.ChangeDutyCycle(0)
            fp_cl3.ChangeDutyCycle(0)
        elif msg == 'STAT0501':
            fp_cl4.ChangeFrequency(freq)
            fp_cl4.ChangeDutyCycle(dutcy)
        elif msg == 'STAT0511':
            p_cl4.ChangeFrequency(freq)
            fp_cl4.ChangeFrequency(freq)
            p_cl4.ChangeDutyCycle(dutcy)
            fp_cl4.ChangeDutyCycle(dutcy)
        elif msg == 'STAT0500':
            p_cl4.ChangeDutyCycle(0)
            fp_cl4.ChangeDutyCycle(0)
        elif 'HI' in msg:
            hi = int(msg[2:])
            freq, dutcy = set_hi_lo(hi,lo)
        elif 'LO' in msg:
            lo = int(msg[2:])
            freq, dutcy = set_hi_lo(hi,lo)
        elif 'PD' in msg:
            puldur = int(msg[2:])
        elif msg == 'END_PROG':
            server.close()
            return
    

def setup_network():
    
    server  = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    host = get_ip_address()
    print('host ip: ' + host)
    port = 5010
    
    time.sleep(1)
    try:
        server.bind((host,port))
    except socket.error:
        try:
            server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            server.bind((host, port))
        except:
            sys.exit('Port' + str(port) + ' is occupied')

    server.listen(5)
    client, address = server.accept()
    print('Client found:', address)
    
    return server, client
    
    
recv_thread = Thread(target=start_cycle)    
while True:    
    if not recv_thread.isAlive():
        server, client = setup_network()
        recv_thread = Thread(target=start_cycle)
        recv_thread.start()
        recv_thread.join()
        server.close()
        server = None
        client = None
