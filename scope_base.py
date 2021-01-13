# -*- coding: utf-8 -*-
"""
Created on Sat Jul  4 15:39:49 2020

@author: jakes
"""

import serial
import numpy as np
import numpy.core.defchararray as npchar
from scipy import fft
from scipy.signal import windows
import matplotlib.pyplot as plt
from matplotlib import ticker
import time
import operator

### Function definitions ###
def secformat(num, units=' ms', elementwise=True):
    """
    Function to convert a number, or array of numbers, into a string representation with units
    appended.

    Parameters
    ----------
    num : float, int or array of such.
        The object to be converted.
    units : string, optional
        One of ' ms', ' V', ' Hz', ' dB' (note the leading space). The default is ' ms'.
    elementwise : boolean, optional
        Whether the function applies its formatting rules to each element of the array
        individually, or based upon the largest element (which is useful for consistency in axis
        labels, for example). The default is True.

    Returns
    -------
    None.

    """
    ind = [' ms', ' V', ' Hz', ' dB'].index(units)
    milliunits = [chr(32)+chr(956)+'s', ' mV', ' mHz', ' mdB'][ind]
    kilounits = [' s', ' kV', ' kHz', 'kdB'][ind]
    num = np.array(num)
    absnum = np.abs(num)
    biggest = absnum.max()
    output = np.empty_like(num, dtype=str)
    
    if elementwise:
        over1 = (absnum//1).astype(bool)
        floatover1 = ((absnum//1)*(absnum%1)).astype(bool)
        over1000 = (absnum//1000).astype(bool)
        floatover1000 = ((absnum//1000)*(absnum%1000)).astype(bool)
    else:
        over1 = np.resize((biggest//1).astype(bool), num.size)
        floatover1 = np.resize(((biggest//1)*(num%1)).astype(bool), num.size)
        over1000 = np.resize((biggest//1000).astype(bool), num.size)
        floatover1000 = np.resize(((biggest//1000)*(num%1000)).astype(bool), num.size)
    equalszero = num == 0
        
    output = npchar.add(((num*1000).astype(int)).astype(str), milliunits)
    output[over1] = npchar.add((num[over1].astype(int)).astype(str), units)
    output[floatover1] = npchar.add(np.round(num[floatover1], 2).astype(str), units)
    output[over1000] = npchar.add(((num[over1000]/1000).astype(int)).astype(str), kilounits)
    output[floatover1000] = npchar.add(np.round(num[floatover1000]/1000, 2).astype(str), kilounits)
    output[equalszero] = '0' + units
    if output.size == 1: return output.item()
    else: return output

def drawplot(timebase, size=(12, 6)):
    """
    Minimal function which draws the figure and axes of the plot window. Also calls timeaxis() to
    add labels appropriate for the time domain.

    Parameters
    ----------
    timebase : float
        The timebase used to create the plot x tick labels (which are equally spaced 'timebase'
        apart).
    size : tuple, optional
        Tuple (width, height) of the figure window. The default is (12, 6).

    Returns
    -------
    figure : matplotlib figure object
        The figure.
    axis : matplotlib axes object
        The axis.

    """
    plt.close('all')
    plt.ioff()
    plt.style.use('dark_background')
    figure = plt.figure(figsize = size)
    axis =  timeaxis(figure.add_subplot(111), timebase)
    axis.set_position([0.05, 0.1, 0.9, 0.8])
    return (figure, axis)

def timeaxis(axis, timebase):
    """
    Modify the submitted 'axis' to create a time-domain plot according to 'timebase'. Returns the
    modified axis.
    """
    axis.set_xlim(0, 10*timebase)
    axis.set_xticks(timebase*np.arange(11))
    axis.set_xticklabels(secformat(timebase*np.arange(11), units=' ms', elementwise=False))
    axis.set_ylim(0, 4)
    axis.set_yticks(np.arange(5))
    axis.set_yticklabels(secformat(np.arange(5), units=' V'))
    axis.grid(True, lw=0.5, ls='--')
    axis.minorticks_on()
    axis.xaxis.set_minor_locator(ticker.LinearLocator(numticks=51))
    axis.yaxis.set_minor_locator(ticker.LinearLocator(numticks=21))
    return axis

def freqaxis(axis, interval):
    """
    Modify the submitted 'axis' to create a frequency-domain plot according to 'interval' - the
    sample period. Returns the modified axis.
    """
    xmax = (10**6)/(2*interval)
    yticks = [-100, -75, -50, -25, 0]
    axis.set_xlim(0, xmax)
    axis.set_xticks((xmax/10)*np.arange(11))
    axis.set_xticklabels(secformat((xmax/10)*np.arange(11), units=' Hz', elementwise=False))
    axis.set_ylim(yticks[0], yticks[-1])
    axis.set_yticks(yticks)
    axis.set_yticklabels(secformat(yticks, units=' dB', elementwise=False))
    axis.grid(True, lw=0.5, ls='--')
    axis.minorticks_on()
    axis.xaxis.set_minor_locator(ticker.LinearLocator(numticks=51))
    axis.yaxis.set_minor_locator(ticker.LinearLocator(numticks=21))
    return axis

def chooserate(tbase, resolution='normal'):
    """
    Pick the sample interval appropriate to the timebase and desired resolution. Standard timebases
    are (in ms) {0.04, 0.1, 0.5, 1, 5, 10, 50, 100, 500}.

    Parameters
    ----------
    tbase : float
        The timebase, in milliseconds.
    resolution : string, optional
        Code to chose how many points are plotted each frame. 'normal' aims for 500, 'higher' for
        1000, 'highest' for 2000. The default is 'normal'.

    Returns
    -------
    int
        Returns the sample interval in microseconds, an integer.

    """
    maxpoints = {'normal':500, 'higher':1000, 'highest':2000}
    
    if 10000*tbase//maxpoints[resolution] > 8:
        return int(10000*tbase//maxpoints[resolution])
    else:
        return 8


def startcomms(COMaddress, interval):
    """
    Open serial communinications with the Teensy 4, via the COM port at the string 'COMaddress'.

    Parameters
    ----------
    COMaddress : str.
        The serial port address of the microcontroller front-end (e.g. "COM7"). If a nonetype is 
        passed, then startcomms treats the global variable 'port' as the address of an already-open
        serial port (this functionality is used to update timebase etc.).
    interval : int.
        The time (in us) between ADC samples. The front-end automatically adjusts the amount of
        averaging that is appropriate.

    Returns
    -------
    port : pyserial port object.
        The serial port over which data and halt messages can now be exchanged.

    """
    global port
    def sendtomicro(stringlist):
        for o in stringlist:
            port.write(bytes(o, 'utf-8'))
            print("Pyserial to Teensy: " + o)
            fileprint("Pyserial to Teensy: " + o + "\n")                
            time.sleep(2)
    try:
        fileprint("Startcomms called\n")
        outstrings = ['-6', format(interval, '03d')]
        if COMaddress:
            port = serial.Serial(COMaddress, 9600, timeout=1)
            time.sleep(0.5)
        port.flushInput()
        sendtomicro(outstrings)
             
        responded = False
        T = time.time()
        while not responded:
            buff = readandprint(port)   
            if buff.decode('utf-8') == "Message received: " + outstrings[1] + "\n":
                responded = True    
            elif time.time() - T > 5:
                sendtomicro(outstrings)
        return port
    except Exception as err:
        fileprint("Startcomms encountered {}: {}\n".format(type(err), err))     
        raise(err)

def readandprint(serialport):
    """
    A simple function that reads one line of the serial port buffer, then prints the result both
    to the terminal, and to the default log file according to fileprint.
    """
    buff = serialport.readline()
    print("Teensy (%03d): " %(serialport.in_waiting), buff)
    fileprint("Teensy ({}): {}\n".format(serialport.in_waiting, buff))
    return buff

def fileprint(message, file='debug.txt', mode='a'):
    """
    Print 'message' to the log file 'file'. Mode is one of the standard io codes ('w' for write,
    'a' for append, etc.)
    """
    with open(file, mode) as outfile:
        outfile.write(message)

def trigfunc(trigger):
    """ Generate the comparison function appropriate to the trigger mode (rising edge, falling edge
        or off: represented by '+', '-', '', respectively).
    """
    if trigger == '+':
        compare = operator.gt
    elif trigger == '-':
        compare = operator.lt
    else:
        compare = lambda x, y: 0
    return compare

def mainloop(port, triglevel, compare, data, target):
    """
    The main data collection and plotting loop of the program.

    Parameters
    ----------
    port : The serial port to write to.
    triglevel : int.
        The trigger threshold which must be exceeded (subceeded) if the trigger is on the rising
        (falling) edge. The value is inconsequential for no trigger. Should be input as an integer
        in the range [0, 1024].
    compare : 2-argument function. 
        Operator.gt if trigger on rising edge, operator.lt if on a falling edge, and any function
        that maps all inputs to zero if no trigger.
    data : Numpy array.
        2 x N array in which to store data. The size of the array controls how many data are
        collected by mainloop.
    target : int.
        The ideal sample interval, as returned by chooserate. This is used to check for corrupted
        data.
    
    Returns
    -------
    data : Numpy array.
        A filled version of the input 'data'.
    """
    triggered = False if compare(1, 0) or compare(0, 1) else True
    last = np.array([0, 1023]) if compare(1, 0) else np.array([0, -1])
    data[1,-1] = 1025
    offenders = []
    i = 0
    while data[1,-1] == 1025:
        """Loop over one data packet (one display's worth of data)."""
        bout = port.readline()
        try:
            datum, delt = int(bout[:4]), int(bout[5:-1])
            if not 0.8*target < int(delt) < 1.2*target:
                """Check for corrupted data."""
                raise ValueError("delt larger than expected")
        except ValueError:
            if i == 0:
    #            print("bad data at first read: {}".format(bout))
                continue
            else:
                print("bad data at point {}: {} w\ buffer length {}".format(i, bout, port.in_waiting))
                offenders.append(i)
                delt, datum = target, 0
        if not triggered:
            if compare(triglevel, last[1]) and compare(datum, triglevel):
                triggered = True
                last[0] = 0
                data[:, 0] = last
                i = 1
        else:
            i += 1   
        last = [last[0] + delt, datum]
        data[:,i] = last
    
    for ind in offenders:
        data[1,ind] = (data[1,ind-1] + data[1,ind+1])/2
    port.flushInput()       #Helps keep on top of serial buffer and respond to changes.
    port.readline()
    port.readline()
    return data
            
def spectrum(data):
    """
    A simple function that performs an FFT on 'data' using a Hann window function. data[0,:] should
    be times in microseconds; data[1,:] should be voltages in volts. Returns an array with
    identical dimensions to data, but now with [0,:] elements frequencies, and [1,:] elements
    the corresponding amplitudes (in dB). Employs a 5 point moving average for large datasets.
    """
    x, y = data[0,:], data[1,:]
    N = y.size
    win = windows.hann(N)
    S1 = np.sum(win)
    f = np.abs(fft.fftfreq(N, x[1]/1000000)[:N//2 + 1])
    psd = (2*np.abs(fft.rfft(y*win, axis=0))**2)/(S1**2)
    if psd.size > 300:
        #psd[2:-2] = (psd[:-4] + psd[1:-3] + psd[2:-2] + psd[3:-1] + psd[4:])/5
        psd[1:-1] = (psd[:-2] + psd[1:-1] + psd[2:])/3
    db = 10*np.log10(psd)
    return np.vstack((f, db))
            

def updategraph(figure, line, data):      
        """Output to graph."""
        line.set_data(data[0,:]*10**(-3), 3.3*data[1,:]/1024)
        plt.pause(0.0001)
        
def finishup(port):
    """ Closes the serial port in the event of a keyboard interrupt or other unforeseen error.
    """
    if port.is_open:
        port.write(b'-6')
        time.sleep(0.2)
        port.write(b'-6')
        print("Pyserial to Teensy: -6")
        port.close()
        print("Port closed")

### Initialisation code ###
baselist = np.array([500, 100, 50, 10, 5, 1, 0.5, 0.1, 0.04])
timebase = 5       #Timebase in milliseconds
trigger = ''      #Trigger=True => trigger on + edge. Trigger=False => trigger on - edge.  

### Main Instructions ###
if __name__ == '__main__':
    try:
        delt = chooserate(timebase, resolution='highest')
        port = None
        port = startcomms("COM7", delt)
        print("Measurement interval =", delt, "us")
        N = int(10**4*timebase//delt)
        print("N =", N)
        fig, ax = drawplot(timebase)       
        line, = ax.plot(np.arange(N), np.zeros(N, dtype=int), color='#983a9e', lw=1)
        data = np.vstack((np.arange(N)*delt, np.zeros(N, dtype=int)))
#        ax = freqaxis(ax, delt)        
        fig.show()
        compare = trigfunc(trigger)
        port.flushInput()
        while True:
            start = time.time()
            port.flushInput()
            data = mainloop(port, 256, compare, data, delt)
            updategraph(fig, line, data)

#            line.set_data(spectrum(data)[0,:], spectrum(3.3*data/1024)[1,:])
#            plt.pause(0.001)

#            print("Exec time = %06.4f  Buffer size = %04d"
#                  %(time.time() - start, port.in_waiting))
    except KeyboardInterrupt:
        finishup(port)
    except Exception as e:
        finishup(port)
        raise e

    