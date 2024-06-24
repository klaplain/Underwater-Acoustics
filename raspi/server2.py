from flask import Flask, redirect, url_for, render_template, request, send_file
import time, datetime,json
import spidev
import RPi.GPIO as GPIO
import os

# for data transformation
import numpy as np
# for visualizing the data
import matplotlib.pyplot as plt
# for opening the media file
import scipy.io.wavfile as wavfile
# Importing libraries using import keyword.
import math

# SPI Command IDs for Acquisition Subsystem
DIRECTORY = 2
DATETIME = 3
LOCATION =4
TRANSFER = 5
FORMAT = 6
DELETE = 7
RECORD = 9

# Set defaults for global variables
fourbytestoread=[0,0,0,0]
twobytestoread=[0,0]
onebytetoread=[0]
AcqSubSystemReady = "Busy"
directory_listing =""
samplingFreq_default="200"
gain_default ="2"
duration_default ="2"
filePrefix_default = "SS"

# Initialize SPI interface
bus = 0  # We only have SPI bus 0 available to us on the Pi
device = 0 #Device is the chip select pin. Set to 0 or 1, depending on the connections
spi = spidev.SpiDev() # Enable SPI
spi.open(bus, device) # Open a connection to a specific bus and device (chip select pin)
spi.max_speed_hz = 1000000  # Set SPI speed and mode
spi.mode = 0

def initialize():
    print("Initializing")
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    GPIO.setup(22, GPIO.OUT) # Reset STM32
    GPIO.output(22, GPIO.LOW)
    time.sleep(0.5)
    GPIO.output(22, GPIO.HIGH) #now release RESET pin

    GPIO.setup(27, GPIO.IN, pull_up_down=GPIO.PUD_UP) # STM32 SPI Busy
    set_datetime()
    time.sleep(0.5)  # need to give the STM32 a break

def directory():  # Code 0
    print("SD Directory")
    msg= [DIRECTORY,0,DIRECTORY,0,DIRECTORY,0,DIRECTORY,0]
    directory_listing=""
    while GPIO.input(27) == GPIO.HIGH  :  pass # Wait until STM ACQ is Ready
    spi.writebytes(msg)
    while True:
        while GPIO.input(27) == GPIO.HIGH  :  pass # Wait until STM ACQ is Ready
        spi.xfer(onebytetoread)
        directory_listing=directory_listing+chr(onebytetoread[0])
        if onebytetoread[0]==12:   #12 signifies the end of the directory listing
            break
    directory_list = directory_listing.split(chr(10))
    #print(directory_listing)

    while GPIO.input(27) == GPIO.HIGH  :  pass # Wait until STM ACQ is Ready before returning
    print("End of Directory")
    return directory_list

def set_datetime(): # Function to set date and time on Acquisition Subsystem Code 3
    print("Set datetime")
    now=datetime.datetime.now()
    msg= [DATETIME,now.hour,now.minute,now.second,4,now.month,now.day,now.year-2000]
    while GPIO.input(27) == GPIO.HIGH  :  pass # Wait until STM ACQ is Ready before sending command
    spi.writebytes(msg)
    time.sleep(0.1) #Give STM ACQ time to set Busy
    while GPIO.input(27) == GPIO.HIGH  :  pass # Wait until STM ACQ is Ready before returning
    return

def transfer(file,hydrophoneArrayName,projectName,lat,long,gain):   # Code 5 - TRANSFER
    this_filename =file.split(" ")[0][2:]
    filename_bytes=bytes(this_filename, "utf8")
    print("Transfer to Pi",this_filename)
    msg= [TRANSFER,0,0,0,0,0,0,0]

    while GPIO.input(27) == GPIO.HIGH  :  pass # Wait until STM ACQ is Ready before sending command
    spi.writebytes(msg)
    time.sleep(0.1) #Give STM ACQ time to set Busy

    # we have just sent the 8 byte packet.
    # Now we need to send length of filename as a 16 bit int

    msg=[len(this_filename) >>8, len(this_filename)]
    while GPIO.input(27) == GPIO.HIGH  :  pass # Wait until STM ACQ is Ready before sending command
    spi.writebytes(msg) # tell the STM ACQ how many bytes to expect
    time.sleep(0.1) #Give STM ACQ time to set Busy

    # now we send the filename
    while GPIO.input(27) == GPIO.HIGH  :  pass # Wait until STM ACQ is Ready before reading data
    spi.writebytes(filename_bytes)
    time.sleep(0.1) #Give STM ACQ time to set Busy
    while GPIO.input(27) == GPIO.HIGH  :  pass # Wait until STM ACQ is Ready before returning

    #open file for writing on RASPI.  We can do this to kill time before the AcqSystem is ready
    f=open("./download/"+this_filename[:-3]+"WAV","wb")
    print("File opened", this_filename[:-3]+"WAV")

    #Get bytecount
    time.sleep(0.1) # give STM time to set the busy flag
    while GPIO.input(27) == GPIO.HIGH  :  pass # Wait until STM ACQ is Ready before requesting/reading bytecount
    print("reading bytecount")
    fourbytestoread=[0,0,0,0]
    spi.xfer(fourbytestoread)
    TotalBytes= (fourbytestoread[0])+(fourbytestoread[1]<<8)+(fourbytestoread[2]<<16)+(fourbytestoread[3]<<24)
    TotalBytes = 1572908
    TotalInts=TotalBytes>>1
    print("Bytes =", TotalBytes,"Ints =",TotalInts)
    tic = time.perf_counter()

    #Get file data
    for this_int in range(TotalInts):
        while GPIO.input(27) == GPIO.HIGH  :  pass # Wait until STM ACQ is Ready before reading next word
        spi.xfer(twobytestoread)
        f.write(twobytestoread[0].to_bytes(1,'big'))
        f.write(twobytestoread[1].to_bytes(1,'little'))
    toc = time.perf_counter()
    print(f"Time to save {toc - tic:0.4f} seconds")
    f.write(createwavmetadata(hydrophoneArrayName,projectName,lat,long,gain))
    f.close()
    print("save done")
    time.sleep(0.1) #Give STM ACQ time to set Busy
    while GPIO.input(27) == GPIO.HIGH  :  pass # Wait until STM ACQ is Ready before returning
    #while False : pass
    return

def format():   # Code 6
    print("Format SD")
    msg= [FORMAT,0,FORMAT,0,FORMAT,0,FORMAT,0]
    while GPIO.input(27) == GPIO.HIGH  :  pass # Wait until STM ACQ is Ready before sending command
    spi.writebytes(msg)
    time.sleep(0.1) #Give STM ACQ time to set Busy
    while GPIO.input(27) == GPIO.HIGH  :  pass # Wait until STM ACQ is Ready before returning
    print("Format Complete")
    return

def delete(file):   # Code 7
    this_filename =file.split(" ")[0][2:]
    filename_bytes=bytes(this_filename, "utf8")
    print("Delete",this_filename)
    msg= [DELETE,0,0,0,0,0,0,0]
    while GPIO.input(27) == GPIO.HIGH  :  pass # Wait until STM ACQ is Ready before sending command
    spi.writebytes(msg)
    time.sleep(0.1) #Give STM ACQ time to set Busy

    # we have just sent the 8 byte packet.  Now we need to send the filename
    # first we need to send length of filename as a 16 bit int

    msg=[len(this_filename) >>8, len(this_filename)]
    while GPIO.input(27) == GPIO.HIGH  :  pass # Wait until STM ACQ is Ready before sending command
    spi.writebytes(msg) # tell the STM ACQ how many bytes to expect
    time.sleep(0.1) #Give STM ACQ time to set Busy

    # now we send the filename
    while GPIO.input(27) == GPIO.HIGH  :  pass # Wait until STM ACQ is Ready before reading data
    spi.writebytes(filename_bytes)
    time.sleep(0.1) #Give STM ACQ time to set Busy
    while GPIO.input(27) == GPIO.HIGH  :  pass # Wait until STM ACQ is Ready before returning

    print("Deletion Complete")
    return

def record(samplingFreq, gain, duration,filePrefix):   # Code 9
    samplingFreq_default = samplingFreq
    gain_default =  gain
    duration_default = duration
    filePrefix_default = filePrefix
    print("Recording",samplingFreq, gain, duration,filePrefix)
    #set_datetime()
    this_filename = filePrefix+datetime.datetime.now().strftime("%Y%m%d%H%M%S")+".DAT"
    filename_bytes=bytes(this_filename, "utf8")
    print("Recording Start", this_filename)
    duration = int(duration)*1000
    msg= [RECORD,((int(samplingFreq) >>8) & 0xff),(int(samplingFreq) & 0xff),int(gain),((int(duration) >>8) & 0xff),(int(duration) & 0xff),((int(0) >>8) & 0xff),(int(0) & 0xff)]
    while GPIO.input(27) == GPIO.HIGH  :  pass # Wait until STM ACQ is Ready before sending command
    spi.writebytes(msg)
    # we have just sent the 8 byte packet.  Now we need to send the filename

    # first we need to send length of filename as a 16 bit int
    msg=[len(this_filename) >>8, len(this_filename)]
    while GPIO.input(27) == GPIO.HIGH  :  pass # Wait until STM ACQ is Ready before sending command
    spi.writebytes(msg) # tell the STM ACQ how many bytes to expect

    # now we send the filename
    time.sleep(0.1) #Give STM ACQ time to set Busy
    while GPIO.input(27) == GPIO.HIGH  :  pass # Wait until STM ACQ is Ready before reading data (what on earth are we reading?
    spi.writebytes(filename_bytes)
    time.sleep(0.1) #Give STM ACQ time to set Busy
    while GPIO.input(27) == GPIO.HIGH  :  pass # Wait until STM ACQ is Ready before returning
    print("Recording Complete")
    return

def create_histogram(wavfilename):
    Fs, aud = wavfile.read(wavfilename)
    print ("Creating Histogram ",wavfilename, aud.shape)
    # select left channel only
    # aud = aud[:,0]
    # trim the first 125 seconds
    first = aud[:int(Fs*200)]
    # powerSpectrum, frequenciesFound, time, imageAxis = plt.specgram(first, Fs=Fs)
    plt.specgram(first, Fs=Fs, cmap="rainbow")
    # Set the title of the plot, xlabel and ylabel
    # and display using show() function
    plt.title("Spectrogram"+wavfilename)
    plt.ylabel("Frequency (Hz)")
    plt.xlabel("Time (secs)")
    # plt.show()
    print("H4")
    plt.savefig("static/hist.png")
    return

def analyze(filename):
    print("Analyze ","./download/"+filename)
    create_histogram("./download/"+filename)

def createwavmetadata(hydrophoneArrayName,projectName,lat,long,gain): # Function to create Subchunk3 meta data string
    INAM = pad_odd(hydrophoneArrayName)
    IPRD = pad_odd(projectName)
    IART = pad_odd("UW Acoustics Investigator")  # Placeholder
    ICMT = pad_odd("UW Acoustics Comments") # Placeholder
    ICRD = pad_odd(str(datetime.datetime.now()))
    IGNR = pad_odd(lat+long)
    ITRK = pad_odd(gain)

    INFO_byte_string = b"INFO"

    INAM_length_string = (len(INAM)+1).to_bytes(4, 'little')
    INAM_byte_string =b''.join([b'INAM',INAM_length_string,INAM.encode('utf-8'),bytes([0])])

    IPRD_length_string = (len(IPRD)+1).to_bytes(4, 'little')
    IPRD_byte_string =b''.join([b'IPRD',IPRD_length_string,IPRD.encode('utf-8'),bytes([0])])

    IART_length_string = (len(IART)+1).to_bytes(4, 'little')
    IART_byte_string =b''.join([b'IART',IART_length_string,IART.encode('utf-8'),bytes([0])])

    ICMT_length_string = (len(ICMT)+1).to_bytes(4, 'little')
    ICMT_byte_string =b''.join([b'ICMT',ICMT_length_string,ICMT.encode('utf-8'),bytes([0])])

    ICRD_length_string = (len(ICRD)+1).to_bytes(4, 'little')
    ICRD_byte_string =b''.join([b'ICRD',ICRD_length_string,ICRD.encode('utf-8'),bytes([0])])

    IGNR_length_string = (len(IGNR)+1).to_bytes(4, 'little')
    IGNR_byte_string =b''.join([b'IGNR',IGNR_length_string,IGNR.encode('utf-8'),bytes([0])])

    ITRK_length_string = (len(ITRK)+1).to_bytes(4, 'little')
    ITRK_byte_string =b''.join([b'ITRK',ITRK_length_string,ITRK.encode('utf-8'),bytes([0])])

    FINAL_byte_string= b''.join([INFO_byte_string,INAM_byte_string,IPRD_byte_string,IART_byte_string,ICMT_byte_string,ICRD_byte_string,IGNR_byte_string,ITRK_byte_string])

    SCK3_length = len(FINAL_byte_string)
    SCK3=b''.join([b'LIST',len(FINAL_byte_string).to_bytes(4, 'little'), FINAL_byte_string])

    return SCK3

def pad_odd(input_string):  # Function to ensure string is an odd number of characters long ready for use in WAV file meta data
    if (len(input_string) % 2) == 0:
        return input_string + ' '
    else:
        return input_string

def raspi_directory():
    dir_list = os.listdir("./download")
    wav_list=[]
    for file in dir_list:
        if (file[-3:]== "WAV"):
            wav_list.append(file)
    return wav_list

app = Flask(__name__)
app.config['SEND_FILE_MAX_AGE_DEFAULT'] = 0

@app.route('/download/<filename>')
def download (filename):
    full_filename="./download/"+filename
    return send_file(full_filename, as_attachment=True)

@app.route("/", methods=["POST","GET"])
def home():
    if request.method == "POST":
        button = request.form["button"]
        if button == "record":
            record(request.form["samplingFreq"],request.form["gain"],request.form["duration"],request.form["filePrefix"])
        elif button == "directory":
            directory()
        elif button == "transfer":
            transfer(request.form["file"],request.form["hydrophoneArrayName"],request.form["projectName"],request.form["lat"],request.form["long"],request.form["gain"])
            print("transfer complete")
        elif button == "analyze":
            print("X ",request.form["wavfile"])
            analyze(request.form["wavfile"])
        elif button == "download":
            print("Download ", request.form["wavfile"])
            print(url_for('download',filename=request.form["wavfile"]))
            return redirect(url_for('download',filename=request.form["wavfile"]))
        elif button == "delete":
            delete(request.form["file"])
        elif button == "formatSD":
            format()
        else:
            print("Do nothing")
        directory_list = directory()
        directory_list.sort(reverse=True)
        raspifile_list = raspi_directory()
        raspifile_list.sort(reverse=True)

        return redirect(url_for("home"))
    else:
        directory_list = directory()
        directory_list.sort(reverse=True)
        raspifile_list = raspi_directory()
        raspifile_list.sort(reverse=True)

        return render_template("app.html", directory_list= directory_list,raspifile_list=raspifile_list, samplingFreq_default=samplingFreq_default,gain_default=gain_default,duration_default=duration_default,filePrefix_default=filePrefix_default)

if __name__ == "__main__":
    initialize()
    app.run(host='0.0.0.0',debug=True)
