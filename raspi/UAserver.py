from flask import Flask, redirect, url_for, render_template, request, send_file
import time, datetime,json
import spidev
import RPi.GPIO as GPIO
import os

import numpy as np                                                      # for data transformation
import matplotlib.pyplot as plt                                         # for visualizing the data
import scipy.io.wavfile as wavfile                                      # for opening the media file
import math                                                             # Importing libraries using import keyword.

directory_listing =""

BUFFER_SIZE = 4096
filetransferbuffer = [0]*BUFFER_SIZE
SD_Card_file_size_dict = {}                                             # Initialise/Empty dictionary.  We will need this dictionary for the SD card files later.
SD_status = ""                                                          # SD Card usage

# Initialize SPI interface
bus = 0                                                                 # We only have SPI bus 0 available to us on the Pi
device = 0                                                              # Device is the chip select pin. Set to 0 or 1, depending on the connections
spi = spidev.SpiDev()                                                   # Enable SPI SD_Card_file_size_dict = {}
spi.open(bus, device)                                                   # Open a connection to a specific bus and device (chip select pin)
spi.max_speed_hz = 12000000                                             # Set SPI speed and mode  2700000 seems to be the maximum stable value.  2.7MHz clock => 337kBytes/sec theoretical
spi.mode = 0
spi.no_cs = True

def show_buffer(buffer):
    for buffer_index in range(0,BUFFER_SIZE,16):
        this_row=buffer[buffer_index:buffer_index+16]
        row_count=int(buffer_index/16)
        print(f"{row_count*16:0{4}x}", end = " ")
        for thisbyte in range(16):
            print(f"{this_row[thisbyte]:0{2}x}", end = " ")
        print("   ", end=" ")
        for thisbyte in range(16):
            printbyte = this_row[thisbyte]
            if printbyte <32 or printbyte>122:
                printbyte=46
            print(chr(printbyte), end = "")
        print()
    return

def stm_to_raspi():                                                     # Get a block of data from the STM32 into the RASPI
    returned_values = [0]*BUFFER_SIZE
    while True:
        while GPIO.input(27) == GPIO.HIGH  :  pass                      # Wait until STM ACQ is Ready before requesting datablock
        returned_values = spi.readbytes(BUFFER_SIZE)
        #show_buffer(returned_values)
        return returned_values

def raspi_to_stm(datablock):                                            # Send a block of data from the RASPI to the STM32
    while GPIO.input(27) == GPIO.HIGH  :  pass                          # Wait until STM ACQ is Ready before sending datablock
    spi.writebytes(datablock)
    while GPIO.input(27) == GPIO.HIGH  :  pass                          # Wait until STM ACQ is Ready before returning
    return

def send_command_to_stm(command_string):
    print(command_string)
    command=bytearray()
    command.extend(command_string.encode())
    command.extend([0]*(BUFFER_SIZE-len(command)))
    raspi_to_stm(command)

def directory():
    global SD_Card_file_size_dict, SD_status
    send_command_to_stm(">DIR")
    returned_directory = stm_to_raspi()                                 #get the directory data from STM32

    directory_string = ""
    for character in returned_directory:
        if character == 12:
            break
        directory_string += chr(character)                              # and put it all into a string

    directory_line = directory_string.replace("//","").split("\n")      # strip out // and break down into separate lines
    SD_status = directory_line[0]                                       # first line describes disk status / usage  - maybe this should be displayed separately
    SD_Card_file_size_dict.clear()                                      # Empty dictionary.  We will need this dictionary for the SD card files later.

    for index in range(1,len(directory_line)):                          # each of the next lines describes a file except for the last 3 lines
        directory_line_items = directory_line[index].split("\t")            # now split out the filename, size, date and time
        if directory_line_items[0].find(".DAT")>0:
            SD_Card_file_size_dict[directory_line_items[0]] = int(directory_line_items[1] )
        else:
            del directory_line[index]                                              # This isn't a valid file line
    directory_line.sort(reverse=True)
    return directory_line[:-1]                                           # Send the file directory but exclude the directory usage

def set_datetime():                                                     # Function to set date and time on Acquisition Subsystem
    print("Set datetime")
    now=datetime.datetime.now()
    send_command_to_stm(f">TIM,{now.hour},{now.minute},{now.second},4,{now.month},{now.day},{now.year-2000}")
    return

def transfer(file,hydrophoneArrayName,projectName,lat,long,gain):       # Function to transfer file from STM32 to RASPI
    global SD_Card_file_size_dict
    this_filename =file.split(" ")[0]
    print("Transfer",this_filename)
    send_command_to_stm(f">XFR,{this_filename}")

    f=open("./download/"+this_filename[:-3]+"WAV","wb")                 #open file for writing on RASPI.

    TotalBytes= SD_Card_file_size_dict.get(this_filename)
    TotalInts=TotalBytes>>1
    print("Samples =",TotalInts," Bytes =", TotalBytes, end="")
    bytestoread=TotalBytes
    tic = time.perf_counter()
    while True:
        if bytestoread > BUFFER_SIZE:
            f.write(bytearray(stm_to_raspi()))
            bytestoread=bytestoread-BUFFER_SIZE
        elif bytestoread == 0:
            break
        else:
            f.write(bytearray(stm_to_raspi()[0:bytestoread]))           # we must have less than BUFFER_SIZE bytes so we need to truncate the bytearray
            bytestoread = 0
            f.write(createwavmetadata(hydrophoneArrayName,projectName,lat,long,gain))
    f.close()
    toc = time.perf_counter()
    print(f"  Time to save {toc - tic:0.4f} secs.  ",int(TotalBytes/(toc-tic))/1000000,"Mbytes per sec" )
    print("Transfer Complete")
    return

def format():                                                           # Function to format SD card on STM32
    print("Format SD")
    send_command_to_stm(">FMT")
    print("Format Complete")
    return

def delete(file):                                                       # Function to delete file on STM32 SD card
    this_filename =file.split(" ")[0]
    print("Delete",this_filename)
    send_command_to_stm(f">DEL,//{this_filename}")
    print("Deletion Complete")
    return

def deletewav(file):                                                       # Function to delete file on STM32 SD card
    this_filename =file.split(" ")[0]
    print("Delete",this_filename)
    if os.path.exists("./download/"+this_filename[:-3]+"WAV"):
            os.remove("./download/"+this_filename[:-3]+"WAV")                   #delete file
    print("Deletion WAV file Complete")
    return


def record(samplingFreq, gain, duration,filePrefix):    # Function to record sound onto .DAT file on STM32 SD card

    config_dict.update({"samplingFreq": samplingFreq})
    config_dict.update({"gain": gain})
    config_dict.update({"duration": duration})
    config_dict.update({"filePrefix": filePrefix})
    with open("config.cfg", "w") as config_file:
        json.dump(config_dict, config_file)  # encode dict into JSON

    print("Recording",samplingFreq, gain, duration,filePrefix)
    this_filename = filePrefix+datetime.datetime.now().strftime("%Y%m%d%H%M%S")+".DAT"
    send_command_to_stm(f">REC,{samplingFreq}, {gain}, {duration},{this_filename}")
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
    if os.path.isfile("static/hist.png"):
         os.remove("static/hist.png")
    plt.savefig("static/hist.png")
    plt.close()
    return

def analyze(filename):
    print("Analyze ","./download/"+filename)
    create_histogram("./download/"+filename)

def createwavmetadata(hydrophoneArrayName,projectName,lat,long,gain):   # Function to create Subchunk3 meta data string
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

def pad_odd(input_string):                                              # Function to ensure string is an odd number of characters long ready for use in WAV file meta data
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
    wav_list.sort(reverse=True)
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
        elif button == "analyze":
            print("X ",request.form["wavfile"])
            analyze(request.form["wavfile"])
        elif button == "download":
            print("Download ", request.form["wavfile"])
            return redirect(url_for('download',filename=request.form["wavfile"]))
        elif button == "delete":
            delete(request.form["file"])
        elif button == "deletewav":
            deletewav(request.form["wavfile"])
        elif button == "formatSD":
            format()
        else:
            print("Do nothing")
        directory_list = directory()
        raspifile_list = raspi_directory()

        return redirect(url_for("home"))
    else:
        directory_list = directory()
        raspifile_list = raspi_directory()

        return render_template("app.html", SD_status=SD_status, directory_list=directory_list,raspifile_list=raspifile_list, samplingFreq=config_dict.get("samplingFreq"),gain=config_dict.get("gain")  ,duration=config_dict.get("duration"),filePrefix=config_dict.get("filePrefix") )

if __name__ == "__main__":
    print("Initializing")
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    GPIO.setup(22, GPIO.OUT)                                            # Reset STM32
    GPIO.output(22, GPIO.LOW)
    time.sleep(0.5)
    GPIO.output(22, GPIO.HIGH)                                          #now release RESET pin

    GPIO.setup(27, GPIO.IN, pull_up_down=GPIO.PUD_UP)                   # STM32 SPI Busy

    set_datetime()

# Read config file to get last used settings for slections and drop downs
    with open("config.cfg", "r") as config_file:# Load the dictionary from the file
        config_dict = json.load(config_file)# Set defaults for global variables

    directory_list = directory()
    raspifile_list = raspi_directory()

    app.run(host='0.0.0.0',debug=True)
