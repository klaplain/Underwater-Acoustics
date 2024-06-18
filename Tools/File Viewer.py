f = open("C:\\Users\\klapl\\OneDrive\\Documents\\BARN\\Underwater_Acoustics\\WAV_files\\test300d3.wav", "rb")

thesebytes = bytes(16)
# Read 16 bytes
row_count =0
while row_count <32768:
    print(f"{row_count*16:0{4}x}", end = " ")
    thesebytes = f.read(16)
    for thisbyte in range(16):
        print(f"{thesebytes[thisbyte]:0{2}x}", end = " ")
    print("   ", end=" ")
    for thisbyte in range(16):
        printbyte = thesebytes[thisbyte]
        if printbyte <48 or printbyte>122:
            printbyte=46
        print(chr(printbyte), end = " ")
    row_count += 1
    print()


