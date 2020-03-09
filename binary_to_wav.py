"""
Script used to convert a binary .txt file to a wav file

Used for MIXR debugging

Usage
python3 binary.py -> This will provide an option to choose what file to decode
and save the wav file with the same name as the input in the current directory

python3 binary.py file_name -> This will generate the output file with the given name

Input File Format
- Name: *.TXT
- Data: 32 bit signed integers (little-endian)

Default Output Format
- .wav file
- 24 bit, 44.1 kHz, Mono

"""
import struct
import wavio
import numpy as np

# import wave
import glob
import sys


# Constants
AUDIO_FREQ = 44.1 * 1000
AUDIO_WIDTH_BYTES = 3


def read_stuff(fname):
    read_data = []
    with open(fname, mode="rb") as f:
        while True:
            # Read in 4 bytes at a time
            data = f.read(4)
            if len(data) < 4:
                # End of file if there's less than 4 bytes of data
                return read_data
            # Unpack as little-endian, 32 bit numbers
            data = struct.unpack("<i", data)[0]
            read_data.append(int(data))


# Prompt the user for the file to convert
# Assume the user is using a Mac and the SD card is called MIXR
FILE_PATH = "/Volumes/MIXR/*.TXT"
files = glob.glob(FILE_PATH)
for i, f in enumerate(files):
    print("{}: {}".format(i, f))

file_to_convert = int(input("Choose a binary file to be converted into wav\n"))
file_name = files[file_to_convert]
print("Converting {}...".format(file_name))

audio_data = read_stuff(files[file_to_convert])
audio_data = np.asarray(audio_data, "int32")

# Set output file name. Path is current directory of the script
if len(sys.argv) > 1:
    output_file = str(sys.argv[1]) + ".wav"
else:
    output_file = file_name.split("/")[3][:-4] + ".wav"

wavio.write(
    output_file, audio_data, AUDIO_FREQ, sampwidth=AUDIO_WIDTH_BYTES, scale="none"
)


# # Alternatively, use Python's built in wave library (seems to take longer)
# wav_file = wave.open("0_wav.wav", "w")
# wav_file.setnchannels(1)  # mono
# wav_file.setsampwidth(3)
# wav_file.setframerate(44100)

# for samp in audio_data:
#     data_as_bytes = struct.pack("<i", int(samp))
#     wav_file.writeframes(data_as_bytes[0:3])

# wav_file.close

# import soundfile as sf
# sf.write('my_24bit_file.wav', np.asarray(audio_data, "int32"), 44100, 'PCM_24')
