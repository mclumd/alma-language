#!/usr/bin/env python
"""
Adapted from code by Sophie Li, 2016
Updated, modified, expanded by Samuel Barham and Matthew Goldberg)

Note: we subscribe to the 'language_input' topic, which is the topic to
which all language input devices (read 'nodes') should publish their data.
Typically, this will be the the 'speech_recognizer' node, but in other cases,
we may wish to simplify the language interface -- removing the noise and inconsis-
tency of imperfect speech recognition -- by using console input, for which
another node has been written.
"""

from pocketsphinx.pocketsphinx import *
#from pocketsphinx import get_model_path, get_data_path
from sphinxbase.sphinxbase import *

import rospy
#from std_msgs.msg import String
import std_msgs
import os
import sys
import pyaudio
import wave
import audioop
from collections import deque
import time
import math


class SpeechDetector:
    def __init__(self):
        # Microphone stream config.
        self.CHUNK = 1024  # CHUNKS of bytes to read each time from mic
        self.FORMAT = pyaudio.paInt16
        self.CHANNELS = 1
        self.RATE = 16000

        self.SILENCE_LIMIT = 2  # Silence limit in seconds. The max ammount of seconds where
                           # only silence is recorded. When this time passes the
                           # recording finishes and the file is decoded

        self.PREV_AUDIO = 0.5  # Previous audio (in seconds) to prepend. When noise
                          # is detected, how much of previously recorded audio is
                          # prepended. This helps to prevent chopping the beginning
                          # of the phrase.

        self.THRESHOLD = 4500
        self.num_phrases = -1

        # These will need to be modified according to where the pocketsphinx folder is
        #MODELDIR = get_model_path()
        #DATADIR = get_data_path() + "/test"
        # Temporarily hardcoded to Sphinx English model directories; fix later!
        MODELDIR = "/home/mcl/Documents/cmusphinx/pocketsphinx-5prealpha/model/en-us"
        DATADIR = "/home/mcl/Documents/cmusphinx/pocketsphinx-5prealpha/test/data"

        # Create a decoder with certain model
        config = Decoder.default_config()
        config.set_string('-hmm', os.path.join(MODELDIR, 'en-us'))
        config.set_string('-lm', os.path.join(MODELDIR, 'en-us.lm.bin'))
        config.set_string('-dict', os.path.join(MODELDIR, 'cmudict-en-us.dict'))
        # config.set_string('verbose', False)

        # Create decoder object for streaming data.
        self.decoder = Decoder(config)

    def setup_mic(self, num_samples=50):
        """ Gets average audio intensity of your mic sound. You can use it to get
            average intensities while you're talking and/or silent. The average
            is the avg of the .2 of the largest intensities recorded.
        """
        print "Getting intensity values from mic."
        p = pyaudio.PyAudio()
        stream = p.open(format=self.FORMAT,
                        channels=self.CHANNELS,
                        rate=self.RATE,
                        input=True,
                        frames_per_buffer=self.CHUNK)

        values = [math.sqrt(abs(audioop.avg(stream.read(self.CHUNK), 4)))
                  for x in range(num_samples)]
        values = sorted(values, reverse=True)
        r = sum(values[:int(num_samples * 0.2)]) / int(num_samples * 0.2)
        print " Finished "
        print " Average audio intensity is ", r
        stream.close()
        p.terminate()

        if r < 3000:
            self.THRESHOLD = 3500
        else:
            self.THRESHOLD = r + 100

    def save_speech(self, data, p):
        """
        Saves mic data to temporary WAV file. Returns filename of saved
        file
        """
        filename = 'output_'+str(int(time.time()))
        # writes data to WAV file
        data = ''.join(data)
        wf = wave.open(filename + '.wav', 'wb')
        wf.setnchannels(1)
        wf.setsampwidth(p.get_sample_size(pyaudio.paInt16))
        wf.setframerate(16000)  # TODO make this value a function parameter?
        wf.writeframes(data)
        wf.close()
        return filename + '.wav'

    def decode_phrase(self, wav_file):
        self.decoder.start_utt()
        stream = open(wav_file, "rb")
        while True:
          buf = stream.read(1024)
          if buf:
            self.decoder.process_raw(buf, False, False)
          else:
            break
        self.decoder.end_utt()
        words = []
        [words.append(seg.word) for seg in self.decoder.seg()]
        return words

    def detected_word(self, words, target):
        count = 0
        for word in words:
          if word == target:
            count += 1
            self.pub.publish("Detected instance " + str(count) + " of word " + target)
        if count > 0:
          return True

    def run(self):
        """
        Listens to Microphone, extracts phrases from it and calls pocketsphinx
        to decode the sound
        """
        self.setup_mic()

        #Open stream
        p = pyaudio.PyAudio()

        stream = p.open(format=self.FORMAT,
                        channels=self.CHANNELS,
                        rate=self.RATE,
                        input=True,
                        frames_per_buffer=self.CHUNK)

        print "* Mic set up and listening. "

        audio2send = []
        cur_data = ''  # current chunk of audio data
        rel = self.RATE/self.CHUNK
        slid_win = deque(maxlen=self.SILENCE_LIMIT * rel)
        #Prepend audio from 0.5 seconds before noise was detected
        prev_audio = deque(maxlen=self.PREV_AUDIO * rel)
        started = False

        self.pub = rospy.Publisher("language_input", std_msgs.msg.String, queue_size=5)
        rospy.init_node("speech_recognizer")

        while True:
            cur_data = stream.read(self.CHUNK)
            slid_win.append(math.sqrt(abs(audioop.avg(cur_data, 4))))

            if sum([x > self.THRESHOLD for x in slid_win]) > 0:
                if started == False:
                    print "Starting recording of phrase"
                    started = True
                audio2send.append(cur_data)

            elif started:
                print "Finished recording, decoding phrase"
                filename = self.save_speech(list(prev_audio) + audio2send, p)
                r = self.decode_phrase(filename)
                print "DETECTED: ", r
                self.pub.publish(' '.join(r))

                # self.detected_word(r, "bottle")
                # if self.detected_word(r, "goodbye"):
                #   stream.close()
                #   p.terminate()
                #   sys.exit()

                # Removes temp audio file
                os.remove(filename)
                # Reset all
                started = False
                slid_win = deque(maxlen=self.SILENCE_LIMIT * rel)
                prev_audio = deque(maxlen=0.5 * rel)
                audio2send = []
                print "Listening ..."

            else:
                prev_audio.append(cur_data)

        print "* Done listening"
        stream.close()
        p.terminate()

if __name__ == "__main__":
    sd = SpeechDetector()
    sd.run()
