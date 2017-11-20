#!/usr/bin/python
# Filename: stt.py (pevious to brl-hearts was s2t.py)
# Created: 07 May 2017
# Author : Alex Sleat followed by Derek Ripper
# Purpose: 1: capture audio by microphone
#          2; Process through one of the defined speech engines to translate to text
#          3: added by Derek - also allow forwav file input so that multiple files
#             can be processed
#          4: added by Derek - convert to run code entirely under ROS for both
#             - Normal speech recognition via microphone which listens indefinitely 
#             - Test Harness mode  (driven by th.py)
################################################################################
# Updates:
# 20 Nov 2017 Derek - add ros log info for verbal instruction feedback
#
# 15 Nov 2017 Derek - Moved wait statement to just before "record" statement. This now avoids
#                     losing gteh first word of an utterrance.
#
# 12 Nov 2017 Derek - Removed 2 bugs relating 'only' to "TH" mode, from the code restructure
#
# 11 Nov 2017 Derek - Fixed minor bugs after Zeke's code restucturing (big tidy up)
#                     when running as a listening only code now genrates a clean string in ros topic
#
# ?? Nov 2017 Zeke - restructured code as a proper class set up due to excessive organic growth!
#
# 25 Oct 2017 Derek - update to read Google_cloud keywords/phrases from a text file
#
# 20 Oct 2017 Derek - Tidy up code so that TH & Microphone params can be set
#                     appropriately for each. Previously ENERGY_THRESHOLD only got applied
#                     to Microphone ad not to the TH(Test Harness)mode.Hence TH assummed 
#                     default value of 300.     
#
# 11 Oct 2017 Derek - add command line arg processing:
#                     * option to wait for speech or listen continuously
# 
# 22 Aug 2017 Derek - In speech mode added a "wait to start listening" key
#
# 11 Aug 2017 Derek - bug fixes 
#
# 04 Aug 2017 Derek  - Added MONO to STEREO conversion for recored speech data
#
# 11 Jul 2017 derek  -Added Mic params as args + remove debug/dev print stmts.
# 
# 26 Jun 2017 Derek - add results files for mic input as needed by ERL competition
#         
################################################################################

import rospy
from std_msgs.msg import String
#import pocketsphinx
import speech_recognition as sr   # sudo pip install SpeechRecognition && sudo apt-get install python-pyaudio
import tag_topics         as TT
import gcp_keywords_r     as gcpk # google cloud platform preferred keyword/sphrases - only used by GCP speech recognition
import gcp_credentials    as gcpc # google cloud platform credentials to access GCP speech recognition

import wave, array, os            # used by mono_to_stereo()


o_tt=TT.tag_topics()

class SpeechRecognizer():
        
    def __init__(self):

        #  these are out of date - initial 30 day  tial period!!
        self.IBM_USERNAME = "c2db0a18-e3b6-4a21-9ecf-8afd2edeeb30"
        self.IBM_PASSWORD = "uAzeboVUvhuP"
          
        self.audio_sources = [ 'mic', 'file' ]
        self.speech_recognition_engines = [ 'google', 'ibm', 'sphinx', 'google_cloud' ]   
        self.r = sr.Recognizer()        
                    
    def set_audio_source(self, audio_source):
        self.audio_source = audio_source
        if self.audio_source == self.audio_sources[0]:
            self.init_mic()
            
    def set_speech_recognition_engine(self, speech_recognition_engine):
        self.speech_recognition_engine = speech_recognition_engine
        if self.speech_recognition_engine == self.speech_recognition_engines[2]:
            self.init_sphinx()
        elif self.speech_recognition_engine == self.speech_recognition_engines[3]:
           self.init_google_cloud() 
   
    def wait(self):

        if len(wait4mic) > 0 :
            print('\n************************************************')
            print(  '*** Press the "ENTER" key to start listening ***')
            print(  '************************************************\n')
            char = raw_input()
        else:
            print('\n************************************************')
            print(  '******     Continuous Listening Mode      ******')
            print(  '************************************************\n')
        
    def init_mic(self):
        self.m = sr.Microphone(device_index = None, sample_rate = 41000)
        
    def init_google_cloud(self):
        self.gcp_kwords = gcpk.gcp_keywords_r()
        
    def init_sphinx(self):
        SPHINXPATH =   rospy.get_param("SR_SPHINXPATH")
        # English-US model from 5prealpha
        ##ps_lm    = SPHINXPATH+"model2/en-us-phone.lm.bin"
        ##ps_dict  = SPHINXPATH+"model2/cmudict-en-us.dict"
        ##ps_hmm   = SPHINXPATH+"model2/en-us"
    
        ## English-US model
        ps_lm    = SPHINXPATH+"0885.lm"
        ps_dict  = SPHINXPATH+"0885.dic"
        ps_hmm   = SPHINXPATH+"model/en-us/en-us"

        ## English-Indian Model
        ##ps_lm   = SPHINXPATH+"model/en-us.lm.bin"
        ##ps_dict = SPHINXPATH+"model/en_in.dic"
        ##ps_hmm  = SPHINXPATH+"model/en_in.cd_cont_5000"
    
        config = pocketsphinx.Decoder.default_config()
        config.set_boolean("-remove_noise", False) 
        config.set_string("-lm",  ps_lm)  # language_model_file
        config.set_string("-dict",ps_dict) # phoneme_dictionary_file
        config.set_string("-hmm",ps_hmm) # 
        decoder = pocketsphinx.Decoder(config)
        
    def recognize_google(self, audio):
        # Google speech API key from Zeke Steer's account
        #API_KEY = "AIzaSyAwXXyxl0opNVs5pRCRC9HjbENGosGkl9A"
        #22/06/2017 - Derek - API key failed, but probably 
        #related to "Cloud" speech , and not the older API    

        # for testing purposes,  using the default API key
        # to use another API key, use:
        # r.recognize_google(audio, key="API_KEY")`
        # instead of `r.recognize_google(audio)`
        
        return self.r.recognize_google(audio, language="en-GB")
        
    def recognize_ibm(self, audio):
        return self.r.recognize_ibm(audio, username=self.IBM_USERNAME,  password=self.IBM_PASSWORD) 
    
    def recognize_sphinx(self, audio):
        return self.r.recognize_sphinx(audio)
    
    def recognize_google_cloud(self, audio):
        return self.r.recognize_google_cloud(
            audio,
            credentials_json  = gcpc.gcp_credentials("Zeke"),
            language          ="en-GB",
            preferred_phrases = self.gcp_kwords)
        
    def recognize(self, audio):
        text = None
        
        try:
            if self.speech_recognition_engine == self.speech_recognition_engines[0]:
                text = self.recognize_google(audio)
            elif self.speech_recognition_engine == self.speech_recognition_engines[1]:
                text = self.recognize_ibm(audio)
            elif self.speech_recognition_engine == self.speech_recognition_engines[2]:
                text = self.recognize_sphinx(audio)
            elif self.speech_recognition_engine == self.speech_recognition_engines[3]:
                text = self.recognize_google_cloud(audio)
        except sr.UnknownValueError:
            print("speech recognition engine could not understand audio")
            text = 'BAD_RECOGNITION'
        except sr.RequestError as e:
            print("could not request results from speech recognition engine: {0}".format(e))
        except TypeError:
            print("****** STT returned a None Type -- Cannot understand so continue...")
            text = 'BAD_RECOGNITION'
        except Exception as e:
            print ("STT exception e: ")
            print(e)
        except:
            print("unknown error")  
        print("\n") # make screen more readable            
        if not text is None:    
            # correctly print unicode characters to standard output
            
            if str is bytes:  # this version of Python (Python 2)
                print("Py2-You said: {}".format(text).encode("utf-8"))
                print("^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^\n")
            else:  # (Python 3+)
                print("py3-You said: {}".format(text))
        
        return text
            
    def get_audio_mic(self, energy_threshold, pause_threshold, dynamic_energy_threshold):
        print("Speech engine is: " + self.speech_recognition_engine)
        print("Energy threshold: " + str(energy_threshold))
        
        with self.m as source:
            self.r.adjust_for_ambient_noise(source)
            self.r.dynamic_energy_threshold = dynamic_energy_threshold # default is "True"
            self.r.energy_threshold = energy_threshold
            self.r.pause_threshold = pause_threshold   # Default is 0.8 secs
            
            self.wait() 
            print("*** Say something now!")
            
            return self.r.listen(source)
            
    def get_audio_file(self, energy_threshold, pause_threshold, file):
        if (run_mode == 'TH'):  index, barefile = o_tt.get_key(file)
        with sr.WavFile(barefile) as source:        
            self.r.energy_threshold = energy_threshold
            self.r.pause_threshold = pause_threshold   # Default is 0.8 secs

            return self.r.record(source)       # extract audio data from the file
            
def callback(data):
    wav_out_file_path = data.data
    rospy.loginfo(rospy.get_name() + ": Received file: %s", wav_out_file_path)
    audio = speech_recognizer.get_audio_file(energy_threshold, pause_threshold, wav_out_file_path)
    index, wav_out_file_path = o_tt.get_key(wav_out_file_path)
    text  = speech_recognizer.recognize(audio)
    
    if not text is None:
        rospy.loginfo(rospy.get_name() + ": Transcribed text is:\n" + text +
            "\n^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^")     
        text = o_tt.add_key(index, text)
        pub.publish(text + '~' + wav_out_file_path) 

def mono_to_stereo(inputfile):
# Author: Derek Ripper
# Date  : 02 Aug 2017 
# Purpose: To convert a MONO .wav file to a STEREO .wav file
# Initial code taken from:
# https://stackoverflow.com/questions/43162121/python-convert-mono-wave-file-to-stereo

    ifile       = wave.open(inputfile)
    numchannels = ifile.getnchannels()

    # convert recorded .wav file to stereo if needed 
    if numchannels == 1 :
        print("\nRecorded Wav file converted from Mono to Stereo")
        tempinput  = inputfile+"_temp"
        outputfile = inputfile
        os.rename(inputfile, tempinput)
	    # print (ifile.getparams())
	    # (1, 2, 44100, 2013900, 'NONE', 'not compressed')
        (nchannels, sampwidth, framerate, nframes, comptype, compname) = ifile.getparams()
        assert comptype == 'NONE'  # Compressed not supported yet
        array_type = {1:'B', 2: 'h', 4: 'l'}[sampwidth]
        left_channel = array.array(array_type, ifile.readframes(nframes))[::nchannels]
        ifile.close()

        # copy mono data to right channel
        stereo = 2 * left_channel
        stereo[0::2] = stereo[1::2] = left_channel
        
        # rewrite .wav file with stereo characteristcs
        ofile = wave.open(outputfile, 'w')
        ofile.setparams((2, sampwidth, framerate, nframes, comptype, compname))
        ofile.writeframes(stereo.tostring())
        ofile.close()
        os.remove(tempinput)
    else:
        # file aready in stereo
        print("\nRecorded Wav file is in stereo already - No conversion performed.")
        ifile.close() 

def cmdlineargs():
    import sys
    numargs = len(sys.argv[1:])
    arg1 = ""
#    print ("numargs *****************"+str(numargs))
#    print(sys.argv[0]) # fully path pyhon script name
#    print(sys.argv[1]) # 1st argument
#                       # plus args not set by user but set by ROS (it appears)
#    print(sys.argv[2]) # ROS node name in form            __name:=  ......
#    print(sys.argv[3]) # ROS fully pathed log name in form __log:=  ......
    if numargs == 3:
        arg1 = sys.argv[1]

    return arg1.upper()
#
#################################################################################################
#
if __name__ == "__main__":
    
    pub = rospy.Publisher("/hearts/stt", String, queue_size = 10)
    rospy.init_node("stt", anonymous = True)

    dynamic_energy_threshold = False # default is "True"
    energy_threshold = rospy.get_param("SR_ENERGY_THRESHOLD")
    pause_threshold = 1.1   # Default is 0.8 secs     
    wav_out_folder_path = rospy.get_param("SR_ERL_DATAPATHOUT")
    speech_recognition_engine = rospy.get_param("SR_speechrec_engine")
    run_mode = rospy.get_param("SR_TH")
    print("*** speech_recognition_engine: " + speech_recognition_engine)  
    print("*** Energy threshold         : " + str(energy_threshold))             
    speech_recognizer = SpeechRecognizer()    
    
    if not speech_recognition_engine in speech_recognizer.speech_recognition_engines:
        speech_recognition_engine = speech_recognizer.speech_recognition_engines[0]
    
    speech_recognizer.set_speech_recognition_engine(speech_recognition_engine)
        
    wait4mic = cmdlineargs()
   
    if run_mode =="TH": 
        rospy.loginfo(rospy.get_name() + ": audio source is wav file")
        speech_recognizer.set_audio_source("file")
        rospy.Subscriber("Wav_FileIn",String, callback)

    else:
        rospy.loginfo(rospy.get_name() + ": audio source is microphone")
        speech_recognizer.set_audio_source("mic")
        passes = 0
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
           
           audio = speech_recognizer.get_audio_mic(energy_threshold, pause_threshold, dynamic_energy_threshold) 
           rospy.loginfo("SPEECH HEARD: ")
           text  = speech_recognizer.recognize(audio)
           passes +=  1
           if not text is None: 
               text = text.strip()
               # provide break out of stt routine
               if text == "stop recording":
                       print("\n***** User command to STOP RECORDING issued *****")
                       print("*****    Use CTRL-C to kill ROS Node \n")
                       quit() 

               # ERL Competition mode for spoken phrase recognition 
               #    (ie wait on ENTER key pressto start recording )
               if len( wait4mic ) !=0: 
            
                   wav_out_file_path = wav_out_folder_path + "fb_mic_phase_speech_audio_" + str(passes) + ".wav"

                   with open(wav_out_file_path, "wb") as f:
                       f.write(audio.get_wav_data())
                       f.close()
            
                   # convert recorded .wav file to stereo if needed
                   mono_to_stereo(wav_out_file_path)           
               
                   rospy.loginfo(rospy.get_name() + ": Transcribed text is:\n" + text +
                   "\n^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^")                  
                   # this form is needed by t2cfr.py to build ERL results.txt file
                   pub.publish(text + "~" + wav_out_file_path) 

               # just pure clean text published for continuous listening mode
               else:
                   pub.publish(text) 


        rate.sleep()

rospy.spin()   
