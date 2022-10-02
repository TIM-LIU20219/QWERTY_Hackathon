import speech_recognition as sr
import os
dir_path = os.path.dirname(os.path.realpath(__file__))
# print(dir_path)
stopper = set(['stop', 'finish', 'terminate', 'quit', 'goodbye'])


def read():
    r = sr.Recognizer()
    with sr.Microphone() as source:
        print("Listening...")
        r.pause_threshold = .5
        audio = r.listen(source)
    try:
        print("Regognizing...")
        strin = r.recognize_google(audio, language='en-us')
        print(strin.split())
        #print(set(strin.split()) & set(p))
        if len(set(strin.split()) & stopper) != 0:
            print("quiting")
            return 0

    except Exception as e:
        print(e)
        print("Unable to understand.")
        return set()


def myCommand():
    r = sr.Recognizer()
    with sr.Microphone() as source:
        audio = r.listen(source, phrase_time_limit=3)
    try:
        command = r.recognize_google(audio).lower()
        print("you said: " + command)
        command = command.split()
        # print(command)
        with open(dir_path + '/utils/command.txt', 'w') as fp:
            # write each item on a new line
            for word in command:
                fp.write("%s\n" % word)
            print('Done')
        #print(set(strin.split()) & set(p))
        if len(set(command) & stopper) != 0:
            print("quiting")
            return 0

        # with open(r'VoiceAndFace/utils/command.txt', 'w') as fp:

    except sr.UnknownValueError:
        print("Sorry, Cant understand, Please say again")
        # command = myCommand()


while True:
    ret = myCommand()
    if ret == 0:
        print("quited")
        break
