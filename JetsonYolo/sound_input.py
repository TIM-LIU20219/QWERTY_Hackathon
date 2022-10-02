import speech_recognition as sr

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
        p = ['Apple', 'apple', 'banana', 'Banana']
        #print(set(strin.split()) & set(p))
        return set(strin.split()) & set(p)


    except Exception as e:
        print(e)
        print("Unable to understand.")
        return set()


while(True):
    output = read()
    if len(output) != 0:
        break

print(output)
