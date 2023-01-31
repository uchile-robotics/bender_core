import speech_recognition as sr
r = sr.Recognizer()
with sr.Microphone() as source:                # use the default microphone as the audio source
    while(True):
        r.adjust_for_ambient_noise(source)         # listen for 1 second to calibrate the energy threshold for ambient noise levels
        audio = r.listen(source)                   # now when we listen, the energy threshold is already set to a good value, and we can reliably catch speech right away
        try:
            print("You said " + r.recognize(audio))    # recognize speech using Google Speech Recognition
        except LookupError:                            # speech is unintelligible
            print("Could not understand audio")