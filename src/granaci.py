from gtts import gTTS
import os
from playsound import playsound
# Language in which you want to convert
language = 'es'

    # Passing the text and language to the engine,
    # here we have marked slow=False. Which tells
    # the module that the converted audio should
    # have a high speed
try:
    myobj = gTTS(text="Letra A", lang=language, slow=False)
    myobj.save("A.mp3")

    # Playing the converted file
    playsound.playsound('open A.wav')
    # Saving the converted audio in a mp3 file named
    # welcome
except:
    pass