#-*-coding:utf-8-*-

import pyttsx
import sys
import time
# engine=pyttsx.init()
# engine.say("Jing Tian turtle bot 3 serve for you")
# engine.runAndWait()
# engine.endLoop()

engine = pyttsx.init()
voices = engine.getProperty('voices')
# engine.setProperty('volume', volume-0.25)
rate=engine.getProperty('rate')
engine.setProperty('rate', rate-100)
# engine.setProperty('voice',voices.v)
# engine.getProperty()
# for voice in voices:
#    engine.setProperty('voice', voice.id)
# engine.say("hi,I'm turtlebot 3.")
time.sleep(0.2)
engine.say("Chen Rui is So fullish")
engine.runAndWait()
engine.say("1")
time.sleep(5)
engine.say("2")
engine.say("3")
engine.runAndWait()