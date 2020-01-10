import pyautogui as pg
import time
import os
def start():
    #os.system('pkill torcs')
    #os.system('torcs -nofuel -nodamage -nolaptime -vision &')
    time.sleep(1)
    pg.click(200,300)
    #pg.press('enter')
    time.sleep(.00001)
    pg.press('enter')
    time.sleep(.00001)
    pg.press('up')
    time.sleep(.00001)
    pg.press('up')
    time.sleep(.00001)
    pg.press('enter')
    time.sleep(.00001)
    pg.press('enter')
    time.sleep(.00001)
    #C= Client(p=3101)
    #C.maxSteps

def video():
    time.sleep(1)
    pg.click(200,300)
    time.sleep(1)
    #pg.press('enter')
    #time.sleep(5)
    #pg.press('enter')
    time.sleep(1)
    pg.press('down')
    time.sleep(1)
    pg.press('down')
    time.sleep(1)
    pg.press('enter')
    time.sleep(1)
    pg.press('down')
    time.sleep(1)
    for i in range(0,12):
        pg.press('shift')
        pg.press('left')
        time.sleep(1)
    pg.press('enter')
    time.sleep(.00001)	
 
