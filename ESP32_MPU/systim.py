from machine import Timer

class SYSTIM():
    def cb(self,t):
        if self.pause == False:
            self.cnt = self.cnt+1
    
    def __init__(self, timerID=0):
        self.cnt = 0
        self.pause = True
        self.tim = Timer(timerID)
        self.tim.init(mode=Timer.PERIODIC, freq=10000, callback=self.cb)

    def pause(self):
        self.pause = True

    def go_on(self):
        self.pause = False
