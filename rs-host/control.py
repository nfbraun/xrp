#!/usr/bin/python
import Tkinter
import serial
import atexit

class PushButton(Tkinter.Button):
    def __init__(self, master, **kwargs):
        self.upcmd = lambda: None
        self.downcmd = lambda: None
    
        if "downcmd" in kwargs:
            self.downcmd = kwargs["downcmd"]
            del kwargs["downcmd"]
    
        if "upcmd" in kwargs:
            self.upcmd = kwargs["upcmd"]
            del kwargs["upcmd"]
            
        apply(Tkinter.Button.__init__, (self, master), kwargs)
        
        self.bind("<Button-1>", self.down_cb)
        self.bind("<ButtonRelease-1>", self.up_cb)
        self.bind("<Leave>", self.up_cb)
        self.bind("<Enter>", lambda e: "break")
        
        self.pressed = False
        
    def down_cb(self, evt):
        if not self.pressed:
            self.pressed = True
            self.downcmd()
        
    def up_cb(self, evt):
        if self.pressed:
            self.pressed = False
            self.upcmd()

class GUI:
    def __init__(self, master, dev):
        self.master = master
        self.dev = dev
        
        self.drv_pwr = Tkinter.IntVar()
        cb = Tkinter.Checkbutton(self.master, text="Enable driver stage",
                                 variable=self.drv_pwr,
                                 command=self.drv_pwr_cb)
        cb.grid(row=0, column=0, columnspan=3, sticky=Tkinter.W)
        
        Tkinter.Label(self.master, text="").grid(row=1, column=0, pady=5)
        
        self.make_button(text="Up", row=2, column=0, cmd="Z")
        self.make_button(text="Down", row=4, column=0, cmd="z")
        
        Tkinter.Label(self.master, text="").grid(row=2, column=1, padx=10)
        
        self.make_button(text="Left", row=3, column=2, cmd="X")
        self.make_button(text="Fwd", row=2, column=3, cmd="y")
        self.make_button(text="Back", row=4, column=3, cmd="Y")
        self.make_button(text="Right", row=3, column=4, cmd="x")
        
    def make_button(self, text, row, column, cmd):
        button = PushButton(self.master, text=text,
                            downcmd=lambda: self.dev.send(cmd),
                            upcmd=lambda: self.dev.send("0"))
        button.grid(row=row, column=column)
        
    def drv_pwr_cb(self):
        if self.drv_pwr.get() == 1:
            self.dev.send("D")
        else:
            self.dev.send("d")
        
class Device:
    def __init__(self):
        self.ser = serial.Serial('/dev/ttyS0', 38400, timeout=0.02, xonxoff=1, rtscts=0)
        atexit.register(self.end)

    def send(self, cmd):
        self.ser.write(cmd)
        
    def end(self):
        self.send("0")
        self.send("d")

dev = Device()

root = Tkinter.Tk()
root.title("Stepper motor control")
gui = GUI(root, dev)

root.mainloop()
