#!/usr/bin/python3
# -*- coding: UTF-8 -*-

import string
import random
import sys
sys.path.append('../serialMaster/')
from ardSerial import *
import ardSerial

from tkinter import *
from tkinter import messagebox
import tkinter.font as tkFont
from PIL import ImageTk, Image
import copy
import threading
from translate import *


width = 8
centerWidth = 2

model = 'Bittle'
postureTable = postureDict[model]

def rgbtohex(r, g, b):
    return f'#{r:02x}{g:02x}{b:02x}'
    
sixAxisNames = ['yaw', 'pitch', 'roll','Spinal','Height','Sideway']
scaleNames = [
    'Head Pan', 'Head Tilt', 'Tail Pan', 'N/A',
    'Shoulder', 'Shoulder', 'Shoulder', 'Shoulder',
    'Arm', 'Arm', 'Arm', 'Arm',
    'Knee', 'Knee', 'Knee', 'Knee']
sideNames = ['Left Front','Right Front', 'Right Back', 'Left Back']
dialTable = {'Connect':'Connected', 'Servo': 'p', 'Gyro': 'g',  'Random': 'z'}
labelSchedulerHeader = ['Repeat','Loop','Frame', 'Speed', 'Delay/ms', 'Trig','Angle','Note', 'Del', 'Add']
axisDisable = {
    'Nybble': [0,5],
    'Bittle': [0,5],
#    'DoF16' : []

}
NaJoints = {
    'Nybble': [3, 4, 5, 6, 7],
    'Bittle': [1, 2, 3, 4, 5, 6, 7],
#    'DoF16' : []
}
jointConfig = {
    'Nybble':'><',
    'Bittle':'>>',
    'DoF16' :'>>'
}
cLoop,cLabel, cSet, cSpeed, cDelay,cTrig,cAngle, cNote, cDel, cAdd = range(len(labelSchedulerHeader))
frameItemWidth =[2,1,3,3,5,2,3,5,1,1,]

#word_file = '/usr/share/dict/words'
#WORDS = open(word_file).read().splitlines()
animalNames=[# used for memorizing individual frames
    'ant','bat','bear','bee','bird','buffalo','cat','chicken','cow','dog','dolphin','duck','elephant','fish','fox','frog','goose','goat','horse','kangaroo','lion','monkey','owl','ox','penguin','person','pig','rabbit','sheep','tiger','whale','wolf','zebra']
WORDS = animalNames

language = languageList['English']

def txt(key):
    return language.get(key, textEN[key])
    

class app:

    def __init__(self):
        self.window = Tk()
        self.sliders = list()
        self.values = list()
        self.dialValue = list()
        self.controllerLabels = list()
        self.binderValue = list()
        self.binderButton = list()
        self.previousBinderValue = [0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0,]
        self.ready = 0
        # , slant='italic')
        self.myFont = tkFont.Font(
            family='Times New Roman', size=20, weight='bold')
        self.window.title(txt('title'))
        self.window.geometry('+100+10')
        self.totalFrame = 0
        self.activeFrame = 0
        self.frameList = list()
        self.frameData = [0,0,0,0,
                            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                            8,0,0,0,]
        self.originalAngle = [0,0,0,0,
                            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                            8,0,0,0,]
        self.pause = False
        self.playStop = False
        self.mirror = False
        self.createMenu()
        self.createController()
        self.createImage()
        self.createDial()
        self.createPosture()
        self.createScheduler()
        self.createSkillSchedule()
        
        self.ready = 1
        self.window.protocol('WM_DELETE_WINDOW', self.on_closing)
        self.window.update()
        self.window.mainloop()
        
    def createMenu(self):
        self.menubar = Menu(self.window, background='#ff8000', foreground='black', activebackground='white', activeforeground='black')
        file = Menu(self.menubar, tearoff=0, background='#ffcc99', foreground='black')
        for key in NaJoints:
            file.add_command(label=key,command = lambda model = key: self.changeModel(model))
        file.add_separator()
        file.add_command(label=txt('Exit'), command=self.window.quit)
        self.menubar.add_cascade(label=txt('Model'), menu=file)
        
        lan = Menu(self.menubar, tearoff=0)
        for l in languageList:
            lan.add_command(label=languageList[l]['lanOption'],command = lambda lanChoice = l: self.changLan(lanChoice))
        self.menubar.add_cascade(label=txt('lanMenu'), menu=lan)
        
        help = Menu(self.menubar, tearoff=0)
        help.add_command(label=txt('About'), command=self.about)
        self.menubar.add_cascade(label=txt('Help'), menu=help)

        self.window.config(menu=self.menubar)
        
    def createController(self):
        self.frameController = Frame(self.window)
        self.frameController.grid(row=0, column=0, rowspan=9, ipadx=10)
        label = Label(self.frameController, text=txt('Joint Controller'), font=self.myFont)
        label.grid(row=0, column=0, columnspan=width)
        self.controllerLabels.append(label)
        unbindButton = Button(self.frameController,text = txt('Unbind All'),fg = 'blue',command = self.unbindAll)
        unbindButton.grid(row=5, column=3, columnspan=2)
        self.controllerLabels.append(unbindButton)
        for i in range(16):
            PAD = 5
            cSPAN = 1
            if i < 4:
                tickDirection = 1
                cSPAN = 4
                if i < 2:
                    ROW = 0
                else:
                    ROW = 11
                if i > 0 and i < 3:
                    COL = 4
                else:
                    COL = 0
                rSPAN = 1
                ORI = HORIZONTAL
                LEN = 360

            else:
                tickDirection = -1
                leftQ = (i - 1) % 4 > 1
                frontQ = i % 4 < 2
                upperQ = i / 4 < 3

                rSPAN = 3
                ROW = 2 + (1-frontQ) * (rSPAN+2)
                if leftQ:
                    COL = (1-leftQ) * (1+centerWidth) + 3 - i // 4
                else:
                    COL = (1-leftQ) * (1+centerWidth) + 1 + i // 4
                ORI = VERTICAL
                LEN = 200
                if ROW == 2:
                    PAD = 10

            if i in NaJoints[model]:
                stt = DISABLED
                clr = 'light yellow'
            else:
                stt = NORMAL
                clr = 'yellow'
            if i in range(8,12):
                sideLabel = txt(sideNames[i%8])+'\n'
            else:
                sideLabel = '\n'
            label = Label(self.frameController,
                          text=sideLabel+'(' + str(i)+')\n'+txt(scaleNames[i]))
 
            value = DoubleVar()
            sliderBar = Scale(self.frameController, state=stt, bg=clr, variable=value, orient=ORI, borderwidth=2, from_=-180*tickDirection, to=180*tickDirection, length=LEN, tickinterval=60, resolution=1,
                              command=lambda value, idx=i:  self.setAngle(idx, value))
            sliderBar.set(0)
            label.grid(row=ROW+1, column=COL, columnspan=cSPAN, pady=2)
            sliderBar.grid(row=ROW+2, column=COL, rowspan=rSPAN, columnspan=cSPAN, ipady = 2, padx = 2)
            
            self.sliders.append(sliderBar)
            self.values.append(value)
            self.controllerLabels.append(label)
            
            if i in range(16):
                binderValue = IntVar()
                values = {"+" : 1,
#                        "." : 0,
                        "-" : -1,}
                for d in range(2):
                    button = Radiobutton(self.frameController, text = list(values)[d], variable = binderValue,value = list(values.values())[d], indicator = 0, state = stt, background = "light blue",width =1,command=lambda joint = i,idx = d :self.updateRadio(joint,idx))
                    if i<4:
                        button.grid(row=ROW+1, column=COL+(1-d)*(cSPAN-1),sticky = 's')
                    else:
                        button.grid(row=ROW+2+d*(rSPAN-1), column=COL,sticky = 'ns'[d])
                    binderValue.set(0)
                    self.binderButton.append(button)
                self.binderValue.append(binderValue)
                
                
        self.frameImu = Frame(self.frameController)
        self.frameImu.grid(row=6, column=3, rowspan=6, columnspan=2, ipady=5)
        for i in range(6):
            frm = -60
            to2 = 60
            if i in axisDisable[model]:
                stt = DISABLED
                clr = 'light yellow'
            else:
                stt = NORMAL
                clr = 'yellow'
            if i == 2:
                frm = -35
                to2 = 35
            elif i == 3:
                frm = -16
                to2 = 10
            elif i == 4:
                frm = -60
                to2 = 40
                
            
            label = Label(self.frameImu, text=txt(sixAxisNames[i]), width = 6, bg='Light Blue1')
            value = DoubleVar()
            sliderBar = Scale(self.frameImu, state=stt, bg=clr, variable=value, orient=HORIZONTAL, borderwidth=2, from_=frm, to=to2, length=120, tickinterval=(to2-frm)//4, resolution=1, command=lambda value, idx=i: self.set6Axis(idx, value))
            sliderBar.set(0)
            label.grid(row=1+i, column=3, columnspan=1)
            sliderBar.grid(row=1+i, column=4, columnspan=centerWidth)
            self.sliders.append(sliderBar)
            self.values.append(value)
            self.controllerLabels.append(label)
        
            
    def createDial(self):
        self.frameDial = Frame(self.window)
        self.frameDial.grid(row=0, column=1, ipady=5)
        labelDial = Label(self.frameDial, text=txt('State Dials'), font=self.myFont)
        labelDial.grid(row=0, column=0, columnspan=4)
        defaultValue = [1,1,1,0]
        for i in range(len(dialTable)):
            key = list(dialTable)[i]
            if len(goodPorts)>0 or i == 0:
                dialState = NORMAL
            else:
                dialState = DISABLED
            if i == 0:
                if len(goodPorts)>0:
                    defaultValue[0] = 1
                    key = 'Connected'
                else:
                    defaultValue[0] = 0
                    key = 'Connect'
            value = BooleanVar()
            button = Checkbutton(self.frameDial, text=txt(key), indicator = 0, width=8, fg='green', state = dialState,var = value,command=lambda idx=i: self.dial(idx))
            value.set(defaultValue[i])
            self.dialValue.append(value)
            if i>0:
                button.grid(row= 1, column= i+1, padx = 3)
            else:
                button.grid(row= 1, column= i, padx = 3)

        self.createPortMenu()
        
    def createPortMenu(self):
        self.port = StringVar()
        options = list(goodPorts.values())
        if len(options)>1:
            options.insert(0,txt('All'))
        elif len(options)==0:
            options.insert(0,txt('None'))
        self.portMenu = OptionMenu(self.frameDial, self.port, *options)
        self.portMenu.config(width=12, fg = 'blue')
        self.port.trace('w', lambda *args : self.changePort(''))
        self.port.set(options[0])
        self.portMenu.grid(row= 1, column= 1, padx = 3)

        
    def changePort(self,magicArg):
        global ports
        inv_dict = {v: k for k, v in goodPorts.items()}
        if self.port.get() == txt('All'):
            ports = goodPorts
        elif self.port.get() == txt('None'):
            ports = []
        else:
            singlePort = inv_dict[self.port.get()]
            ports = [singlePort]
            
            
    def createPosture(self):
        self.framePosture = Frame(self.window)
        self.framePosture.grid(row=1, column=1, ipady=5)
        labelPosture = Label(self.framePosture, text=txt('Postures'), font=self.myFont)
        labelPosture.grid(row=0, column=0, columnspan=4)
        i = 0
        for pose in postureTable:
            button = Button(self.framePosture, text=pose, fg='blue', width=10, command=lambda p=pose:  self.setPose(p))
            button.grid(row=i//4 + 1, column=i % 4)
            i += 1
            
    def createScheduler(self):
        self.frameScheduler = Frame(self.window)
        self.frameScheduler.grid(row=2, column=1, ipady=5)
        labelScheduler = Label(self.frameScheduler, text=txt('Scheduler'), font=self.myFont)
        labelScheduler.grid(row=0, column=0, columnspan=4)
        
        self.buttonRun = Button(self.frameScheduler, text=txt('Play'), width=10, fg='green', command=self.playThread)
        self.buttonRun.grid(row=1, column=0)

        buttonExp = Button(self.frameScheduler, text=txt('Import'), width=10, fg='blue', command=self.popImport)
        buttonExp.grid(row=1, column=1)
        
        buttonRestart = Button(self.frameScheduler, text=txt('Restart'), width=10, fg='red', command=self.restartScheduler)
        buttonRestart.grid(row=1, column=2)

        buttonExp = Button(self.frameScheduler, text=txt('Export'), width=10, fg='blue', command=self.export)
        buttonExp.grid(row=1, column=3)
        
        buttonUndo = Button(self.frameScheduler, text=txt('Undo'), width=10, fg='blue', state=DISABLED, command=self.restartScheduler)
        buttonUndo.grid(row=2, column=0)

        buttonRedo = Button(self.frameScheduler, text=txt('Redo'), width=10, fg='blue', state=DISABLED, command=self.restartScheduler)
        buttonRedo.grid(row=2, column=1)
        
        self.MirrorBox = Checkbutton(self.frameScheduler, text = txt('mirror'),indicator = 0, width=9,fg='blue',variable = self.mirror, onvalue=True, offvalue=False,
            command = self.setMirror).grid(row=2, column=2, sticky = 'e')
            
        buttonMirror = Button(self.frameScheduler, text = txt('>|<'), width=1,fg='blue',
            command = self.generateMirrorFrame)
        buttonMirror.grid(row=2, column=2, sticky = 'w')
        
#        Spinbox(self.frameScheduler, values=[txt('Gait'),txt('Behavior')], width=10,fg='blue',textvariable =  self.gaitOrBehavior, wrap=True).grid(row=2, column=3)
        self.gaitOrBehavior = StringVar()
        self.GorB = OptionMenu(self.frameScheduler, self.gaitOrBehavior, txt('Gait'), txt('Behavior'))
        self.GorB.config(width=9, fg = 'blue')
        self.gaitOrBehavior.set(txt('Behavior'))
        self.GorB.grid(row=2, column=3)

        
    def setMirror(self):
        self.mirror = not self.mirror
    
        
    def createSkillSchedule(self):
        self.frameSkillSchedule = Frame(self.window) #https://blog.teclado.com/tkinter-scrollable-frames/
        self.frameSkillSchedule.grid(row = 3, column = 1,ipadx=2, ipady=5)
        
        self.vLoop = IntVar()
        self.loopRepeat = Entry(self.frameSkillSchedule, width=frameItemWidth[cLoop], textvariable=self.vLoop, bd=1)
        self.loopRepeat.grid(row=0, column=cLoop)
        
        for i in range(1, len(labelSchedulerHeader)):
            Label(self.frameSkillSchedule,text = txt(labelSchedulerHeader[i]),width = frameItemWidth[i]+2, anchor='w').grid(row = 0, column = i)
        
        canvas = Canvas(self.frameSkillSchedule, width = 480, height = 470)
        scrollbar = Scrollbar(self.frameSkillSchedule, orient='vertical', command=canvas.yview)
        self.scrollable_frame = Frame(canvas)
        
        self.scrollable_frame.bind(
            '<Configure>',
            lambda e: canvas.config(
                scrollregion=canvas.bbox('all')
            )
        )
        canvas.create_window((0, 0), window=self.scrollable_frame, anchor='nw')
        canvas.config(yscrollcommand=scrollbar.set)
        
        canvas.grid(row = 1, column = 0, columnspan = len(labelSchedulerHeader))
        scrollbar.grid(row = 1, column =len(labelSchedulerHeader),sticky = 'ns')
        self.restartScheduler()

    def createImage(self):
        image = Image.open(model+'.jpg')
        ratio = image.size[0]/180
        image=image.resize((180, round(image.size[1]/ratio)))
        
        img = ImageTk.PhotoImage(image)
        self.frameImage = Label(self.frameController, image=img)
        self.frameImage.image = img
        self.frameImage.grid(row=3, column=3, rowspan=2, columnspan=2)
        
    def closeWindow(self):
        print(self.window.winfo_children())
        for w in self.window.winfo_children():
            w.destroy()
        self.window.destroy()
        
    def changLan(self,l):
        global language
        if self.ready and txt('lan') != l:
            language = languageList[l]
            self.window.title(txt('title'))
            self.menubar.destroy()
            self.controllerLabels[0].config(text = txt('Joint Controller'))
            self.controllerLabels[1].config(text = txt('Unbind All'))
            for i in range(6):
                self.controllerLabels[2 + 16 + i].config(text = txt(sixAxisNames[i]))
            for i in range(16):
                if i in range(8,12):
                    sideLabel = txt(sideNames[i%8])+'\n'
                else:
                    sideLabel = '\n'
                self.controllerLabels[2 + i].config(text=sideLabel+'(' + str(i)+')\n'+txt(scaleNames[i]))
            self.frameDial.destroy()
            self.framePosture.destroy()
            self.frameScheduler.destroy()
            self.createMenu()
            self.createDial()
            self.createPosture()
            self.createScheduler()
            for i in range(1,len(labelSchedulerHeader)):
                self.frameSkillSchedule.winfo_children()[i].config(text = txt(labelSchedulerHeader[i]))
            for r in range(len(self.frameList)):
                tt = '< '+txt('Set')
                ft = 'sans 12'
                if self.activeFrame == r:
                    ft = 'sans 14 bold'
                    if self.frameList[r][2] != self.frameData:
                        tt = '* '+ txt('Save')
                self.getWidget(r,cSet).config(text = tt, font = ft)
                
                if not self.getWidget(r, cSpeed).get().isnumeric():
                    self.getWidget(r, cSpeed).delete(0, END)
                    self.getWidget(r, cSpeed).insert(0,txt('max'))
        
    def about(self):
        messagebox.showinfo('Petoi Controller UI', 'www.petoi.com')
    
    def changeModel(self,modelName):
        global model
        global postureTable
        if self.ready and modelName != model:
            model = modelName
            postureTable = postureDict[model]
            self.framePosture.destroy()
            self.frameImage.destroy()

            for i in range(16):
                if i in NaJoints[model]:
                    stt = DISABLED
                    clr = 'light yellow'
                else:
                    stt = NORMAL
                    clr = 'yellow'
                self.sliders[i].config(state = stt, bg = clr)
                self.binderButton[i*2].config(state = stt)
                self.binderButton[i*2+1].config(state = stt)
            self.createPosture()
            self.createImage()
        
    def addFrame(self, currentRow):
        singleFrame = Frame(self.scrollable_frame, borderwidth=1, relief=RAISED)
        
        vChecked = BooleanVar()
        loopCheck = Checkbutton(singleFrame, variable = vChecked, onvalue=True, offvalue=False,
            command=lambda idx=currentRow: self.setCheckBox(idx))
        loopCheck.grid(row = 0,column = cLoop)
        
        rowLabel = Label(singleFrame, text = str(currentRow), width = frameItemWidth[cLabel])
        rowLabel.grid(row=0, column=cLabel)
        
        setButton = Button(singleFrame, text = '< '+txt('Set'), font='sans 14 bold', fg='blue', width=frameItemWidth[cSet], command=lambda idx=currentRow: self.setFrame(idx))
        
        vSpeed = StringVar()
        Spinbox(singleFrame, width=frameItemWidth[cSpeed], values = ('1','2','4','8','12','16','32','48',txt('max')), textvariable = vSpeed, wrap=True).grid(row=0, column=cSpeed)
        
        vDelay = IntVar()
        delayStep = 50
        delayOption = list(range(0,100,50))+list(range(100,1000,100))+list(range(1000,6001,1000))
        Spinbox(singleFrame, width=frameItemWidth[cDelay], values=delayOption, textvariable = vDelay, wrap=True).grid(row=0, column=cDelay)
        
        vTrig = StringVar()
        Spinbox(singleFrame, width=frameItemWidth[cTrig], values=[0,-1,1,-2,2], textvariable = vTrig, wrap=True).grid(row=0, column=cTrig)
        
        vAngle = IntVar()
        Spinbox(singleFrame, width=frameItemWidth[cAngle], from_ = -256, to = 255, textvariable = vAngle, wrap=True).grid(row=0, column=cAngle)
        
        vNote = StringVar()
#        letters = string.ascii_lowercase + string.ascii_uppercase + string.digits
#        vNote.set('note: '+ ''.join(random.choice(letters) for i in range(5)) )
        
        while True:
            note = random.choice(WORDS)
            if len(note) <= 5:
                break
        vNote.set(note + str(currentRow))  # 'note')
        color = rgbtohex(random.choice(range(64, 192)), random.choice(range(64, 192)), random.choice(range(64, 192)))
        Entry(singleFrame, width=frameItemWidth[cNote], fg=color, textvariable=vNote, bd=1).grid(row=0, column=cNote)

        delButton = Button(singleFrame, text='<', fg='red', width=frameItemWidth[cDel], command=lambda idx=currentRow: self.delFrame(idx))

        addButton = Button(singleFrame, text='v', fg='green', width=frameItemWidth[cAdd], command=lambda idx=currentRow: self.addFrame(idx + 1))

        setButton.grid(row=0, column=cSet)
        delButton.grid(row=0, column=cDel)
        addButton.grid(row=0, column=cAdd)

        self.updateButtonCommand(currentRow, 1)
        
        if currentRow ==0:
            newFrameData = copy.deepcopy(self.frameData)
        else:
            newFrameData = copy.deepcopy(self.frameList[currentRow-1][2])
            if self.activeFrame>=currentRow:
                self.activeFrame+=1
        newFrameData[3] = 0# don't add the loop tag
        vSpeed.set('8')
        vDelay.set(0)
        self.frameList.insert(currentRow, [currentRow, singleFrame, newFrameData])
        self.changeButtonState(currentRow)
        singleFrame.grid(row=currentRow + 1, column=0, columnspan=7, ipadx=1, pady=1)
#        singleFrame.update() #any update() after rebuilding the window will cause some problem in the button's function
            
    def pause(self):
        self.pause = true
            
    def delFrame(self, currentRow):
        self.frameList[currentRow][1].destroy()
        del self.frameList[currentRow]
        self.updateButtonCommand(currentRow, -1)
        if self.activeFrame == currentRow:
            if currentRow > 0:
                self.setFrame(self.activeFrame-1)
            elif self.totalFrame>self.activeFrame:
                self.activeFrame+=1
                self.setFrame(self.activeFrame-1)
        elif self.activeFrame > currentRow:
#        if self.activeFrame >= currentRow:
            self.activeFrame -= 1

    def getWidget(self, row, idx):
        frame = self.frameList[row]
        widgets = frame[1].winfo_children()
        return widgets[idx]
        
    def updateButtonCommand(self, currentRow, shift):
        for f in range(currentRow, len(self.frameList)):
            frame = self.frameList[f]
            frame[0] += shift
            widgets = frame[1].winfo_children()
            widgets[cLabel].config(text = str(frame[0])) #set
            widgets[cLoop].config(command=lambda idx=frame[0]: self.setCheckBox(idx))
            widgets[cSet].config(command=lambda idx=frame[0]: self.setFrame(idx)) #set
            widgets[cDel].config(command=lambda idx=frame[0]: self.delFrame(idx)) #delete
            widgets[cAdd].config(command=lambda idx=frame[0]: self.addFrame(idx + 1)) #add
            frame[1].grid(row=frame[0] + 1)
        self.totalFrame += shift

    def changeButtonState(self, currentRow):
        if self.totalFrame > 0:
            self.getWidget(currentRow, cSet).config(text = '< '+txt('Set'), font='sans 14 bold')
            if currentRow !=self.activeFrame:
                if self.activeFrame>=0 and self.activeFrame<self.totalFrame:
                    self.getWidget(self.activeFrame, cSet).config(text = '< '+txt('Set'), font='sans 12')
                self.activeFrame = currentRow
            self.originalAngle[0] = 0
        
    def setFrame(self, currentRow):
        currentFrame = self.frameList[currentRow]
        if currentRow != self.activeFrame:
            indexedList = list()
            for i in range(16):
                if self.frameData[4+i] != currentFrame[2][4+i]:
                    indexedList += [i,currentFrame[2][4+i]]
            self.frameData = copy.deepcopy(currentFrame[2])
            self.updateSliders(self.frameData)
            self.changeButtonState(currentRow)
            self.frameController.update()
            print(indexedList)
            if len(indexedList)>16:
                send(ports, ['L',self.frameData[4:20],0])
            elif len(indexedList):
                send(ports, ['I',indexedList,0])
        else:
            for i in range(20):
                if currentFrame[2][4+i] != self.frameData[4+i]: # the joint that's changed
                    for f in range(currentRow+1, self.totalFrame):
                        frame1 = self.frameList[f-1]
                        frame2 = self.frameList[f]
                        if frame1[2][4+i] == frame2[2][4+i]: # carry over to the next frame
                           frame2[2][4+i] = self.frameData[4+i]
                        else:
                            break
#                currentFrame[2][4+i] = self.frameData[4+i]
            currentFrame[2][4:] = copy.deepcopy(self.frameData[4:])
            
            self.getWidget(currentRow, cSet).config(text = '< '+txt('Set'), font='sans 14 bold')
        if self.totalFrame ==1:
            self.activeFrame=0
        
    def closePop(self,top):
        top.destroy()
        
    def insert_val(self,e):
        e.insert(0, 'Hello World!')
        
    def clearSkillText(self):
        self.skillText.delete("1.0","end")
        
    def loadSkillDataText(self, top):
        skillDataString = self.skillText.get('1.0','end')
        if len(skillDataString) == 1:
            messagebox.showwarning(title=Warning, message='Empty input!')
            print('Empty input!')
            return
        self.restartScheduler()
        skillData = list(map(int,''.join(skillDataString.split()).split('{')[1].split('}')[0].split(',')[:-1]))
            
        if skillData[0] < 0:
            header = 7
            frameSize = 20
            loopFrom, loopTo, repeat = skillData[4:7]
            self.vLoop.set(repeat)
            copyFrom = 4
            self.gaitOrBehavior.set(txt('Behavior'))
        else:
            header = 4
            if skillData[0] == 1: #posture
                frameSize = 16
                copyFrom = 4
            else: #gait
                if model == 'Nybble' or 'Bittle':
                    frameSize = 8
                    copyFrom = 12
                else:
                    frameSize = 12
                    copyFrom = 8
            self.gaitOrBehavior.set(txt('Gait'))
        if (len(skillData)-header)% abs(skillData[0])!=0 or frameSize != (len(skillData)-header)//abs(skillData[0]):
            messagebox.showwarning(title=Warning, message='Wrong format!')
            print('Wrong format!')
            return
        top.destroy()
        
        for f in range(abs(skillData[0])):
            if f != 0:
                self.addFrame(f)
            frame = self.frameList[f]
            frame[2][copyFrom:copyFrom+frameSize] = copy.deepcopy(skillData[header + frameSize*f:header+frameSize*(f+1)])
            if skillData[3]>1:
                frame[2][4:20] = list(map(lambda x: x*2,frame[2][4:20]))
            
            if skillData[0]< 0:
                if f == loopFrom or f == loopTo:
                    self.getWidget(f, cLoop).select()
                    frame[2][3]=1
                else:
                    frame[2][3]=0
#                    print(self.getWidget(f, cLoop).get())
                self.getWidget(f, cSpeed).delete(0, END)
                if frame[2][20] == 0:
                    self.getWidget(f, cSpeed).insert(0,txt('max'))
                else:
                    self.getWidget(f, cSpeed).insert(0,frame[2][20])
                self.getWidget(f, cDelay).delete(0, END)
                self.getWidget(f, cDelay).insert(0,frame[2][21]*50)
                
                self.getWidget(f, cTrig).delete(0, END)
                self.getWidget(f, cAngle).delete(0, END)
                self.getWidget(f, cTrig).insert(0,frame[2][22])
                self.getWidget(f, cAngle).insert(0,frame[2][23])
                
            else:
                self.getWidget(f, cSpeed).delete(0, END)
                self.getWidget(f, cSpeed).insert(0,txt('max'))
        self.activeFrame = f
        if self.totalFrame ==1:
            self.activeFrame=-1
        self.setFrame(0)

        
        
    def popImport(self):
       #Create a Toplevel window
        top= Toplevel(self.window)
        top.geometry('900x500')

       #Create an Entry Widget in the Toplevel window
        Button(top,text= txt('Open File'), width = 10, state = DISABLED, command= lambda:insert_val(entry)).grid(row = 0, column = 0)
        Button(top,text= txt('Clear'), width = 10, command= self.clearSkillText).grid(row = 0, column = 1)
       #Create a Button Widget in the Toplevel Window
        Button(top, text=txt('Cancel'), width = 10, command=lambda:self.closePop(top)).grid(row =0, column = 2 )
        Button(top, text=txt('Ok'), width = 10, command= lambda:self.loadSkillDataText(top)).grid(row =0, column = 3 )
       
        entryFrame = Frame(top)
        entryFrame.grid(row = 1, column = 0, columnspan = 4,padx = 10,pady = 10)
                
        self.skillText= Text(entryFrame, width= 120, spacing1= 2)
        self.skillText.insert('1.0',txt('exampleFormat')
        +'\n\nconst int8_t hi[] PROGMEM ={\n\
    -4,  0, -30, 1,\n\
     1,  2,   3,\n\
     0,-20, -60,   0,   0,   0,   0,   0,  35,  30, 120, 105,  75,  60, -40, -30,     4, 2, 0, 0,\n\
    35, -5, -60,   0,   0,   0,   0,   0, -75,  30, 125,  95,  40,  75, -45, -30,    10, 0, 0, 0,\n\
    40,  0, -35,   0,   0,   0,   0,   0, -60,  30, 125,  95,  60,  75, -45, -30,    10, 0, 0, 0,\n\
     0,  0, -45,   0,  -5,  -5,  20,  20,  45,  45, 105, 105,  45,  45, -45, -45,     8, 0, 0, 0,\n\
};')
        self.skillText.grid(row = 0, column = 0)
        
        scrollY = Scrollbar(entryFrame, width = 20, orient = VERTICAL)
        scrollY.grid(row = 0, column = 1, sticky='ns')
        scrollY.config(command=self.skillText.yview)
        self.skillText.config(yscrollcommand=scrollY.set)
        
#        scrollX = Scrollbar(entryFrame, width = 20, orient = VERTICAL)
#        scrollX.grid(row = 1, column = 0, sticky='ew')
#        scrollX.config(command=self.skillText.xview)
#        self.skillText.config(xscrollcommand=scrollX.set)
        entryFrame.columnconfigure(0, weight=1)
        entryFrame.rowconfigure(0, weight=1)
        
    def playThread(self):
        self.playStop = False
        self.buttonRun.config(text = txt('Stop'), fg = 'red',command = self.stop)
        t=threading.Thread(target=self.play)
        t.start()
        
    def play(self):
        if self.activeFrame+1 == self.totalFrame:
            self.getWidget(self.activeFrame, cSet).config(text = '< '+txt('Set'), font='sans 12')
            self.activeFrame = 0;
        for f in range(self.activeFrame, self.totalFrame):
            if self.playStop:
                break
            frame = self.frameList[f]
            
            indexedList = list()
            for i in range(16):
                if self.frameData[4+i] != frame[2][4+i]:
                    indexedList += [i,frame[2][4+i]]
            self.frameData = copy.deepcopy(frame[2])
            self.updateSliders(self.frameData)
            self.changeButtonState(f)
            if len(indexedList)>16:
                send(ports, ['L',self.frameData[4:20],0])
            elif len(indexedList):
                send(ports, ['I',indexedList,0])

        self.buttonRun.config(text = txt('Play'), fg = 'green',command = self.playThread)
        self.playStop = False
        
        
    def stop(self):
        self.buttonRun.config(text = txt('Play'), fg = 'green',command = self.playThread)
        self.playStop = True
        
    def mirrorAngles(self,singleFrame):
        singleFrame[1]=-singleFrame[1]
        singleFrame[4]=-singleFrame[4]
        singleFrame[4+2]=-singleFrame[4+2]
        for i in range(4,16,2):
            singleFrame[4+i],singleFrame[4+i+1] = singleFrame[4+i+1],singleFrame[4+i]
        if abs(singleFrame[22]) == 2:
            singleFrame[22]=-singleFrame[22]
            singleFrame[23]=-singleFrame[23]
            
    def generateMirrorFrame(self):
        self.values[16+2].set(-1*self.values[16+2].get())
        self.values[16+5].set(-1*self.values[16+5].get())
        self.mirrorAngles(self.originalAngle)
        self.mirrorAngles(self.frameData)
        self.updateSliders(self.frameData)
        self.indicateEdit()
        self.frameController.update()
        send(ports, ['L',self.frameData[4:20],0])
                    
    def export(self):
        if self.activeFrame+1 == self.totalFrame:
            self.getWidget(self.activeFrame, cSet).config(text = '< '+txt('Set'), font='sans 12')
            self.window.update()
            self.activeFrame = 0;
        skillData = list()
        loopStructure = list()
        period = self.totalFrame - self.activeFrame
        if model =='Nybble' or model == 'Bittle':
            copyFrom = 12
            frameSize = 8
        else:
            copyFrom = 8
            frameSize = 12
        if self.gaitOrBehavior.get() == txt('Behavior'):
            period = -period
            copyFrom = 4
            frameSize = 20
        if self.totalFrame == 1:
            period = 1
            copyFrom = 4
            frameSize = 16
        angleRatio = 1
        startFrame = self.activeFrame
        for f in range(startFrame, self.totalFrame):
            frame = self.frameList[f]
            self.frameData = copy.deepcopy(frame[2])
            if max(self.frameData[4:20]) > 125 or min(self.frameData[4:20]) < -125:
                angleRatio = 2
            if(self.frameData[3]==1):
                loopStructure.append(f - startFrame)
            if self.getWidget(f, cSpeed).get() == txt('max'):
                self.frameData[20] = 0
            else:
                self.frameData[20] = int(self.getWidget(f, cSpeed).get())
            self.frameData[21] = int(self.getWidget(f, cDelay).get())//50
            self.frameData[22] = int(self.getWidget(f, cTrig).get())
            self.frameData[23] = int(self.getWidget(f, cAngle).get())
            if self.mirror:
                self.mirrorAngles(self.frameData)
            self.updateSliders(self.frameData)
            self.changeButtonState(f)
            self.frameController.update()
            skillData.append(self.frameData[copyFrom: copyFrom + frameSize])
        if period == 1:
            print(self.frameData[4:20])
            send(ports, ['L',self.frameData[4:20],0])
            return
            
        if angleRatio ==2:
            for r in skillData:
                print(r)
                if frameSize == 8 or frameSize == 12:
                    r = list(map(lambda x: x//angleRatio,r))
                elif frameSize == 20:
                    r[:16] = list(map(lambda x: x//angleRatio,r[:16]))
        if len(loopStructure)==0:
            loopStructure = [0]
        print('{')
        print('{:>4},{:>4},{:>4},{:>4},'.format(*[period, 0, 0, angleRatio]))
        if period <0 and self.gaitOrBehavior.get() == txt('Behavior'):
            print('{:>4},{:>4},{:>4},'.format(*[ loopStructure[0], loopStructure[-1], self.loopRepeat.get()]))
        for row in skillData:
            print(('{:>4},'*frameSize).format(*row))
        print('};')
        
        if self.gaitOrBehavior.get() == txt('Behavior'):
            skillData.insert(0,[loopStructure[0], loopStructure[-1], int(self.loopRepeat.get())])
        skillData.insert(0,[period, 0, 0, angleRatio])
        flat_list = [item for sublist in skillData for item in sublist]
        print(flat_list)
        send(ports, ['K',flat_list,0])


    def restartScheduler(self):
        for f in self.frameList:
            f[1].destroy()
        self.frameList.clear()
        self.frameData = [0,0,0,0,
                            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                            8,0,0,0,]
        self.totalFrame = 0
        self.activeFrame = 0
        self.addFrame(0)
        self.vLoop.set(0)
#        self.window.update()
#        self.setPose('calib')
        
    def indicateEdit(self):
        frame = self.frameList[self.activeFrame]
        if frame[2] != self.frameData:
            self.getWidget(self.activeFrame, cSet).config(text = '* '+ txt('Save'), font='sans 14 bold')
#            print('frm',frame[2])
#            print('dat',self.frameData)
        else:
            self.getWidget(self.activeFrame, cSet).config(text = '< '+txt('Set'), font='sans 14 bold')
            
    def setCheckBox(self,currentRow):
        frame = self.frameList[currentRow]
        if frame[2][3] == 0:
            frame[2][3] = 1
        else:
            frame[2][3] = 0
    
    
    def unbindAll(self):
        for i in range(16):
            self.binderValue[i].set(0)
            self.previousBinderValue[i]=0
            self.changeRadioColor(i,0)
        self.controllerLabels[1].config(fg = 'blue')
            
    def changeRadioColor(self,joint,value): # -1, 0, 1
        if value:
            self.binderButton[joint*2 + (1-value)//2].configure(background ='red')
            self.binderButton[joint*2 + (value + 1)//2].configure(background ='light blue')
        else:
            self.binderButton[joint*2].configure(background ='light blue')
            self.binderButton[joint*2+1].configure(background ='light blue')
        self.binderButton[joint*2].update()
        self.binderButton[joint*2+1].update()
        if 1 in self.previousBinderValue or -1 in self.previousBinderValue:
            self.controllerLabels[1].config(fg = 'red')
        else:
            self.controllerLabels[1].config(fg = 'blue')
    
    def updateRadio(self,joint,idx):
        if(self.previousBinderValue[joint]==self.binderValue[joint].get()):
            self.binderValue[joint].set(0)
        self.previousBinderValue[joint]=self.binderValue[joint].get()
        self.changeRadioColor(joint,self.binderValue[joint].get())
        
    def setAngle(self, idx, value):
        if self.ready == 1:
            value = int(value)
            if self.binderValue[idx].get() ==0:
                self.frameData[4 + idx] = value
                if value>-126 and value<126:
                    send(ports, ['I', [idx, value], 0])
                else:
                    send(ports, ['i', [idx, value], 0])
            else:
                diff = value - self.frameData[4 + idx]
                indexedList = list()
                for i in range(16):
                    if self.binderValue[i].get():
                        self.frameData[4+i] +=diff*self.binderValue[i].get()*self.binderValue[idx].get()
                        indexedList+=[i,self.frameData[4+i]]
                send(ports, ['I',indexedList, 0])
                
            self.indicateEdit()
            self.updateSliders(self.frameData)
            

    def set6Axis(self, i, value): # a more precise function should be based on inverse kinematics
        value = int(value)
        if self.ready == 1:
#            send(ports, ['t', [i, value], 0.0])
            if self.originalAngle[0]==0:
                self.originalAngle[4:20] = copy.deepcopy(self.frameData[4:20])
                self.originalAngle[0]=1
            if i == 0:#ypr
                positiveGroup = []
                negativeGroup = []
            elif i == 1:#pitch
                if jointConfig[model] == '>>':
                    positiveGroup = [2,6,7,10,11,12,13]
                    negativeGroup = [1,4,5,8,9,14,15]
                else:
                    positiveGroup = [6,7,12,13,14,15]
                    negativeGroup = [1,4,5,8,9,10,11,]
            elif i == 2:#roll
                if jointConfig[model] == '>>':
                    positiveGroup = [4,7,8,11,13,14]
                    negativeGroup = [0,5,6,9,10,12,15]
                else:
                    positiveGroup = [4,7,8,10,13,15]
                    negativeGroup = [0,2,5,6,9,11,12,14]
            elif i == 3:#Spinal
                if jointConfig[model] == '>>':
                    positiveGroup = [8,9,10,11,12,13,14,15]
                    negativeGroup = []
                else:
                    positiveGroup = [8,9,10,11,12,13,14,15]
                    negativeGroup = []
                
            elif i == 4:#Height
                if jointConfig[model] == '>>':
                    positiveGroup = [12,13,14,15]
                    negativeGroup = [8,9,10,11,]
                else:
                    positiveGroup = [12,13,10,11,]
                    negativeGroup = [8,9,14,15]
            elif i == 5:#Sideway
                if jointConfig[model] == '>>':
                    positiveGroup = []
                    negativeGroup = []
            
            for j in range(16):
                leftQ = (j - 1) % 4 > 1
                frontQ = j % 4 < 2
                upperQ = j / 4 < 3
                factor = 1
                if j > 3:
                    if not upperQ:
                        factor = 2
                    else:
                        factor = 1
                    if i == 1:
                        if (value > 0 and frontQ and upperQ) or (value < 0 and not frontQ and upperQ):
                            factor /= 4
                    if i == 2:

                        if (value > 0 and not leftQ) or (value < 0 and leftQ):
                            factor /=2

                if j in positiveGroup:
                    self.frameData[4+j] = self.originalAngle[4+j] + int(value*factor)
                if j in negativeGroup:
                    self.frameData[4+j] = self.originalAngle[4+j] - int(value*factor)


            send(ports, ['L',self.frameData[4:20],0])
            self.updateSliders(self.frameData)
            self.indicateEdit()

    def setPose(self, pose):
        if self.ready == 1:
            self.getWidget(self.activeFrame, cNote).delete(0, END)
            self.getWidget(self.activeFrame, cNote).insert(0, pose + str(self.activeFrame))
            self.updateSliders(postureTable[pose])
            self.indicateEdit()
            for i in range(6):
                self.values[16+i].set(0)
            send(ports, ['k'+pose, 0])
            if pose == 'rest':
                send(ports, ['d', 0])
    
    def setSpeed(self):
        self.frameData[20] = self.getWidget(self.activeFrame, cSpeed).get()
        
    def setDelay(self):
        self.frameData[21] = int(self.getWidget(self.activeFrame, cDelay).get())
            
    def updateSliders(self, angles ):
        for i in range(16):
            self.values[i].set(angles[4+i])
            self.frameData[4+i] = angles[4+i]

    def dial(self, i):
        if self.ready == 1:
            global ports
            global goodPorts
            buttons = self.frameDial.winfo_children()[2:5]
            key = list(dialTable)[i]
            if key == 'Connect':
                if len(goodPorts)>0:
                    state = send(ports, ['b', [10,90],0],1)
                    closeAllSerial(goodPorts)
                    self.portMenu['menu'].delete(0,'end')
                    self.port.set(txt('None'))
                    self.frameDial.winfo_children()[1].config(text = txt('Connect'),fg = 'red')
                    self.dialValue[0].set(False)
                    for b in buttons:
                        b.config(state = DISABLED)
                else:
                    self.dialValue[0].set(True)
                    self.frameDial.winfo_children()[0].update()
                    connectPort()
                    self.portMenu.destroy()
                    self.createPortMenu()
                    state = send(ports, ['b', [10,90],0])
                    if len(goodPorts)>0:
                        self.frameDial.winfo_children()[1].config(text = txt('Connected'),fg='green')
                        for b in buttons:
                            b.configure(state = NORMAL)
                    else:
                        self.dialValue[0].set(False)
                        self.frameDial.winfo_children()[0].update()
                    
            elif len(goodPorts)>0:
                state = send(ports, [dialTable[key], 0])
                if state == 'p':
                    self.dialValue[i].set(True)
                    self.frameDial.winfo_children()[2].config(fg = 'green')
                elif state == 'P':
                    self.dialValue[i].set(False)
                    self.frameDial.winfo_children()[2].config(fg = 'red')
                elif state == 'g':
                    self.dialValue[i].set(False)
                    self.frameDial.winfo_children()[3].config(fg = 'red')
                elif state == 'G':
                    self.dialValue[i].set(True)
                    self.frameDial.winfo_children()[3].config(fg = 'green')
                elif state == 'z':
                    self.dialValue[i].set(False)
                    self.frameDial.winfo_children()[4].config(fg = 'red')
                elif state == 'Z':
                    self.dialValue[i].set(True)
                    self.frameDial.winfo_children()[4].config(fg = 'green')

    def on_closing(self):
        if messagebox.askokcancel(txt('Quit'), txt('Do you want to quit?')):
            self.window.destroy()
    

if __name__ == '__main__':
    try:
        connectPort()
        ports = goodPorts
#        time.sleep(2)
#        if len(goodPorts)>0:
#            t=threading.Thread(target=keepReadingSerial,args=(goodPorts,))
#            t.start()
        app()
        closeAllSerial(goodPorts)
    except Exception as e:
        logger.info("Exception")
        closeAllSerial(goodPorts)
        raise e

# unused text codes for references
#        letters = WORDS#string.ascii_lowercase + string.ascii_uppercase + string.digits
#        vNote.set('note: '+ ''.join(random.choice(letters) for i in range(5)) )#'note')
