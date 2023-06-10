from PIL import Image
from playsound import playsound as ps
import PySimpleGUI as sg
import time
from termcolor import colored

# GUI.py - initial script when I was creating my GUI - this is just an empty GUI shell
splash_layout = [
    [sg.Text('Jaco Voice\n Commander',
             text_color='black',
             background_color='white',
             justification ='center',
             font=('Tw Cen MT', 72)),
     sg.Image(filename= 'Images/jackal.png')]
    ]

timeout = 4000
splash_window = sg.Window('Splash Screen', 
                          splash_layout, 
                          no_titlebar = True, 
                          keep_on_top=True,
                          background_color = 'white', 
                          grab_anywhere = True).read(timeout= timeout,close=True)

main_layout = [  
    [sg.Column([
    [sg.Image(filename='Images/jaco.png',background_color='white')],
    [sg.Text('Jaco says:',text_color='black',background_color='white',justification ='center',font=('Tw Cen MT', 20))], 
    [sg.Text('...', text_color='lightblue',background_color='black',justification ='left',size=(40,1),font=('Tw Cen MT', 20),key='-SPEAK-')],
    [sg.Text('Status:',text_color='black',background_color='white',justification ='center',font=('Tw Cen MT', 20))], 
    [sg.Text('Starting up...', text_color='yellow',background_color='black',justification ='left',size=(40,1),font=('Tw Cen MT', 20))]
    ],background_color='white'),
    sg.Column([
        [sg.Text('Joint States', text_color='blue',background_color='white',justification ='center',size=(20,1),font=('Tw Cen MT', 30))],
        [sg.Text('1 - Shoulder Lift', text_color='black',background_color='white',justification ='left',size=(20,1),font=('Tw Cen MT', 20))],
        [sg.ProgressBar(100, orientation= 'h',expand_x=True,size=(20,20),key='-JOINT1-')],
        [sg.Text('2 - Shoulder Pan', text_color='black',background_color='white',justification ='left',size=(20,1),font=('Tw Cen MT', 20))],
        [sg.ProgressBar(100, orientation= 'h',expand_x=True,size=(20,20),key='-JOINT2-')],
        [sg.Text('3 - Elbow Flex', text_color='black',background_color='white',justification ='left',size=(20,1),font=('Tw Cen MT', 20))],
        [sg.ProgressBar(100, orientation= 'h',expand_x=True,size=(20,20),key='-JOINT3-')],
        [sg.Text('4 -Wrist Pan', text_color='black',background_color='white',justification ='left',size=(20,1),font=('Tw Cen MT', 20))],
        [sg.ProgressBar(100, orientation= 'h',expand_x=True,size=(20,20),key='-JOINT4-')],
        [sg.Text('5 - Wrist Rotate', text_color='black',background_color='white',justification ='left',size=(20,1),font=('Tw Cen MT', 20))],
        [sg.ProgressBar(100, orientation= 'h',expand_x=True,size=(20,20),key='-JOINT5-')],
        [sg.Text('6 - Hand Rotate', text_color='black',background_color='white',justification ='left',size=(20,1),font=('Tw Cen MT', 20))],
        [sg.ProgressBar(100, orientation= 'h',expand_x=True,size=(20,20),key='-JOINT6-')],
        [sg.Button('EXIT')]
        ],background_color='white')],
               
        ]

main_window = sg.Window('Jaco Voice Commander',
                        main_layout,background_color='white',finalize=True
                        )

ps("Audio/boot.wav")
print(colored('JACO says:', 'blue') + " Hello There!")
speak = 'Hello there!'

main_window['-SPEAK-'].update(speak)
main_window.refresh()
ps("Audio/Hello There 2.wav")

print(colored('JACO says:', 'blue') + " I am Jaco, your robotic assistant.")
speak = 'I am Jaco, your robotic assistant.'
main_window['-SPEAK-'].update(speak)
main_window.refresh()
ps("Audio/I am Jaco.wav")

print("Moving to 'Home' State...")
armstate = 'Home'

print(colored('JACO says:', 'blue') + " I'm Listening...")
speak = "I'm listening..."
main_window['-SPEAK-'].update(speak)
main_window.refresh()
