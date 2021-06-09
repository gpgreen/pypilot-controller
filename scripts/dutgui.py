#!/usr/bin/env python3
''' Python script to hardware test the pypilot-controller

Motor commands can be sent via the gui, and the received messages
can be seen in text
'''

import crc8
import serial
import struct
from tkinter import *
from tkinter import ttk

msg = bytearray()

def state_decode(state):
    if state == 0:
        s = "Start"
    elif state == 1:
        s = "WaitEntry"
    elif state == 2:
        s = "Wait"
    elif state == 3:
        s = "Engage"
    elif state == 4:
        s = "Operational"
    elif state == 5:
        s = "DisengageEntry"
    elif state == 6:
        s = "Disengage"
    elif state == 7:
        s = "DetachEntry"
    elif state == 8:
        s = "Detach"
    elif state == 9:
        s = "PowerDown"
    else:
        s = "Unknown"
    return s

def handle_recvd_pkt(pkt):
    s = "recvd:{} Ok CRC".format(pkt.hex())
    val = struct.unpack(">H", pkt[1:])[0]
    print("code: {:x} val:{}".format(pkt[0], val))
    if pkt[0] == 0x20:
        s += "\nCurrent: " + state_decode(val)
    elif pkt[0] == 0x21:
        s += "\nPrevious: " + state_decode(val)
    write_to_serial_widget(s + '\n')
    
    
def build_pkt(typecode, value):
    pkt = struct.pack(">BH", typecode, value)
    crc = calc_crc8(pkt)
    pkt = struct.pack(">BHB", typecode, value, crc[0])
    return pkt

def open_serial():
    global serial_port
    s = serial.Serial("/dev/ttyUSB0", 115200, timeout=0)
    s.close()
    s.open()
    serial_port = s

def close_serial():
    global serial_port
    serial_port.close()
    serial_port = None

def send_packet(pkt):
    global serial_port
    print("Sending pkt:", pkt.hex())
    serial_port.write(pkt)
    
def write_to_serial_widget(s):
    numlines = int(serial_output.index('end - 1 line').split('.')[0])
    serial_output['state'] = 'normal'
    if numlines == 20:
        serial_output.delete(1.0, 2.0)
    serial_output.insert('end', s)
    serial_output['state'] = 'disabled'
    
def poll_serial():
    global msg
    read_byte = serial_port.read()
    if len(read_byte) != 0:
        msg = msg + read_byte
        if len(msg) == 4:
            pkt = msg[:3]
            crc = calc_crc8(pkt)
            if crc[0] == msg[3]:
                handle_recvd_pkt(pkt)
                msg = b''
            else:
                msg = msg[1:]
    root.after(1, poll_serial)
    
def update_position(val):
    print("Position:", val)
    pos = int(round(float(val) * 10))
    print("Position:", val, pos)
    try:
        if serial_port and pos >= -2000 and pos <= 2000:
            pkt = build_pkt(0xc7, pos)
            send_packet(pkt)
    except NameError:
        # this happens when widget is initialized
        pass
    
def reset(*args):
    print("reset")
    pkt = build_pkt(0xe7, 0)
    send_packet(pkt)

def disengage(*args):
    print("disengage")
    pkt = build_pkt(0x68, 0)
    send_packet(pkt)

def calc_crc8(msg):
    hash = crc8.crc8(initial_start=0xFF)
    hash.update(msg)
    #print("crc8:0x{}".format(hash.hexdigest()))
    return hash.digest()
    
root = Tk()
root.title("pypilot-controller device test")

mainframe = ttk.Frame(root, padding="3 3 12 12")
mainframe.grid(column=0, row=0, sticky=(N, W, E, S))
root.columnconfigure(0, weight=1)
root.rowconfigure(0, weight=1)

position = StringVar()
position_scale = ttk.Scale(mainframe, orient=HORIZONTAL, length=200, from_=0, to=200, command=update_position)
position_scale.set(100)
position_scale.grid(column=2, row=1, sticky=(W, E))

ttk.Button(mainframe, text="Reset", command=reset).grid(column=1, row=2, sticky=W)
ttk.Button(mainframe, text="Disengage", command=disengage).grid(column=2, row=2, sticky=W)

flags = StringVar()
ttk.Label(mainframe, textvariable=flags).grid(column=1, row=3, sticky=(W, E))

serial_output = Text(mainframe, width=80, height=20)
serial_output.grid(column=1, row=5, sticky=(W, E))
txt = serial_output.get('1.0', 'end')
print("txt:", txt)

ttk.Label(mainframe, text="position").grid(column=1, row=1, sticky=W)
ttk.Label(mainframe, text="flags").grid(column=1, row=3, sticky=W)
ttk.Label(mainframe, text="serial output").grid(column=1, row=4, sticky=W)

for child in mainframe.winfo_children(): 
    child.grid_configure(padx=5, pady=5)

position_scale.focus()
#root.bind("<Return>", calculate)

open_serial()
root.after(1, poll_serial)

root.mainloop()
close_serial()