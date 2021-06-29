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

engaged = False
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
    #s = "recvd:{} Ok CRC".format(pkt.hex())
    s = ""
    val = struct.unpack("<H", pkt[1:])[0]
    print("code: {:x} val:{}".format(pkt[0], val))
    if pkt[0] == 0x20:
        s += "Current: " + state_decode(val)
    elif pkt[0] == 0x21:
        s += "Previous: " + state_decode(val)
    elif pkt[0] == 0x1c:
        s += "Current: {}".format(val)
    elif pkt[0] == 0xb3:
        s += "Voltage: {}".format(val)
    elif pkt[0] == 0xf9:
        s += "Controller Temp: {}".format(val)
    elif pkt[0] == 0x48:
        s += "Motor Temp: {}".format(val)
    elif pkt[0] == 0xa7:
        s += "Rudder Angle: {}".format(val)
    elif pkt[0] == 0x8f:
        flags = []
        if val & 0x1:
            flags.append("SYNC")
        if val & 0x2:
            flags.append("TEMPFAULT")
        if val & 0x4:
            flags.append("CURFAULT")
        if val & 0x8:
            flags.append("ENGAGED")
        if val & 0x10:
            flags.append("INVALID")
        if val & 0x20:
            flags.append("PORTFAULT")
        if val & 0x40:
            flags.append("STBDFAULT")
        if val & 0x80:
            flags.append("BV")
        if val & 0x100:
            flags.append("R-")
        if val & 0x200:
            flags.append("R+")
        if val & 0x400:
            flags.append("CURRNG")
        if val & 0x800:
            flags.append("BADFUSE")
        if val & 0x1000:
            flags.append("REBOOT")
        s += "Flags: {}".format(" ".join(flags))
        flags_var.set("".join(flags))
    elif pkt[0] == 0x9a:
        s += "EEPROM[{}]: 0x{:02x}".format(val & 0xFF, val >> 8)
    write_to_serial_widget(s + '\n')
    
    
def build_pkt(typecode, value):
    pkt = struct.pack("<BH", typecode, value)
    crc = calc_crc8(pkt)
    pkt = struct.pack("<BHB", typecode, value, crc[0])
    return pkt

def calc_crc8(msg):
    hash = crc8.crc8(initial_start=0xFF)
    hash.update(msg)
    #print("crc8:0x{}".format(hash.hexdigest()))
    return hash.digest()
    
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
    #print("numlines:",numlines)
    serial_output['state'] = 'normal'
    if numlines == 20:
        serial_output.delete(1.0, 2.0)
    serial_output.insert('end', s)
    serial_output['state'] = 'disabled'

def write_eeprom():
    addr = int(write_addr_var.get())
    byte_val = int(write_byte_var.get())
    if byte_val >= 0 and byte_val <= 256:
        print("write eeprom[{}]: 0x{:02x}".format(addr, byte_val))
        send_packet(build_pkt(0x53, addr + (byte_val << 8)))
    else:
        print("invalid eeprom write value")
    
def read_eeprom():
    start_addr = int(start_var.get())
    end_addr = int(end_var.get())
    if start_addr < end_addr:
        print("read eeprom[{}] -> [{}]".format(start_addr, end_addr))
        send_packet(build_pkt(0x91, start_addr + (end_addr << 8)))
    else:
        print("read range invalid")

def poll_serial():
    global msg
    read_bytes = serial_port.read()
    if len(read_bytes) != 0:
        msg += read_bytes
        #print("buf:{}".format(msg.hex()))
        if len(msg) >= 4:
            pkt = msg[:3]
            crc = calc_crc8(pkt)
            if crc[0] == msg[3]:
                handle_recvd_pkt(pkt)
                msg = msg[4:]
            else:
                print("bad crc:{}".format(pkt.hex()))
                msg = msg[1:]
    if engaged:
        update_position(position_scale.get())
    root.after(200, poll_serial)
    
def update_position(val):
    pos = int(round(float(val) * 10))
    print("Position:", val, pos)
    try:
        if serial_port and engaged and pos >= -2000 and pos <= 2000:
            send_packet(build_pkt(0xc7, pos))
    except NameError:
        # this happens when widget is initialized
        pass

def engage():
    global engaged
    print("engage")
    engaged = True
    
def reset(*args):
    print("reset")
    send_packet(build_pkt(0xe7, 0))

def disengage(*args):
    global engaged
    print("disengage")
    pkt = build_pkt(0x68, 0)
    send_packet(pkt)
    engaged = False

root = Tk()
root.title("pypilot-controller device test")

mainframe = ttk.Frame(root, padding="3 3 12 12")
mainframe.grid(column=0, row=0, sticky=(N, W, E, S))
root.columnconfigure(0, weight=1)
root.rowconfigure(0, weight=1)


ttk.Label(mainframe, text="position").grid(column=1, row=1, sticky=W)
position_scale = ttk.Scale(mainframe, orient=HORIZONTAL, length=200, from_=0, to=200, command=update_position)
position_scale.set(100)
position_scale.grid(column=2, row=1, columnspan=3)

ttk.Button(mainframe, text="Engage", command=engage).grid(column=1, row=2)
ttk.Button(mainframe, text="Disengage", command=disengage).grid(column=2, row=2)
ttk.Button(mainframe, text="Reset", command=reset).grid(column=3, row=2)

ttk.Label(mainframe, text="Flags:").grid(column=1, row=3, sticky=W)
flags_var = StringVar()
ttk.Entry(mainframe, textvariable=flags_var).grid(column=2, row=3, columnspan=3)

ttk.Button(mainframe, text="Read EEPROM", command=read_eeprom).grid(column=1, row=4)
ttk.Label(mainframe, text="start address").grid(column=2, row=4, sticky=E)
start_var = StringVar()
ttk.Entry(mainframe, textvariable=start_var).grid(column=3, row=4)
start_var.set("0")
ttk.Label(mainframe, text="end address").grid(column=4, row=4, sticky=E)
end_var = StringVar()
ttk.Entry(mainframe, textvariable=end_var).grid(column=5, row=4)
end_var.set("0")

ttk.Button(mainframe, text="Write EEPROM", command=write_eeprom).grid(column=1, row=5)
ttk.Label(mainframe, text="address").grid(column=2, row=5, sticky=E)
write_addr_var = StringVar()
ttk.Entry(mainframe, textvariable=write_addr_var).grid(column=3, row=5)
write_addr_var.set("0")
ttk.Label(mainframe, text="byte").grid(column=4, row=5, sticky=E)
write_byte_var = StringVar()
ttk.Entry(mainframe, textvariable=write_byte_var).grid(column=5, row=5)
write_byte_var.set("0")

ttk.Label(mainframe, text="serial output").grid(column=1, row=6, sticky=W)

serial_output = Text(mainframe, width=80, height=20)
serial_output.grid(column=1, row=7, columnspan=5)
txt = serial_output.get('1.0', 'end')
print("txt:", txt)

for child in mainframe.winfo_children(): 
    child.grid_configure(padx=5, pady=5)

position_scale.focus()
#root.bind("<Return>", calculate)

open_serial()
root.after(20, poll_serial)

root.mainloop()
close_serial()
