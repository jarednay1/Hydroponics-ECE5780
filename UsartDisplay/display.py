#Jared Nay - u1167483, Tyler Evans - u1313811, Inhyup Lee - u1214753
# Woojin Lee - u1398084
#this is the display code for the pico
#ASTHON APPROVED THIS AS A FUNCTIONAL DISPLAY
#THIS IS NOTHING MORE OR NOTHING LESS.
from machine import UART, Pin
#link to lib used. 
#https://pypi.org/project/raspberrypi-tm1637/
#credit due to
import tm1637
import time
#pins (0 TX) (1 RX)
#link here
numb  = 1
led = Pin("LED", Pin.OUT)
uart = UART(0, baudrate=115200, tx=Pin(0), rx=Pin(1))
tm = tm1637.TM1637(clk=Pin(16), dio=Pin(17))
while True:
    time.sleep(0.2)
    data = str(uart.read())
    #print(data)
    #print(type(data))
    if data == "b'0'":
        tm.number(0)
    elif data == "b'1'":
        tm.number(5)
    elif data == "b'2'":
        tm.number(10)
    elif data == "b'3'":
        tm.number(15)
    elif data == "b'4'":
        tm.number(9999)
         
    
