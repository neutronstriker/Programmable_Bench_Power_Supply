@echo off
rem: Use this file along with "Arduino_board" upload settings to help you change the port and baudrate for the board that you are using.
set port=COM17
set baud=57600
BlueIsp.cmd %1 m328p %port% %baud%