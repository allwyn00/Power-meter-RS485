# Power-meter-RS485
Sample code to read registers from XHD331-R power meter
Reading from the imput registers starting from 0x0016
Voltage , current, power,etc... is stored in two registers with 16 bit.
Data read from two registers is combined as a 32 bit and converted into float value. 
