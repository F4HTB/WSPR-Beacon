EESchema Schematic File Version 2
LIBS:power
LIBS:device
LIBS:transistors
LIBS:conn
LIBS:linear
LIBS:regul
LIBS:74xx
LIBS:cmos4000
LIBS:adc-dac
LIBS:memory
LIBS:xilinx
LIBS:microcontrollers
LIBS:dsp
LIBS:microchip
LIBS:analog_switches
LIBS:motorola
LIBS:texas
LIBS:intel
LIBS:audio
LIBS:interface
LIBS:digital-audio
LIBS:philips
LIBS:display
LIBS:cypress
LIBS:siliconi
LIBS:opto
LIBS:atmel
LIBS:contrib
LIBS:valves
LIBS:arduino
LIBS:balise_wspr-cache
EELAYER 25 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L CONN_01X05 P2
U 1 1 59B408E9
P 6750 2050
F 0 "P2" H 6750 2350 50  0000 C CNN
F 1 "GPS" V 6850 2050 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x05_Pitch2.54mm" H 6750 2050 50  0001 C CNN
F 3 "" H 6750 2050 50  0000 C CNN
	1    6750 2050
	1    0    0    -1  
$EndComp
$Comp
L CONN_01X07 P7
U 1 1 59B4094C
P 6900 3800
F 0 "P7" H 6900 4200 50  0000 C CNN
F 1 "SI5351" V 7000 3800 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x07_Pitch2.54mm" H 6900 3800 50  0001 C CNN
F 3 "" H 6900 3800 50  0000 C CNN
	1    6900 3800
	1    0    0    1   
$EndComp
$Comp
L CONN_01X04 P5
U 1 1 59B40993
P 5500 2850
F 0 "P5" H 5500 3100 50  0000 C CNN
F 1 "LCD" V 5600 2850 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x04_Pitch2.54mm" H 5500 2850 50  0001 C CNN
F 3 "" H 5500 2850 50  0000 C CNN
	1    5500 2850
	1    0    0    -1  
$EndComp
$Comp
L C C9
U 1 1 59B40A2B
P 4750 2950
F 0 "C9" H 4775 3050 50  0000 L CNN
F 1 "100nF" H 4775 2850 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 4788 2800 50  0001 C CNN
F 3 "" H 4750 2950 50  0000 C CNN
	1    4750 2950
	1    0    0    -1  
$EndComp
$Comp
L L L2
U 1 1 59B40AAB
P 4700 3500
F 0 "L2" V 4650 3500 50  0000 C CNN
F 1 "VK200" V 4775 3500 50  0000 C CNN
F 2 "Choke_Axial_ThroughHole:Choke_Horizontal_RM15mm" H 4700 3500 50  0001 C CNN
F 3 "" H 4700 3500 50  0000 C CNN
	1    4700 3500
	0    1    1    0   
$EndComp
$Comp
L +5V #PWR01
U 1 1 59B40F01
P 2950 1000
F 0 "#PWR01" H 2950 850 50  0001 C CNN
F 1 "+5V" H 2950 1140 50  0000 C CNN
F 2 "" H 2950 1000 50  0000 C CNN
F 3 "" H 2950 1000 50  0000 C CNN
	1    2950 1000
	1    0    0    -1  
$EndComp
$Comp
L +5V #PWR02
U 1 1 59B40F25
P 4550 3450
F 0 "#PWR02" H 4550 3300 50  0001 C CNN
F 1 "+5V" H 4550 3590 50  0000 C CNN
F 2 "" H 4550 3450 50  0000 C CNN
F 3 "" H 4550 3450 50  0000 C CNN
	1    4550 3450
	1    0    0    -1  
$EndComp
$Comp
L +5V #PWR03
U 1 1 59B40F34
P 4650 2800
F 0 "#PWR03" H 4650 2650 50  0001 C CNN
F 1 "+5V" H 4650 2940 50  0000 C CNN
F 2 "" H 4650 2800 50  0000 C CNN
F 3 "" H 4650 2800 50  0000 C CNN
	1    4650 2800
	0    -1   -1   0   
$EndComp
$Comp
L GND #PWR04
U 1 1 59B40F54
P 2950 4650
F 0 "#PWR04" H 2950 4400 50  0001 C CNN
F 1 "GND" H 2950 4500 50  0000 C CNN
F 2 "" H 2950 4650 50  0000 C CNN
F 3 "" H 2950 4650 50  0000 C CNN
	1    2950 4650
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR05
U 1 1 59B41167
P 4050 3150
F 0 "#PWR05" H 4050 2900 50  0001 C CNN
F 1 "GND" H 4050 3000 50  0000 C CNN
F 2 "" H 4050 3150 50  0000 C CNN
F 3 "" H 4050 3150 50  0000 C CNN
	1    4050 3150
	0    -1   -1   0   
$EndComp
$Comp
L GND #PWR06
U 1 1 59B41298
P 6300 3600
F 0 "#PWR06" H 6300 3350 50  0001 C CNN
F 1 "GND" H 6300 3450 50  0000 C CNN
F 2 "" H 6300 3600 50  0000 C CNN
F 3 "" H 6300 3600 50  0000 C CNN
	1    6300 3600
	1    0    0    -1  
$EndComp
$Comp
L L L1
U 1 1 59B416B0
P 4750 1850
F 0 "L1" V 4700 1850 50  0000 C CNN
F 1 "100uH" V 4825 1850 50  0000 C CNN
F 2 "Capacitors_THT:CP_Radial_D10.0mm_P3.80mm" H 4750 1850 50  0001 C CNN
F 3 "" H 4750 1850 50  0000 C CNN
	1    4750 1850
	0    1    1    0   
$EndComp
$Comp
L +5V #PWR07
U 1 1 59B416B6
P 4600 1800
F 0 "#PWR07" H 4600 1650 50  0001 C CNN
F 1 "+5V" H 4600 1940 50  0000 C CNN
F 2 "" H 4600 1800 50  0000 C CNN
F 3 "" H 4600 1800 50  0000 C CNN
	1    4600 1800
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR08
U 1 1 59B416BC
P 5650 2200
F 0 "#PWR08" H 5650 1950 50  0001 C CNN
F 1 "GND" H 5650 2050 50  0000 C CNN
F 2 "" H 5650 2200 50  0000 C CNN
F 3 "" H 5650 2200 50  0000 C CNN
	1    5650 2200
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR09
U 1 1 59B416C4
P 6250 1950
F 0 "#PWR09" H 6250 1700 50  0001 C CNN
F 1 "GND" H 6250 1800 50  0000 C CNN
F 2 "" H 6250 1950 50  0000 C CNN
F 3 "" H 6250 1950 50  0000 C CNN
	1    6250 1950
	1    0    0    -1  
$EndComp
$Comp
L C C7
U 1 1 59B416CC
P 5650 2000
F 0 "C7" H 5675 2100 50  0000 L CNN
F 1 "100nF" H 5550 1900 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 5688 1850 50  0001 C CNN
F 3 "" H 5650 2000 50  0000 C CNN
	1    5650 2000
	-1   0    0    -1  
$EndComp
$Comp
L GND #PWR010
U 1 1 59B4179B
P 4750 3100
F 0 "#PWR010" H 4750 2850 50  0001 C CNN
F 1 "GND" H 4750 2950 50  0000 C CNN
F 2 "" H 4750 3100 50  0000 C CNN
F 3 "" H 4750 3100 50  0000 C CNN
	1    4750 3100
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR011
U 1 1 59B4187F
P 4950 2700
F 0 "#PWR011" H 4950 2450 50  0001 C CNN
F 1 "GND" H 4950 2550 50  0000 C CNN
F 2 "" H 4950 2700 50  0000 C CNN
F 3 "" H 4950 2700 50  0000 C CNN
	1    4950 2700
	0    1    1    0   
$EndComp
Text GLabel 2150 3150 0    60   Input ~ 0
SDA
Text GLabel 2150 3250 0    60   Input ~ 0
SCL
Text GLabel 6650 3700 0    60   Input ~ 0
SDA
Text GLabel 6650 3800 0    60   Input ~ 0
SCL
Text GLabel 5250 2900 0    60   Input ~ 0
SDA
Text GLabel 5250 3000 0    60   Input ~ 0
SCL
Text GLabel 6500 2150 0    60   Input ~ 0
RX
Text GLabel 3700 2900 2    60   Input ~ 0
TX
Text GLabel 6500 2250 0    60   Input ~ 0
PPS
Text GLabel 6650 3900 0    60   Input ~ 0
OUT0
Text GLabel 6650 4100 0    60   Input ~ 0
OUT2
Text GLabel 3700 3550 2    60   Input ~ 0
PPS
Text GLabel 3700 3000 2    60   Input ~ 0
RX
Text GLabel 6500 2050 0    60   Input ~ 0
TX
Text GLabel 3700 3300 2    60   Input ~ 0
OUT0
Text GLabel 7050 2500 0    60   Input ~ 0
OUT2
$Comp
L R R6
U 1 1 59B42F7D
P 3850 3150
F 0 "R6" V 3930 3150 50  0000 C CNN
F 1 "1k" V 3850 3150 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 3780 3150 50  0001 C CNN
F 3 "" H 3850 3150 50  0000 C CNN
	1    3850 3150
	0    1    1    0   
$EndComp
$Comp
L GND #PWR012
U 1 1 59B43661
P 5300 2200
F 0 "#PWR012" H 5300 1950 50  0001 C CNN
F 1 "GND" H 5300 2050 50  0000 C CNN
F 2 "" H 5300 2200 50  0000 C CNN
F 3 "" H 5300 2200 50  0000 C CNN
	1    5300 2200
	1    0    0    -1  
$EndComp
$Comp
L C C6
U 1 1 59B43667
P 5300 2000
F 0 "C6" H 5325 2100 50  0000 L CNN
F 1 "1uF" H 5325 1900 50  0000 L CNN
F 2 "Capacitors_THT:CP_Radial_D4.0mm_P1.50mm" H 5338 1850 50  0001 C CNN
F 3 "" H 5300 2000 50  0000 C CNN
	1    5300 2000
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR013
U 1 1 59B44093
P 5950 2200
F 0 "#PWR013" H 5950 1950 50  0001 C CNN
F 1 "GND" H 5950 2050 50  0000 C CNN
F 2 "" H 5950 2200 50  0000 C CNN
F 3 "" H 5950 2200 50  0000 C CNN
	1    5950 2200
	1    0    0    -1  
$EndComp
$Comp
L C C8
U 1 1 59B44099
P 5950 2000
F 0 "C8" H 5975 2100 50  0000 L CNN
F 1 "10nF" H 5950 1900 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 5988 1850 50  0001 C CNN
F 3 "" H 5950 2000 50  0000 C CNN
	1    5950 2000
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR014
U 1 1 59B4442B
P 5400 3850
F 0 "#PWR014" H 5400 3600 50  0001 C CNN
F 1 "GND" H 5400 3700 50  0000 C CNN
F 2 "" H 5400 3850 50  0000 C CNN
F 3 "" H 5400 3850 50  0000 C CNN
	1    5400 3850
	1    0    0    -1  
$EndComp
$Comp
L C C11
U 1 1 59B44431
P 5400 3650
F 0 "C11" H 5425 3750 50  0000 L CNN
F 1 "100nF" H 5425 3550 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 5438 3500 50  0001 C CNN
F 3 "" H 5400 3650 50  0000 C CNN
	1    5400 3650
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR015
U 1 1 59B44438
P 5050 3850
F 0 "#PWR015" H 5050 3600 50  0001 C CNN
F 1 "GND" H 5050 3700 50  0000 C CNN
F 2 "" H 5050 3850 50  0000 C CNN
F 3 "" H 5050 3850 50  0000 C CNN
	1    5050 3850
	1    0    0    -1  
$EndComp
$Comp
L C C10
U 1 1 59B4443E
P 5050 3650
F 0 "C10" H 5075 3750 50  0000 L CNN
F 1 "1uF" H 5075 3550 50  0000 L CNN
F 2 "Capacitors_THT:CP_Radial_D4.0mm_P1.50mm" H 5088 3500 50  0001 C CNN
F 3 "" H 5050 3650 50  0000 C CNN
	1    5050 3650
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR016
U 1 1 59B44445
P 5700 3850
F 0 "#PWR016" H 5700 3600 50  0001 C CNN
F 1 "GND" H 5700 3700 50  0000 C CNN
F 2 "" H 5700 3850 50  0000 C CNN
F 3 "" H 5700 3850 50  0000 C CNN
	1    5700 3850
	1    0    0    -1  
$EndComp
$Comp
L C C12
U 1 1 59B4444B
P 5700 3650
F 0 "C12" H 5725 3750 50  0000 L CNN
F 1 "10nF" H 5725 3550 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 5738 3500 50  0001 C CNN
F 3 "" H 5700 3650 50  0000 C CNN
	1    5700 3650
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR017
U 1 1 59B44A8E
P 5000 2200
F 0 "#PWR017" H 5000 1950 50  0001 C CNN
F 1 "GND" H 5000 2050 50  0000 C CNN
F 2 "" H 5000 2200 50  0000 C CNN
F 3 "" H 5000 2200 50  0000 C CNN
	1    5000 2200
	1    0    0    -1  
$EndComp
$Comp
L C C5
U 1 1 59B44A94
P 5000 2000
F 0 "C5" H 5025 2100 50  0000 L CNN
F 1 "100uF" H 5025 1900 50  0000 L CNN
F 2 "Capacitors_THT:CP_Radial_D10.0mm_P3.50mm" H 5038 1850 50  0001 C CNN
F 3 "" H 5000 2000 50  0000 C CNN
	1    5000 2000
	1    0    0    -1  
$EndComp
$Comp
L R R3
U 1 1 59B44E80
P 7200 2650
F 0 "R3" V 7280 2650 50  0000 C CNN
F 1 "100" V 7200 2650 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 7130 2650 50  0001 C CNN
F 3 "" H 7200 2650 50  0000 C CNN
	1    7200 2650
	1    0    0    -1  
$EndComp
$Comp
L R R4
U 1 1 59B44EE3
P 8050 2650
F 0 "R4" V 8150 2650 50  0000 C CNN
F 1 "47" V 8050 2650 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 7980 2650 50  0001 C CNN
F 3 "" H 8050 2650 50  0000 C CNN
	1    8050 2650
	1    0    0    -1  
$EndComp
$Comp
L R R1
U 1 1 59B44F48
P 7450 2500
F 0 "R1" V 7530 2500 50  0000 C CNN
F 1 "470" V 7450 2500 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 7380 2500 50  0001 C CNN
F 3 "" H 7450 2500 50  0000 C CNN
	1    7450 2500
	0    1    1    0   
$EndComp
$Comp
L POT RV1
U 1 1 59B44FB1
P 7800 2500
F 0 "RV1" V 7625 2500 50  0000 C CNN
F 1 "1k" V 7700 2500 50  0000 C CNN
F 2 "Potentiometers:Potentiometer_Trimmer_Bourns_3266Y" H 7800 2500 50  0001 C CNN
F 3 "" H 7800 2500 50  0000 C CNN
	1    7800 2500
	0    1    -1   0   
$EndComp
$Comp
L GND #PWR018
U 1 1 59B45170
P 8050 2850
F 0 "#PWR018" H 8050 2600 50  0001 C CNN
F 1 "GND" H 8050 2700 50  0000 C CNN
F 2 "" H 8050 2850 50  0000 C CNN
F 3 "" H 8050 2850 50  0000 C CNN
	1    8050 2850
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR019
U 1 1 59B4519C
P 7200 2850
F 0 "#PWR019" H 7200 2600 50  0001 C CNN
F 1 "GND" H 7200 2700 50  0000 C CNN
F 2 "" H 7200 2850 50  0000 C CNN
F 3 "" H 7200 2850 50  0000 C CNN
	1    7200 2850
	1    0    0    -1  
$EndComp
$Comp
L CONN_01X02 P4
U 1 1 59B42D4C
P 8600 2550
F 0 "P4" H 8600 2700 50  0000 C CNN
F 1 "SMA" V 8700 2550 50  0000 C CNN
F 2 "Connectors_RF:sma_straight_32k101-400l5" H 8600 2550 50  0001 C CNN
F 3 "" H 8600 2550 50  0000 C CNN
	1    8600 2550
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR020
U 1 1 59B42F62
P 8400 2650
F 0 "#PWR020" H 8400 2400 50  0001 C CNN
F 1 "GND" H 8400 2500 50  0000 C CNN
F 2 "" H 8400 2650 50  0000 C CNN
F 3 "" H 8400 2650 50  0000 C CNN
	1    8400 2650
	1    0    0    -1  
$EndComp
$Comp
L C C3
U 1 1 59B43234
P 2800 1600
F 0 "C3" H 2825 1700 50  0000 L CNN
F 1 "100nF" H 2825 1500 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 2838 1450 50  0001 C CNN
F 3 "" H 2800 1600 50  0000 C CNN
	1    2800 1600
	0    -1   -1   0   
$EndComp
$Comp
L C C2
U 1 1 59B43285
P 2800 1300
F 0 "C2" H 2825 1400 50  0000 L CNN
F 1 "10uF" H 2825 1200 50  0000 L CNN
F 2 "Capacitors_THT:CP_Radial_D4.0mm_P1.50mm" H 2838 1150 50  0001 C CNN
F 3 "" H 2800 1300 50  0000 C CNN
	1    2800 1300
	0    -1   -1   0   
$EndComp
$Comp
L GND #PWR021
U 1 1 59B4367A
P 2650 1650
F 0 "#PWR021" H 2650 1400 50  0001 C CNN
F 1 "GND" H 2650 1500 50  0000 C CNN
F 2 "" H 2650 1650 50  0000 C CNN
F 3 "" H 2650 1650 50  0000 C CNN
	1    2650 1650
	1    0    0    -1  
$EndComp
$Comp
L CONN_01X02 P1
U 1 1 59B436EA
P 1250 800
F 0 "P1" H 1250 950 50  0000 C CNN
F 1 "Alim" V 1350 800 50  0000 C CNN
F 2 "Connectors_Molex:Molex_KK-41791-02_02x3.96mm_Straight" H 1250 800 50  0001 C CNN
F 3 "" H 1250 800 50  0000 C CNN
	1    1250 800 
	0    -1   -1   0   
$EndComp
$Comp
L GND #PWR022
U 1 1 59B4387A
P 1200 1050
F 0 "#PWR022" H 1200 800 50  0001 C CNN
F 1 "GND" H 1200 900 50  0000 C CNN
F 2 "" H 1200 1050 50  0000 C CNN
F 3 "" H 1200 1050 50  0000 C CNN
	1    1200 1050
	1    0    0    -1  
$EndComp
$Comp
L D D1
U 1 1 59B43EED
P 1550 1000
F 0 "D1" H 1550 1100 50  0000 C CNN
F 1 "D" H 1550 900 50  0000 C CNN
F 2 "Diodes_THT:D_DO-41_SOD81_P7.62mm_Horizontal" H 1550 1000 50  0001 C CNN
F 3 "" H 1550 1000 50  0000 C CNN
	1    1550 1000
	-1   0    0    1   
$EndComp
$Comp
L GND #PWR023
U 1 1 59B44BCD
P 2400 1250
F 0 "#PWR023" H 2400 1000 50  0001 C CNN
F 1 "GND" H 2400 1100 50  0000 C CNN
F 2 "" H 2400 1250 50  0000 C CNN
F 3 "" H 2400 1250 50  0000 C CNN
	1    2400 1250
	-1   0    0    1   
$EndComp
$Comp
L C C1
U 1 1 59B45B90
P 1750 1150
F 0 "C1" H 1775 1250 50  0000 L CNN
F 1 "10uF" H 1775 1050 50  0000 L CNN
F 2 "Capacitors_THT:CP_Radial_D4.0mm_P1.50mm" H 1788 1000 50  0001 C CNN
F 3 "" H 1750 1150 50  0000 C CNN
	1    1750 1150
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR024
U 1 1 59B45CB1
P 1750 1350
F 0 "#PWR024" H 1750 1100 50  0001 C CNN
F 1 "GND" H 1750 1200 50  0000 C CNN
F 2 "" H 1750 1350 50  0000 C CNN
F 3 "" H 1750 1350 50  0000 C CNN
	1    1750 1350
	1    0    0    -1  
$EndComp
$Comp
L R R2
U 1 1 59B46D0B
P 2050 3450
F 0 "R2" V 2130 3450 50  0000 C CNN
F 1 "10k" V 2050 3450 50  0000 C CNN
F 2 "Resistors_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 1980 3450 50  0001 C CNN
F 3 "" H 2050 3450 50  0000 C CNN
	1    2050 3450
	0    1    1    0   
$EndComp
$Comp
L R R5
U 1 1 59B46D5E
P 2050 3600
F 0 "R5" V 2130 3600 50  0000 C CNN
F 1 "10k" V 2050 3600 50  0000 C CNN
F 2 "Resistors_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 1980 3600 50  0001 C CNN
F 3 "" H 2050 3600 50  0000 C CNN
	1    2050 3600
	0    1    1    0   
$EndComp
$Comp
L VCC #PWR025
U 1 1 59B47163
P 1750 950
F 0 "#PWR025" H 1750 800 50  0001 C CNN
F 1 "VCC" H 1750 1100 50  0000 C CNN
F 2 "" H 1750 950 50  0000 C CNN
F 3 "" H 1750 950 50  0000 C CNN
	1    1750 950 
	1    0    0    -1  
$EndComp
$Comp
L VCC #PWR026
U 1 1 59B47286
P 1850 3450
F 0 "#PWR026" H 1850 3300 50  0001 C CNN
F 1 "VCC" H 1850 3600 50  0000 C CNN
F 2 "" H 1850 3450 50  0000 C CNN
F 3 "" H 1850 3450 50  0000 C CNN
	1    1850 3450
	0    -1   -1   0   
$EndComp
$Comp
L GND #PWR027
U 1 1 59B47402
P 1850 3600
F 0 "#PWR027" H 1850 3350 50  0001 C CNN
F 1 "GND" H 1850 3450 50  0000 C CNN
F 2 "" H 1850 3600 50  0000 C CNN
F 3 "" H 1850 3600 50  0000 C CNN
	1    1850 3600
	0    1    1    0   
$EndComp
$Comp
L GND #PWR028
U 1 1 59B48ED7
P 6000 3850
F 0 "#PWR028" H 6000 3600 50  0001 C CNN
F 1 "GND" H 6000 3700 50  0000 C CNN
F 2 "" H 6000 3850 50  0000 C CNN
F 3 "" H 6000 3850 50  0000 C CNN
	1    6000 3850
	1    0    0    -1  
$EndComp
$Comp
L C C13
U 1 1 59B48EDD
P 6000 3650
F 0 "C13" H 6025 3750 50  0000 L CNN
F 1 "1nF" H 6025 3550 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 6038 3500 50  0001 C CNN
F 3 "" H 6000 3650 50  0000 C CNN
	1    6000 3650
	1    0    0    -1  
$EndComp
$Comp
L CONN_01X08 P3
U 1 1 59B49A37
P 4100 2450
F 0 "P3" H 4100 2900 50  0000 C CNN
F 1 "CONN_01X08" V 4200 2450 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x08_Pitch2.54mm" H 4100 2450 50  0001 C CNN
F 3 "" H 4100 2450 50  0000 C CNN
	1    4100 2450
	1    0    0    -1  
$EndComp
$Comp
L +5V #PWR029
U 1 1 59B4A919
P 3550 1500
F 0 "#PWR029" H 3550 1350 50  0001 C CNN
F 1 "+5V" H 3550 1640 50  0000 C CNN
F 2 "" H 3550 1500 50  0000 C CNN
F 3 "" H 3550 1500 50  0000 C CNN
	1    3550 1500
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR030
U 1 1 59B4A93D
P 3850 2050
F 0 "#PWR030" H 3850 1800 50  0001 C CNN
F 1 "GND" H 3850 1900 50  0000 C CNN
F 2 "" H 3850 2050 50  0000 C CNN
F 3 "" H 3850 2050 50  0000 C CNN
	1    3850 2050
	-1   0    0    1   
$EndComp
$Comp
L C C4
U 1 1 59B4B07D
P 3550 1750
F 0 "C4" H 3575 1850 50  0000 L CNN
F 1 "100nF" H 3450 1650 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 3588 1600 50  0001 C CNN
F 3 "" H 3550 1750 50  0000 C CNN
	1    3550 1750
	-1   0    0    -1  
$EndComp
$Comp
L GND #PWR031
U 1 1 59B4B1CB
P 3550 2000
F 0 "#PWR031" H 3550 1750 50  0001 C CNN
F 1 "GND" H 3550 1850 50  0000 C CNN
F 2 "" H 3550 2000 50  0000 C CNN
F 3 "" H 3550 2000 50  0000 C CNN
	1    3550 2000
	1    0    0    -1  
$EndComp
$Comp
L CONN_01X03 P6
U 1 1 59B4BFDE
P 4250 3700
F 0 "P6" H 4250 3900 50  0000 C CNN
F 1 "CONN_01X03" V 4350 3700 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x03_Pitch2.54mm" H 4250 3700 50  0001 C CNN
F 3 "" H 4250 3700 50  0000 C CNN
	1    4250 3700
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR032
U 1 1 59B4C15E
P 4050 3850
F 0 "#PWR032" H 4050 3600 50  0001 C CNN
F 1 "GND" H 4050 3700 50  0000 C CNN
F 2 "" H 4050 3850 50  0000 C CNN
F 3 "" H 4050 3850 50  0000 C CNN
	1    4050 3850
	1    0    0    -1  
$EndComp
$Comp
L CONN_01X03 P8
U 1 1 59B4C424
P 1500 2100
F 0 "P8" H 1500 2300 50  0000 C CNN
F 1 "CONN_01X03" V 1600 2100 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x03_Pitch2.54mm" H 1500 2100 50  0001 C CNN
F 3 "" H 1500 2100 50  0000 C CNN
	1    1500 2100
	0    -1   -1   0   
$EndComp
$Comp
L D_Zener D3
U 1 1 59B4C81D
P 1900 2500
F 0 "D3" H 1900 2600 50  0000 C CNN
F 1 "D_Zener" H 1900 2400 50  0000 C CNN
F 2 "Resistors_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" H 1900 2500 50  0001 C CNN
F 3 "" H 1900 2500 50  0000 C CNN
	1    1900 2500
	0    -1   -1   0   
$EndComp
$Comp
L GND #PWR033
U 1 1 59B4C931
P 1900 2300
F 0 "#PWR033" H 1900 2050 50  0001 C CNN
F 1 "GND" H 1900 2150 50  0000 C CNN
F 2 "" H 1900 2300 50  0000 C CNN
F 3 "" H 1900 2300 50  0000 C CNN
	1    1900 2300
	-1   0    0    1   
$EndComp
$Comp
L GND #PWR034
U 1 1 59B4D579
P 1000 2850
F 0 "#PWR034" H 1000 2600 50  0001 C CNN
F 1 "GND" H 1000 2700 50  0000 C CNN
F 2 "" H 1000 2850 50  0000 C CNN
F 3 "" H 1000 2850 50  0000 C CNN
	1    1000 2850
	0    1    1    0   
$EndComp
$Comp
L D_Zener D2
U 1 1 59B4C6C1
P 1200 2850
F 0 "D2" H 1200 2950 50  0000 C CNN
F 1 "D_Zener" H 1200 2750 50  0000 C CNN
F 2 "Resistors_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" H 1200 2850 50  0001 C CNN
F 3 "" H 1200 2850 50  0000 C CNN
	1    1200 2850
	-1   0    0    1   
$EndComp
$Comp
L GND #PWR035
U 1 1 59B4D727
P 1300 2300
F 0 "#PWR035" H 1300 2050 50  0001 C CNN
F 1 "GND" H 1300 2150 50  0000 C CNN
F 2 "" H 1300 2300 50  0000 C CNN
F 3 "" H 1300 2300 50  0000 C CNN
	1    1300 2300
	0    1    1    0   
$EndComp
$Comp
L R R8
U 1 1 59B4E7EA
P 1600 2550
F 0 "R8" V 1680 2550 50  0000 C CNN
F 1 "R" V 1600 2550 50  0000 C CNN
F 2 "Resistors_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 1530 2550 50  0001 C CNN
F 3 "" H 1600 2550 50  0000 C CNN
	1    1600 2550
	1    0    0    -1  
$EndComp
$Comp
L R R7
U 1 1 59B4E917
P 1450 2550
F 0 "R7" V 1530 2550 50  0000 C CNN
F 1 "R" V 1450 2550 50  0000 C CNN
F 2 "Resistors_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 1380 2550 50  0001 C CNN
F 3 "" H 1450 2550 50  0000 C CNN
	1    1450 2550
	1    0    0    -1  
$EndComp
$Comp
L Conn_01x03 J1
U 1 1 59B43CC3
P 2300 1550
F 0 "J1" H 2300 1750 50  0000 C CNN
F 1 "Conn_01x03" H 2300 1350 50  0000 C CNN
F 2 "TO_SOT_Packages_THT:TO-220_Horizontal" H 2300 1550 50  0001 C CNN
F 3 "" H 2300 1550 50  0001 C CNN
	1    2300 1550
	0    1    1    0   
$EndComp
$Comp
L arduino_mini U1
U 1 1 59B407FD
P 2950 2950
F 0 "U1" H 3450 2000 70  0000 C CNN
F 1 "arduino_mini" H 3700 1900 70  0000 C CNN
F 2 "arduino:arduino_mini" H 2950 2900 60  0000 C CNN
F 3 "" H 2950 2950 60  0001 C CNN
	1    2950 2950
	1    0    0    -1  
$EndComp
Connection ~ 5700 3500
Connection ~ 5400 3500
Connection ~ 5050 3500
Wire Wire Line
	5700 3800 5700 3850
Wire Wire Line
	5050 3800 5050 3850
Wire Wire Line
	5400 3800 5400 3850
Wire Wire Line
	5950 2150 5950 2200
Wire Wire Line
	5300 3000 5250 3000
Wire Wire Line
	5300 2900 5250 2900
Connection ~ 4750 2800
Wire Wire Line
	4650 2800 5300 2800
Wire Wire Line
	5300 2150 5300 2200
Wire Wire Line
	4000 3150 4050 3150
Wire Wire Line
	3700 3150 3700 3300
Wire Wire Line
	3650 3250 3700 3250
Wire Wire Line
	3650 3550 3700 3550
Wire Wire Line
	3650 3000 3700 3000
Wire Wire Line
	3650 2900 3700 2900
Wire Wire Line
	6650 3900 6700 3900
Wire Wire Line
	6500 2250 6550 2250
Wire Wire Line
	6500 2150 6550 2150
Wire Wire Line
	6500 2050 6550 2050
Wire Wire Line
	6650 3800 6700 3800
Wire Wire Line
	6650 3700 6700 3700
Wire Wire Line
	2150 3250 2250 3250
Wire Wire Line
	2150 3150 2250 3150
Wire Wire Line
	4950 2700 5300 2700
Wire Wire Line
	5650 2150 5650 2200
Wire Wire Line
	6250 1950 6550 1950
Wire Wire Line
	4600 1800 4600 1850
Wire Wire Line
	6700 3600 6300 3600
Wire Wire Line
	4750 3050 4750 3100
Wire Wire Line
	4550 3450 4550 3500
Wire Wire Line
	2950 4500 2950 4650
Wire Wire Line
	4900 1850 6550 1850
Connection ~ 5300 1850
Connection ~ 5650 1850
Connection ~ 5950 1850
Wire Wire Line
	5000 2150 5000 2200
Connection ~ 3700 3250
Wire Wire Line
	7200 2800 7200 2850
Wire Wire Line
	8050 2800 8050 2850
Wire Wire Line
	7050 2500 7300 2500
Connection ~ 7200 2500
Wire Wire Line
	7600 2500 7650 2500
Wire Wire Line
	7800 2350 7950 2350
Wire Wire Line
	7950 2350 7950 2500
Wire Wire Line
	7950 2500 8400 2500
Connection ~ 8050 2500
Wire Wire Line
	8400 2600 8400 2650
Wire Wire Line
	2950 1000 2950 1800
Connection ~ 2950 1600
Wire Wire Line
	2650 1300 2650 1650
Connection ~ 2650 1600
Connection ~ 2950 1300
Wire Wire Line
	1200 1000 1200 1050
Wire Wire Line
	1300 1000 1400 1000
Wire Wire Line
	2300 1000 2950 1000
Wire Wire Line
	2400 1250 2400 1350
Wire Wire Line
	1700 1000 2200 1000
Connection ~ 1750 1000
Wire Wire Line
	1750 1300 1750 1350
Wire Wire Line
	1750 950  1750 1000
Wire Wire Line
	6650 4100 6700 4100
Connection ~ 5000 1850
Connection ~ 6000 3500
Wire Wire Line
	6000 3800 6000 3850
Wire Wire Line
	3900 2100 3850 2100
Wire Wire Line
	3750 2200 3900 2200
Wire Wire Line
	3550 1500 3550 1600
Wire Wire Line
	3550 1900 3550 2000
Wire Wire Line
	3550 1600 3750 1600
Wire Wire Line
	3750 1600 3750 2200
Wire Wire Line
	3850 2050 3850 2300
Wire Wire Line
	3650 3350 3650 3400
Wire Wire Line
	3650 3400 4050 3400
Wire Wire Line
	4050 3400 4050 3600
Wire Wire Line
	3650 3450 4000 3450
Wire Wire Line
	4000 3450 4000 3700
Wire Wire Line
	4000 3700 4050 3700
Wire Wire Line
	4050 3800 4050 3850
Wire Wire Line
	3850 2300 3900 2300
Connection ~ 3850 2100
Wire Wire Line
	2200 1000 2200 1350
Wire Wire Line
	2300 1350 2300 1000
Wire Wire Line
	4850 3500 6700 3500
Wire Wire Line
	2250 3450 2200 3450
Wire Wire Line
	2200 3450 2200 3600
Wire Wire Line
	1900 3450 1850 3450
Wire Wire Line
	1900 3600 1850 3600
Wire Wire Line
	1400 2300 1300 2300
Wire Wire Line
	1500 2300 1500 2400
Wire Wire Line
	1500 2400 1450 2400
Wire Wire Line
	1600 2300 1600 2400
Wire Wire Line
	1350 2850 2250 2850
Wire Wire Line
	1450 2850 1450 2700
Wire Wire Line
	1600 2750 2250 2750
Wire Wire Line
	1600 2750 1600 2700
Connection ~ 1450 2850
Wire Wire Line
	1000 2850 1050 2850
Wire Wire Line
	1900 2650 1900 2750
Connection ~ 1900 2750
Wire Wire Line
	1900 2300 1900 2350
Wire Wire Line
	3650 2800 3750 2800
Wire Wire Line
	3750 2800 3750 2400
Wire Wire Line
	3750 2400 3900 2400
Wire Wire Line
	3650 2700 3800 2700
Wire Wire Line
	3800 2700 3800 2500
Wire Wire Line
	3800 2500 3900 2500
Wire Wire Line
	3650 2600 3900 2600
Wire Wire Line
	3900 2700 3850 2700
Wire Wire Line
	3850 2700 3850 2550
Wire Wire Line
	3850 2550 3650 2550
Wire Wire Line
	3650 2550 3650 2500
Wire Wire Line
	3900 2800 3700 2800
Wire Wire Line
	3700 2800 3700 2400
Wire Wire Line
	3700 2400 3650 2400
$Comp
L Conn_01x01 J5
U 1 1 59C8D347
P 3700 5700
F 0 "J5" H 3700 5800 50  0000 C CNN
F 1 "Conn_01x01" H 3700 5600 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x01_Pitch2.54mm" H 3700 5700 50  0001 C CNN
F 3 "" H 3700 5700 50  0001 C CNN
	1    3700 5700
	1    0    0    -1  
$EndComp
$Comp
L Conn_01x01 J3
U 1 1 59C8D4C8
P 3700 5450
F 0 "J3" H 3700 5550 50  0000 C CNN
F 1 "Conn_01x01" H 3700 5350 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x01_Pitch2.54mm" H 3700 5450 50  0001 C CNN
F 3 "" H 3700 5450 50  0001 C CNN
	1    3700 5450
	1    0    0    -1  
$EndComp
$Comp
L Conn_01x01 J7
U 1 1 59C8D535
P 3700 5950
F 0 "J7" H 3700 6050 50  0000 C CNN
F 1 "Conn_01x01" H 3700 5850 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x01_Pitch2.54mm" H 3700 5950 50  0001 C CNN
F 3 "" H 3700 5950 50  0001 C CNN
	1    3700 5950
	1    0    0    -1  
$EndComp
$Comp
L Conn_01x01 J4
U 1 1 59C8D625
P 2750 5700
F 0 "J4" H 2750 5800 50  0000 C CNN
F 1 "Conn_01x01" H 2750 5600 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x01_Pitch2.54mm" H 2750 5700 50  0001 C CNN
F 3 "" H 2750 5700 50  0001 C CNN
	1    2750 5700
	-1   0    0    -1  
$EndComp
$Comp
L Conn_01x01 J2
U 1 1 59C8D62B
P 2750 5450
F 0 "J2" H 2750 5550 50  0000 C CNN
F 1 "Conn_01x01" H 2750 5350 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x01_Pitch2.54mm" H 2750 5450 50  0001 C CNN
F 3 "" H 2750 5450 50  0001 C CNN
	1    2750 5450
	-1   0    0    -1  
$EndComp
$Comp
L Conn_01x01 J6
U 1 1 59C8D631
P 2750 5950
F 0 "J6" H 2750 6050 50  0000 C CNN
F 1 "Conn_01x01" H 2750 5850 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x01_Pitch2.54mm" H 2750 5950 50  0001 C CNN
F 3 "" H 2750 5950 50  0001 C CNN
	1    2750 5950
	-1   0    0    -1  
$EndComp
$Comp
L GND #PWR036
U 1 1 59C8D7C5
P 3000 5450
F 0 "#PWR036" H 3000 5200 50  0001 C CNN
F 1 "GND" H 3000 5300 50  0000 C CNN
F 2 "" H 3000 5450 50  0001 C CNN
F 3 "" H 3000 5450 50  0001 C CNN
	1    3000 5450
	0    -1   -1   0   
$EndComp
$Comp
L GND #PWR037
U 1 1 59C8D858
P 3000 5700
F 0 "#PWR037" H 3000 5450 50  0001 C CNN
F 1 "GND" H 3000 5550 50  0000 C CNN
F 2 "" H 3000 5700 50  0001 C CNN
F 3 "" H 3000 5700 50  0001 C CNN
	1    3000 5700
	0    -1   -1   0   
$EndComp
$Comp
L GND #PWR038
U 1 1 59C8D8E4
P 3000 5950
F 0 "#PWR038" H 3000 5700 50  0001 C CNN
F 1 "GND" H 3000 5800 50  0000 C CNN
F 2 "" H 3000 5950 50  0001 C CNN
F 3 "" H 3000 5950 50  0001 C CNN
	1    3000 5950
	0    -1   -1   0   
$EndComp
$Comp
L GND #PWR039
U 1 1 59C8D9B9
P 3450 5450
F 0 "#PWR039" H 3450 5200 50  0001 C CNN
F 1 "GND" H 3450 5300 50  0000 C CNN
F 2 "" H 3450 5450 50  0001 C CNN
F 3 "" H 3450 5450 50  0001 C CNN
	1    3450 5450
	0    1    1    0   
$EndComp
$Comp
L GND #PWR040
U 1 1 59C8DA45
P 3450 5700
F 0 "#PWR040" H 3450 5450 50  0001 C CNN
F 1 "GND" H 3450 5550 50  0000 C CNN
F 2 "" H 3450 5700 50  0001 C CNN
F 3 "" H 3450 5700 50  0001 C CNN
	1    3450 5700
	0    1    1    0   
$EndComp
$Comp
L GND #PWR041
U 1 1 59C8DAD1
P 3450 5950
F 0 "#PWR041" H 3450 5700 50  0001 C CNN
F 1 "GND" H 3450 5800 50  0000 C CNN
F 2 "" H 3450 5950 50  0001 C CNN
F 3 "" H 3450 5950 50  0001 C CNN
	1    3450 5950
	0    1    1    0   
$EndComp
$Comp
L Conn_01x01 J11
U 1 1 59C8DC30
P 3700 6500
F 0 "J11" H 3700 6600 50  0000 C CNN
F 1 "Conn_01x01" H 3700 6400 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x01_Pitch2.54mm" H 3700 6500 50  0001 C CNN
F 3 "" H 3700 6500 50  0001 C CNN
	1    3700 6500
	1    0    0    -1  
$EndComp
$Comp
L Conn_01x01 J9
U 1 1 59C8DC36
P 3700 6250
F 0 "J9" H 3700 6350 50  0000 C CNN
F 1 "Conn_01x01" H 3700 6150 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x01_Pitch2.54mm" H 3700 6250 50  0001 C CNN
F 3 "" H 3700 6250 50  0001 C CNN
	1    3700 6250
	1    0    0    -1  
$EndComp
$Comp
L Conn_01x01 J13
U 1 1 59C8DC3C
P 3700 6750
F 0 "J13" H 3700 6850 50  0000 C CNN
F 1 "Conn_01x01" H 3700 6650 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x01_Pitch2.54mm" H 3700 6750 50  0001 C CNN
F 3 "" H 3700 6750 50  0001 C CNN
	1    3700 6750
	1    0    0    -1  
$EndComp
$Comp
L Conn_01x01 J10
U 1 1 59C8DC42
P 2750 6500
F 0 "J10" H 2750 6600 50  0000 C CNN
F 1 "Conn_01x01" H 2750 6400 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x01_Pitch2.54mm" H 2750 6500 50  0001 C CNN
F 3 "" H 2750 6500 50  0001 C CNN
	1    2750 6500
	-1   0    0    -1  
$EndComp
$Comp
L Conn_01x01 J8
U 1 1 59C8DC48
P 2750 6250
F 0 "J8" H 2750 6350 50  0000 C CNN
F 1 "Conn_01x01" H 2750 6150 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x01_Pitch2.54mm" H 2750 6250 50  0001 C CNN
F 3 "" H 2750 6250 50  0001 C CNN
	1    2750 6250
	-1   0    0    -1  
$EndComp
$Comp
L Conn_01x01 J12
U 1 1 59C8DC4E
P 2750 6750
F 0 "J12" H 2750 6850 50  0000 C CNN
F 1 "Conn_01x01" H 2750 6650 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x01_Pitch2.54mm" H 2750 6750 50  0001 C CNN
F 3 "" H 2750 6750 50  0001 C CNN
	1    2750 6750
	-1   0    0    -1  
$EndComp
$Comp
L GND #PWR042
U 1 1 59C8DC54
P 3000 6250
F 0 "#PWR042" H 3000 6000 50  0001 C CNN
F 1 "GND" H 3000 6100 50  0000 C CNN
F 2 "" H 3000 6250 50  0001 C CNN
F 3 "" H 3000 6250 50  0001 C CNN
	1    3000 6250
	0    -1   -1   0   
$EndComp
$Comp
L GND #PWR043
U 1 1 59C8DC5A
P 3000 6500
F 0 "#PWR043" H 3000 6250 50  0001 C CNN
F 1 "GND" H 3000 6350 50  0000 C CNN
F 2 "" H 3000 6500 50  0001 C CNN
F 3 "" H 3000 6500 50  0001 C CNN
	1    3000 6500
	0    -1   -1   0   
$EndComp
$Comp
L GND #PWR044
U 1 1 59C8DC60
P 3000 6750
F 0 "#PWR044" H 3000 6500 50  0001 C CNN
F 1 "GND" H 3000 6600 50  0000 C CNN
F 2 "" H 3000 6750 50  0001 C CNN
F 3 "" H 3000 6750 50  0001 C CNN
	1    3000 6750
	0    -1   -1   0   
$EndComp
$Comp
L GND #PWR045
U 1 1 59C8DC66
P 3450 6250
F 0 "#PWR045" H 3450 6000 50  0001 C CNN
F 1 "GND" H 3450 6100 50  0000 C CNN
F 2 "" H 3450 6250 50  0001 C CNN
F 3 "" H 3450 6250 50  0001 C CNN
	1    3450 6250
	0    1    1    0   
$EndComp
$Comp
L GND #PWR046
U 1 1 59C8DC6C
P 3450 6500
F 0 "#PWR046" H 3450 6250 50  0001 C CNN
F 1 "GND" H 3450 6350 50  0000 C CNN
F 2 "" H 3450 6500 50  0001 C CNN
F 3 "" H 3450 6500 50  0001 C CNN
	1    3450 6500
	0    1    1    0   
$EndComp
$Comp
L GND #PWR047
U 1 1 59C8DC72
P 3450 6750
F 0 "#PWR047" H 3450 6500 50  0001 C CNN
F 1 "GND" H 3450 6600 50  0000 C CNN
F 2 "" H 3450 6750 50  0001 C CNN
F 3 "" H 3450 6750 50  0001 C CNN
	1    3450 6750
	0    1    1    0   
$EndComp
Wire Wire Line
	2950 5450 3000 5450
Wire Wire Line
	2950 5700 3000 5700
Wire Wire Line
	2950 5950 3000 5950
Wire Wire Line
	2950 6250 3000 6250
Wire Wire Line
	2950 6500 3000 6500
Wire Wire Line
	2950 6750 3000 6750
Wire Wire Line
	3450 5450 3500 5450
Wire Wire Line
	3450 5700 3500 5700
Wire Wire Line
	3450 5950 3500 5950
Wire Wire Line
	3450 6250 3500 6250
Wire Wire Line
	3450 6500 3500 6500
Wire Wire Line
	3450 6750 3500 6750
$Comp
L Conn_01x01 J15
U 1 1 59C8EC62
P 3700 7050
F 0 "J15" H 3700 7150 50  0000 C CNN
F 1 "Conn_01x01" H 3700 6950 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x01_Pitch2.54mm" H 3700 7050 50  0001 C CNN
F 3 "" H 3700 7050 50  0001 C CNN
	1    3700 7050
	1    0    0    -1  
$EndComp
$Comp
L Conn_01x01 J17
U 1 1 59C8EC68
P 3700 7300
F 0 "J17" H 3700 7400 50  0000 C CNN
F 1 "Conn_01x01" H 3700 7200 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x01_Pitch2.54mm" H 3700 7300 50  0001 C CNN
F 3 "" H 3700 7300 50  0001 C CNN
	1    3700 7300
	1    0    0    -1  
$EndComp
$Comp
L Conn_01x01 J14
U 1 1 59C8EC6E
P 2750 7050
F 0 "J14" H 2750 7150 50  0000 C CNN
F 1 "Conn_01x01" H 2750 6950 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x01_Pitch2.54mm" H 2750 7050 50  0001 C CNN
F 3 "" H 2750 7050 50  0001 C CNN
	1    2750 7050
	-1   0    0    -1  
$EndComp
$Comp
L Conn_01x01 J16
U 1 1 59C8EC74
P 2750 7300
F 0 "J16" H 2750 7400 50  0000 C CNN
F 1 "Conn_01x01" H 2750 7200 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x01_Pitch2.54mm" H 2750 7300 50  0001 C CNN
F 3 "" H 2750 7300 50  0001 C CNN
	1    2750 7300
	-1   0    0    -1  
$EndComp
$Comp
L GND #PWR048
U 1 1 59C8EC7A
P 3000 7050
F 0 "#PWR048" H 3000 6800 50  0001 C CNN
F 1 "GND" H 3000 6900 50  0000 C CNN
F 2 "" H 3000 7050 50  0001 C CNN
F 3 "" H 3000 7050 50  0001 C CNN
	1    3000 7050
	0    -1   -1   0   
$EndComp
$Comp
L GND #PWR049
U 1 1 59C8EC80
P 3000 7300
F 0 "#PWR049" H 3000 7050 50  0001 C CNN
F 1 "GND" H 3000 7150 50  0000 C CNN
F 2 "" H 3000 7300 50  0001 C CNN
F 3 "" H 3000 7300 50  0001 C CNN
	1    3000 7300
	0    -1   -1   0   
$EndComp
$Comp
L GND #PWR050
U 1 1 59C8EC86
P 3450 7050
F 0 "#PWR050" H 3450 6800 50  0001 C CNN
F 1 "GND" H 3450 6900 50  0000 C CNN
F 2 "" H 3450 7050 50  0001 C CNN
F 3 "" H 3450 7050 50  0001 C CNN
	1    3450 7050
	0    1    1    0   
$EndComp
$Comp
L GND #PWR051
U 1 1 59C8EC8C
P 3450 7300
F 0 "#PWR051" H 3450 7050 50  0001 C CNN
F 1 "GND" H 3450 7150 50  0000 C CNN
F 2 "" H 3450 7300 50  0001 C CNN
F 3 "" H 3450 7300 50  0001 C CNN
	1    3450 7300
	0    1    1    0   
$EndComp
Wire Wire Line
	2950 7050 3000 7050
Wire Wire Line
	2950 7300 3000 7300
Wire Wire Line
	3450 7050 3500 7050
Wire Wire Line
	3450 7300 3500 7300
$Comp
L Conn_01x01 J19
U 1 1 59C8F034
P 5100 5500
F 0 "J19" H 5100 5600 50  0000 C CNN
F 1 "Conn_01x01" H 5100 5400 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x01_Pitch2.54mm" H 5100 5500 50  0001 C CNN
F 3 "" H 5100 5500 50  0001 C CNN
	1    5100 5500
	1    0    0    -1  
$EndComp
$Comp
L Conn_01x01 J21
U 1 1 59C8F03A
P 5100 5750
F 0 "J21" H 5100 5850 50  0000 C CNN
F 1 "Conn_01x01" H 5100 5650 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x01_Pitch2.54mm" H 5100 5750 50  0001 C CNN
F 3 "" H 5100 5750 50  0001 C CNN
	1    5100 5750
	1    0    0    -1  
$EndComp
$Comp
L Conn_01x01 J18
U 1 1 59C8F040
P 4150 5500
F 0 "J18" H 4150 5600 50  0000 C CNN
F 1 "Conn_01x01" H 4150 5400 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x01_Pitch2.54mm" H 4150 5500 50  0001 C CNN
F 3 "" H 4150 5500 50  0001 C CNN
	1    4150 5500
	-1   0    0    -1  
$EndComp
$Comp
L Conn_01x01 J20
U 1 1 59C8F046
P 4150 5750
F 0 "J20" H 4150 5850 50  0000 C CNN
F 1 "Conn_01x01" H 4150 5650 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x01_Pitch2.54mm" H 4150 5750 50  0001 C CNN
F 3 "" H 4150 5750 50  0001 C CNN
	1    4150 5750
	-1   0    0    -1  
$EndComp
$Comp
L GND #PWR052
U 1 1 59C8F04C
P 4400 5500
F 0 "#PWR052" H 4400 5250 50  0001 C CNN
F 1 "GND" H 4400 5350 50  0000 C CNN
F 2 "" H 4400 5500 50  0001 C CNN
F 3 "" H 4400 5500 50  0001 C CNN
	1    4400 5500
	0    -1   -1   0   
$EndComp
$Comp
L GND #PWR053
U 1 1 59C8F052
P 4400 5750
F 0 "#PWR053" H 4400 5500 50  0001 C CNN
F 1 "GND" H 4400 5600 50  0000 C CNN
F 2 "" H 4400 5750 50  0001 C CNN
F 3 "" H 4400 5750 50  0001 C CNN
	1    4400 5750
	0    -1   -1   0   
$EndComp
$Comp
L GND #PWR054
U 1 1 59C8F058
P 4850 5500
F 0 "#PWR054" H 4850 5250 50  0001 C CNN
F 1 "GND" H 4850 5350 50  0000 C CNN
F 2 "" H 4850 5500 50  0001 C CNN
F 3 "" H 4850 5500 50  0001 C CNN
	1    4850 5500
	0    1    1    0   
$EndComp
$Comp
L GND #PWR055
U 1 1 59C8F05E
P 4850 5750
F 0 "#PWR055" H 4850 5500 50  0001 C CNN
F 1 "GND" H 4850 5600 50  0000 C CNN
F 2 "" H 4850 5750 50  0001 C CNN
F 3 "" H 4850 5750 50  0001 C CNN
	1    4850 5750
	0    1    1    0   
$EndComp
Wire Wire Line
	4350 5500 4400 5500
Wire Wire Line
	4350 5750 4400 5750
Wire Wire Line
	4850 5500 4900 5500
Wire Wire Line
	4850 5750 4900 5750
$Comp
L GND #PWR056
U 1 1 59C8F905
P 6200 4450
F 0 "#PWR056" H 6200 4200 50  0001 C CNN
F 1 "GND" H 6200 4300 50  0000 C CNN
F 2 "" H 6200 4450 50  0000 C CNN
F 3 "" H 6200 4450 50  0000 C CNN
	1    6200 4450
	1    0    0    -1  
$EndComp
$Comp
L R R9
U 1 1 59C8F90C
P 6200 4250
F 0 "R9" V 6280 4250 50  0000 C CNN
F 1 "1k" V 6200 4250 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 6130 4250 50  0001 C CNN
F 3 "" H 6200 4250 50  0000 C CNN
	1    6200 4250
	-1   0    0    1   
$EndComp
Wire Wire Line
	6200 4400 6200 4450
Wire Wire Line
	6700 4000 6200 4000
Wire Wire Line
	6200 4000 6200 4100
$Comp
L R R11
U 1 1 59CD4E49
P 5700 4800
F 0 "R11" V 5780 4800 50  0000 C CNN
F 1 "5k" V 5700 4800 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 5630 4800 50  0001 C CNN
F 3 "" H 5700 4800 50  0001 C CNN
	1    5700 4800
	1    0    0    1   
$EndComp
Text GLabel 5700 5000 3    60   Input ~ 0
SDA
Text GLabel 5500 5000 3    60   Input ~ 0
SCL
$Comp
L R R10
U 1 1 59CD524B
P 5500 4800
F 0 "R10" V 5580 4800 50  0000 C CNN
F 1 "5k" V 5500 4800 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 5430 4800 50  0001 C CNN
F 3 "" H 5500 4800 50  0001 C CNN
	1    5500 4800
	1    0    0    1   
$EndComp
Wire Wire Line
	5700 5000 5700 4950
Wire Wire Line
	5700 4650 5700 4600
Wire Wire Line
	5500 5000 5500 4950
Wire Wire Line
	5500 4650 5500 4600
$Comp
L +5V #PWR057
U 1 1 59CD66AD
P 5500 4600
F 0 "#PWR057" H 5500 4450 50  0001 C CNN
F 1 "+5V" H 5500 4740 50  0000 C CNN
F 2 "" H 5500 4600 50  0000 C CNN
F 3 "" H 5500 4600 50  0000 C CNN
	1    5500 4600
	1    0    0    -1  
$EndComp
$Comp
L +5V #PWR058
U 1 1 59CD68D2
P 5700 4600
F 0 "#PWR058" H 5700 4450 50  0001 C CNN
F 1 "+5V" H 5700 4740 50  0000 C CNN
F 2 "" H 5700 4600 50  0000 C CNN
F 3 "" H 5700 4600 50  0000 C CNN
	1    5700 4600
	1    0    0    -1  
$EndComp
$EndSCHEMATC
