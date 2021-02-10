EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title "Autonomous Buzzer JHE42B"
Date "2021-02-05"
Rev "4"
Comp "Reversed schematic"
Comment1 "Needs to be double checked"
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L Device:R_Small R3
U 1 1 6011476C
P 5050 4000
F 0 "R3" V 5246 4000 50  0000 C CNN
F 1 "992ohm" V 5155 4000 50  0000 C CNN
F 2 "" H 5050 4000 50  0001 C CNN
F 3 "~" H 5050 4000 50  0001 C CNN
	1    5050 4000
	0    1    -1   0   
$EndComp
$Comp
L Device:Buzzer BZ
U 1 1 60171F86
P 8550 3400
F 0 "BZ" H 8702 3429 50  0000 L CNN
F 1 "Buzzer" H 8702 3338 50  0000 L CNN
F 2 "" V 8525 3500 50  0001 C CNN
F 3 "~" V 8525 3500 50  0001 C CNN
	1    8550 3400
	1    0    0    -1  
$EndComp
Wire Wire Line
	5150 4000 5550 4000
Wire Wire Line
	4800 3000 6150 3000
$Comp
L Device:R_Small R2
U 1 1 601A0ACE
P 4700 3000
F 0 "R2" V 4896 3000 50  0000 C CNN
F 1 "9.94Kohm" V 4805 3000 50  0000 C CNN
F 2 "" H 4700 3000 50  0001 C CNN
F 3 "~" H 4700 3000 50  0001 C CNN
	1    4700 3000
	0    1    -1   0   
$EndComp
$Comp
L Device:LED BuzzerLight
U 1 1 6014D1A4
P 3850 4400
F 0 "BuzzerLight" H 3843 4617 50  0000 C CNN
F 1 "LED" H 3843 4526 50  0000 C CNN
F 2 "" H 3850 4400 50  0001 C CNN
F 3 "~" H 3850 4400 50  0001 C CNN
	1    3850 4400
	-1   0    0    1   
$EndComp
$Comp
L Device:R_Small 2.2Ohm
U 1 1 60150121
P 3600 4400
F 0 "2.2Ohm" V 3700 4400 50  0000 C CNN
F 1 "R_shunt 62R0" V 3800 4400 50  0000 C CNN
F 2 "" H 3600 4400 50  0001 C CNN
F 3 "~" H 3600 4400 50  0001 C CNN
	1    3600 4400
	0    1    -1   0   
$EndComp
$Comp
L Diode:1N5819 Diode1
U 1 1 60158A46
P 3350 4400
F 0 "Diode1" H 3350 4550 50  0000 C CNN
F 1 "||SL - 1N5819" H 3450 4650 50  0000 C CNN
F 2 "Diode_THT:D_DO-41_SOD81_P10.16mm_Horizontal" H 3350 4225 50  0001 C CNN
F 3 "http://www.vishay.com/docs/88525/1n5817.pdf" H 3350 4400 50  0001 C CNN
	1    3350 4400
	-1   0    0    1   
$EndComp
$Comp
L Regulator_Linear:XC6206PxxxMR VoltReg_662k
U 1 1 60147507
P 8850 4550
F 0 "VoltReg_662k" H 8850 4792 50  0000 C CNN
F 1 "XC6206PxxxMR 5vTo3.3v" H 8850 4700 50  0000 C CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 8850 4775 50  0001 C CIN
F 3 "https://www.torexsemi.com/file/xc6206/XC6206.pdf" H 8850 4550 50  0001 C CNN
	1    8850 4550
	1    0    0    -1  
$EndComp
$Comp
L Device:R_Small 2.2Ohm
U 1 1 60128808
P 7700 3600
F 0 "2.2Ohm" V 7896 3600 50  0000 C CNN
F 1 "R_shunt" V 7805 3600 50  0000 C CNN
F 2 "" H 7700 3600 50  0001 C CNN
F 3 "~" H 7700 3600 50  0001 C CNN
	1    7700 3600
	1    0    0    1   
$EndComp
$Comp
L Transistor_BJT:MMBT3904 1AM
U 1 1 601218E6
P 4200 4400
F 0 "1AM" H 4550 4350 50  0000 C CNN
F 1 "MMBT3904" H 4600 4500 50  0000 C CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 4400 4325 50  0001 L CIN
F 3 "https://www.onsemi.com/pub/Collateral/2N3903-D.PDF" H 4200 4400 50  0001 L CNN
	1    4200 4400
	1    0    0    1   
$EndComp
$Comp
L power:GND #PWR?
U 1 1 601E26E8
P 4300 4600
F 0 "#PWR?" H 4300 4350 50  0001 C CNN
F 1 "GND" H 4305 4427 50  0000 C CNN
F 2 "" H 4300 4600 50  0001 C CNN
F 3 "" H 4300 4600 50  0001 C CNN
	1    4300 4600
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 6011EC22
P 6150 5000
F 0 "#PWR?" H 6150 4750 50  0001 C CNN
F 1 "GND" H 6155 4827 50  0000 C CNN
F 2 "" H 6150 5000 50  0001 C CNN
F 3 "" H 6150 5000 50  0001 C CNN
	1    6150 5000
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 601F37C2
P 8850 4850
F 0 "#PWR?" H 8850 4600 50  0001 C CNN
F 1 "GND" H 8855 4677 50  0000 C CNN
F 2 "" H 8850 4850 50  0001 C CNN
F 3 "" H 8850 4850 50  0001 C CNN
	1    8850 4850
	1    0    0    -1  
$EndComp
$Comp
L MCU_ST_STM8:STM8S003F3P Controller
U 1 1 6010B02B
P 6150 4000
F 0 "Controller" H 6150 5181 50  0000 C CNN
F 1 "STM8S003F3P" H 6150 5090 50  0000 C CNN
F 2 "Package_SO:TSSOP-20_4.4x6.5mm_P0.65mm" H 6200 5100 50  0001 L CNN
F 3 "http://www.st.com/st-web-ui/static/active/en/resource/technical/document/datasheet/DM00024550.pdf" H 6100 3600 50  0001 C CNN
	1    6150 4000
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 601FD995
P 7700 2900
F 0 "#PWR?" H 7700 2650 50  0001 C CNN
F 1 "GND" H 7705 2727 50  0000 C CNN
F 2 "" H 7700 2900 50  0001 C CNN
F 3 "" H 7700 2900 50  0001 C CNN
	1    7700 2900
	-1   0    0    1   
$EndComp
$Comp
L power:GND #PWR?
U 1 1 601FE4D8
P 7900 4600
F 0 "#PWR?" H 7900 4350 50  0001 C CNN
F 1 "GND" H 7905 4427 50  0000 C CNN
F 2 "" H 7900 4600 50  0001 C CNN
F 3 "" H 7900 4600 50  0001 C CNN
	1    7900 4600
	1    0    0    -1  
$EndComp
$Comp
L Device:R_Small R1
U 1 1 6017D0ED
P 7650 4600
F 0 "R1" V 7846 4600 50  0000 C CNN
F 1 "9.94Kohm" V 7755 4600 50  0000 C CNN
F 2 "" H 7650 4600 50  0001 C CNN
F 3 "~" H 7650 4600 50  0001 C CNN
	1    7650 4600
	0    1    -1   0   
$EndComp
Wire Wire Line
	6750 4000 7400 4000
Wire Wire Line
	7550 4600 7400 4600
Wire Wire Line
	7400 4600 7400 4000
Connection ~ 7400 4000
Wire Wire Line
	7400 4000 7500 4000
Wire Wire Line
	7750 4600 7900 4600
Wire Wire Line
	7900 4600 7900 4000
Connection ~ 7900 4600
$Comp
L Transistor_FET:BSN20 Q
U 1 1 6018635A
P 7700 3900
F 0 "Q" V 7700 3650 50  0000 L CNN
F 1 "BSN20" V 7600 3600 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 7900 3825 50  0001 L CIN
F 3 "http://www.diodes.com/assets/Datasheets/ds31898.pdf" H 7700 3900 50  0001 L CNN
	1    7700 3900
	0    1    1    0   
$EndComp
$Comp
L Device:Battery_Cell LIPO_1S
U 1 1 6011DE40
P 7700 3000
F 0 "LIPO_1S" H 7582 3004 50  0000 R CNN
F 1 "Battery_Cell" H 7582 3095 50  0000 R CNN
F 2 "" V 7700 3060 50  0001 C CNN
F 3 "~" V 7700 3060 50  0001 C CNN
	1    7700 3000
	-1   0    0    1   
$EndComp
Wire Wire Line
	7700 3200 7700 3300
Wire Wire Line
	7700 3300 8450 3300
Connection ~ 7700 3300
Wire Wire Line
	7700 3300 7700 3500
Wire Wire Line
	4600 3000 2250 3000
$Comp
L power:GND #PWR?
U 1 1 602456CD
P 2250 3200
F 0 "#PWR?" H 2250 2950 50  0001 C CNN
F 1 "GND" H 2250 3000 50  0000 C CNN
F 2 "" H 2250 3200 50  0001 C CNN
F 3 "" H 2250 3200 50  0001 C CNN
	1    2250 3200
	1    0    0    -1  
$EndComp
$Comp
L Device:LED PowerOn
U 1 1 6024A005
P 2650 3400
F 0 "PowerOn" H 2700 3650 50  0000 C CNN
F 1 "LED" H 2700 3550 50  0000 C CNN
F 2 "" H 2650 3400 50  0001 C CNN
F 3 "~" H 2650 3400 50  0001 C CNN
	1    2650 3400
	-1   0    0    1   
$EndComp
$Comp
L Device:R_Small 980Ohm
U 1 1 6024A00B
P 2900 3400
F 0 "980Ohm" V 2700 3450 50  0000 C CNN
F 1 "R5" V 2800 3400 50  0000 C CNN
F 2 "" H 2900 3400 50  0001 C CNN
F 3 "~" H 2900 3400 50  0001 C CNN
	1    2900 3400
	0    1    -1   0   
$EndComp
$Comp
L Connector_Generic:Conn_01x03 Remote
U 1 1 6023EF45
P 2050 3100
F 0 "Remote" H 2450 3450 50  0000 R CNN
F 1 "Header" H 2450 3350 50  0000 R CNN
F 2 "" H 2050 3100 50  0001 C CNN
F 3 "~" H 2050 3100 50  0001 C CNN
	1    2050 3100
	-1   0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 601CA5AE
P 4100 3750
F 0 "#PWR?" H 4100 3500 50  0001 C CNN
F 1 "GND" H 4105 3577 50  0000 C CNN
F 2 "" H 4100 3750 50  0001 C CNN
F 3 "" H 4100 3750 50  0001 C CNN
	1    4100 3750
	1    0    0    -1  
$EndComp
$Comp
L Switch:SW_Push SW1
U 1 1 6013F9ED
P 4300 3750
F 0 "SW1" H 4300 4035 50  0000 C CNN
F 1 "SW_Push" H 4300 3944 50  0000 C CNN
F 2 "" H 4300 3950 50  0001 C CNN
F 3 "~" H 4300 3950 50  0001 C CNN
	1    4300 3750
	1    0    0    -1  
$EndComp
$Comp
L Device:R_Small 9.45kOhm
U 1 1 601376D3
P 3650 3400
F 0 "9.45kOhm" V 3450 3400 50  0000 C CNN
F 1 "R4" V 3550 3400 50  0000 C CNN
F 2 "" H 3650 3400 50  0001 C CNN
F 3 "~" H 3650 3400 50  0001 C CNN
	1    3650 3400
	0    1    -1   0   
$EndComp
Wire Wire Line
	4500 3750 4950 3750
Wire Wire Line
	4950 3750 4950 3500
Wire Wire Line
	4950 3500 5550 3500
Wire Wire Line
	3000 3400 3200 3400
Wire Wire Line
	3200 4400 3200 3400
Connection ~ 3200 3400
Wire Wire Line
	3200 3400 3550 3400
Wire Wire Line
	4300 4200 4300 4000
Wire Wire Line
	4300 4000 4950 4000
Text Label 1700 3050 0    50   ~ 0
Buzzer
Text Label 1850 3150 0    50   ~ 0
5V
Text Label 1800 3250 0    50   ~ 0
GND
Wire Wire Line
	3750 3400 5550 3400
Wire Wire Line
	2250 3100 2500 3100
Wire Wire Line
	2500 3100 2500 3400
$EndSCHEMATC
