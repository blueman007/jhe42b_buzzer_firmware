# JHE42B and JHE42B_S open firmware

The goal of this project is to replace the default firmware of the JHE42B and JHE42B_S buzzers, which use the STM8S003F3 chip, with a custom firmware that introduces common features found in other autonomous buzzers, especially the ability to disarm the buzzer without using the onboard button.

# Features

(checked if implemented)

- [x] BEEPS every 30s after power loss (default)
- [x] Turn ON/OFF onboard red LED depending on arming status (ON when armed, blinking when triggered, off when disarmed)
- [x] Arm/Disarm system when button is long-pressed
- [x] Disarm if power supply is removed after ``10-12s`` if armed (usefull when button is not accesible)
- [x] Power saving by changing BEEPS frequency `10s` / `30s` / `60s` (default ``30s``)
- [x] Sleep mode when disarmed in order not to strain the battery
- [x] BEEPS on signal pin request
- [x] LIGHT flashs independant of beeps


## Board testing matrix

|Model|BEEPS on power loss|Arming Red LED|Button disarm| Power source trick disarm| Power saving | LED Flash | BEEPS on signal
|---|---|---|---|---|---|---|---|
|JHE42B|:heavy_check_mark:|:heavy_check_mark:|:heavy_check_mark:|:heavy_check_mark:||:heavy_check_mark:|:heavy_check_mark:|
|JHE42B_S|:grey_question:|:grey_question:|:grey_question:|:grey_question:||:grey_question:|:grey_question:|

:x: ko / :heavy_check_mark: ok / :grey_question: todo or more testing required

# User manual

1. ``Arming`` the Buzzer:
   - Apply power to the buzzer to arm it. You will hear a 3-note arming sequence and see the red LED turn ON.
2. ``Alarm``:
   - The buzzer will beep every 30 seconds after power is lost.
3. ``Disarming`` the Buzzer:
   - Long press the onboard button for 8 seconds (you hear a 3-notes melody when disarming), or
   - Apply power for ``10 seconds`` and then remove it.
   - You will hear a 3-note disarming tune, and the red LED will turn OFF.
4. ``Rearming``:
   - After disarming, long press the onboard button for 8 seconds or power cycle the device to be rearm ((you hear a 3-notes melody when arming).


# How to Build/Flash
## Requirements
- [vscode](https://code.visualstudio.com/) or any IDE supporting [PlatformIO](https://platformio.org/)
- [ST-link v2](https://a.aliexpress.com/_ExNkgOT) (so you can write firmware onto the STM8 flash memory) [how to connect it?](#STMLinker-connection)

## PlatformIO setup

- Follow instruction of [platfomio-ststl8](https://github.com/platformio/platform-ststm8#readme)
- clone the repo and open the project in vscode/pio
- [PlatformIO](https://platformio.org/) should download and install all the toolchain/dependencies
- build
- upload/flash

Otherwise add board definition [stm8s003f3.json](https://github.com/nerocide/platform-ststm8/blob/develop/boards/stm8s003f3.json) in :
> C:\Users\<user>\.platformio\platforms\ststm8\boards


## Unlock chip (inmportant)
Every chip has to be unlocked once (from experience), after the first unlock it's not needed anymore.
```
cd .platformio\packages\tool-stm8tools
stm8flash -c stlinkv2 -u -p stm8s003?3
```

## STMLinker connection

Connect the wires to the linker as shown below.

<img src="https://user-images.githubusercontent.com/32848699/128609610-dc31b74a-f0d0-4a57-a1cb-39078045d49f.jpg" width="500"/>


Alternatively, if you don't want to solder on the board, you can use a setup like this:

<img src="ressources/img/pince-a-linge.jpg" width="100"/>
+
<img src="ressources/img/1mm-vertical-pin-header.jpg" width="135"/>


# Compatible Boards
Model|Controller|Chip Mark|Pictures
-----|----------|---------|-----
JHE42B|STM8S003F3 TSSOP20|STM8S003F3P6|<img src="ressources/img/JHE42B_front.jpg" width="250"/> <img src="ressources/img/JHE42B_back.jpg" width="250"/>
JHE42B_S|STM8S003F3 UFQFPN20|S033PHVG822Y|<img src="ressources/img/JHE42B_S_front.jpg" width="250"/> <img src="ressources/img/JHE42B_S_back.jpg" width="250"/>


## Commercial shot
<!-- ![Commercial shot](/img/both_buzzer_full.jpg =100x) -->
<img src="ressources/img/both_buzzer_full.jpg" width="300"/>


<!-- ## JHE42B -->
<!-- ![JHE42B front](/img/JHE42B_front.jpg) -->
<!-- ![JHE42B back](/img/JHE42B_back.jpg) -->

<!-- <img src="ressources/img/JHE42B_front.jpg" width="300"/> <img src="ressources/img/JHE42B_back.jpg" width="300"/> -->
<!-- <img src="/img/jhe42b_schematic.png" width="600"/> -->

<!-- ## JHE42B_S -->
<!-- ![JHE42B_S front](/img/JHE42B_S_front.jpg) -->
<!-- ![JHE42B_S back](/img/JHE42B_S_back.jpg) -->

<!-- <img src="ressources/img/JHE42B_S_front.jpg" width="300"/> <img src="ressources/img/JHE42B_S_back.jpg" width="300"/> -->

# Reverse Engineering Process
(checked if working)

- [x] power supply detection
- [x] blink onboard red LED
- [x] blink onboard white LED
- [x] bip on buzzer
- [x] button press
# Resources
[ST STM8S003F3 DataSheet](ressources/doc/STM8S0003F3_datasheet_dm00024550.pdf)
[ST STM8 debug]([ressources/doc/STM8S0003F3_datasheet_dm00024550.pdf](https://circuitdigest.com/microcontroller-projects/serial-monitor-on-stm8s-using-cosmic-and-stvd))
