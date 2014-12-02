AP300
=====
Control of a Pluggit AP300 device over RF
-----------------------------------------------
My project was inspired from another nice project written in german: https://github.com/d00616/P300.

However, I wanted it to keep the whole system complexity and cost (~10EUR) as low as possible.
Another advantage of this project is that there is no need to open the AP300 case in order to control it! Of course, air quality monitoring could therefore be not implemented, but it also wasn't on agenda.

This project is based on an Arduino Pro Mini board equipped with an ATmega328P and serves as main controller, which is connected via SPI to the RF module with an nRF905 chip (http://www.fasttech.com/product/1380701-nrf905-arduino-compatible-wireless-communication).
Bothe are supplied with 3.3V.
The Arduino board is connected to the PC via a USB->Serial converter module which allows easy SW upload from Arduino IDE.
The data received over the RF module is "sniffed" in a first instance by Arduino and transmitted to the PC, where the data is displayed in a serial terminal software window (RealTerm in my case).

Needles to say that you can use another controller board instead of Arduino Pro Mini.
However, keep in mind that the nRF905 module should be supplied with 3.3V, so it would be preferable to use the same supply voltage for your controller as well. Alternatively, if your controller board works with 5V, you could use a level shifter to adapt the SPI lines from 5V to 3.3V, but this will be not discussed and you will get no support here.

Furthermore, the interface to the PC can be replaced by a WiFi<->Serial bridge (I actually intend to do this). In this case your AP300 device turns out to be controllable on your home network over a web interface (I will come back later to this).
