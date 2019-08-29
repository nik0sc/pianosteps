# pianosteps
This is the Python code (well, my code) for the SUTD sonic staircase (aka "piano steps") installation at Esplanade Flipside 2018.

The hardware is a Raspberry Pi 3 with an array of 8 SHARP GP2Y0A02YK0F infrared distance sensors connected to two Adafruit ADS1115 
4-channel I2C analog-to-digital converter (ADC) breakout boards. Sound synthesis is done in software (because it's a Raspberry 
Pi...) and the output piped to a very large speaker hidden somewhere near the steps.

We achieved actually quite good responsiveness once we cranked up the ADC sample rate and implemented a simple low-pass filter in 
software. Thanks to Ronen from [GRIDI](https://en.wikipedia.org/wiki/GRIDI) for the filtering tip. 

In hindsight, our solution achieved amazing reliability for what it was. The I2C bus wiring was far too long and definitely not 
carefully controlled for capacitance. The physical sensors were installed in flimsy little plastic boxes and secured to the ground 
with... gaffer tape. In a high foot traffic area with way too many curious kids and distracted adults just waiting to kick them 
out of alignment. We left simple instructions to the Esplanade staff to turn the Pi off and on again if the sound glitched or got 
stuck. And this worked! 

In other words, this was a glorious little hack.

Thanks to Tzu Shieh (hardware) and Lionell (idea) and the Mission X team (moral support) too.

- `piano.py` main script run on Pi startup
- `piano.md` documentation for field maintenance
- `simpletest*.py` script for checking adc values
- `services/` systemd units
- `confs/` configuration files for `piano.py`, has option of C major scale or SMB theme
