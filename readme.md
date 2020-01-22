# DrumKid
## Lo-fi aleatoric Arduino drum machine

![DrumKid](v4/misc/drumkid1.jpg?raw=true "DrumKid")

### Concept
A drum machine which generates rhythms using probability. Imagine a traditional step-based drum machine, but where various dice are rolled for each step, with the resulting numbers used to determine the behaviour of the beat.

### Setup
DrumKid is based around an Arduino, and coded using the Arduino IDE. Audio is produced by the Mozzi library. You can build a version of DrumKid using an Arduino, a breadboard, and standard electronic components, or you can build/buy the "full" version described below.

![Breadboard layout](v4/breadboard/drumkid%20breadboard.png?raw=true "Breadboard layout")

### Design
DrumKid is a portable, handheld, battery-powered instrument with a minimalist design. It consists of a PCB (printed circuit board) for the front panel, featuring knobs, buttons, LEDs, a power switch and a headphone/line output. The electronics are protected/enclosed by two laser-cut pieces and six metal standoffs.

### Open Source
This project is almost entirely open source. The code, breadboard layout, schematics, Gerber (PCB) files, and laser-cutter files are all included in the repository, with the aim of making this project easy to duplicate and adapt. Please note that the project depends on the Mozzi library, which is released under a Creative Commons non-commercial licence. I have used it here with kind permission from its creator, but please be aware that if you create a commercial product using this code, you will need to request permission from him. Go to <https://sensorium.github.io/Mozzi/> for more details.

### Versions
DrumKid has been through various iterations, and I have taken to referring to each new PCB design as a new version. I will try not to make too many breaking changes from one version to the next, and I'm currently hoping that version 6 will be around for a long time, but please make sure to match the PCB/breadboard version number to the Arduino code version number.

Versions 1-3 were early prototypes.

Version 4 was the first successful working design, with full code, schematics, Gerbers, etc available on GitHub.

Version 5 contained various PCB errors and has been superseded, but is very similar to V6.

Version 6 is under development right now and will hopefully be the basis of the first commercially available DrumKid - finalised breadboard files, BOM, etc will appear in due course.