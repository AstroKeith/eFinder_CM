# eFinder CM (aka Nexus eFinder Pro)

## Basics

eFinder CM is a digital finder for astronomical telescopes, utilising plate-solving to improve pointing accuracy, and an IMU to estimate telescope position in-between plate-solves.
The DIY versions are, 
- Full Version, functionally the same as the commercially available device (AstroDevices).
- Minimal Version, with no UART board it has no ServoCat/SkyTracker support.

<img width="460" height="550" alt="IMG_7918" src="https://github.com/user-attachments/assets/8701e1e7-83ab-4ffc-a69f-7d29d30fc9a1" />

Requires:

- microSd card loaded with Raspberry Pi 64bit Bookworm OS Lite (No desktop)
- Raspberry Pi CM4 or Pi4b
- BNO085 IMU
- Waveshare Nano Base Board (B)
- CP2303 UART module
- A custom housing. 3d print files will be available
- A Camera, the RPi HQ Camera module is recommended, although the Arducam equivalent can work.
- Camera lens, 25mm f1.2 cctv lens
- A host computer (or Nexus DSC Pro)

Full details at [
](https://astrokeith.com/equipment/efinder)https://astrokeith.com/equipment/efinder

The repo includes a pdf describing how to prepare a complete working micro sdCard for the Pi Zero 2W

## Compatibility

The eFinder CM is designed to operate alongside a host computer or Nexus DSC Pro


## Operation

ssh & Samba file sharing is enabled at efinder.local, or whatever hostname you have chosen.

A forum for builders and users can be found at https://groups.io/g/eFinder

## Acknowledgements and Licences

The eFinder CM uses olive-solve.

