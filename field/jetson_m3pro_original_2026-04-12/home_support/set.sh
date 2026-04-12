#! /bin/bash
sleep 3

/usr/bin/pactl set-default-sink alsa_output.usb-C-Media_Electronics_Inc._USB_Audio_Device-00.analog-stereo
/usr/bin/pactl set-default-source alsa_input.usb-C-Media_Electronics_Inc._USB_Audio_Device-00.mono-fallback
