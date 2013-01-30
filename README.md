# RF12_Stream

Arduino library to provide a software emulation of a transparent
serial link using the HopeRF RFM12B transceiver modules. The library
extends the Arduino stream class so the user is able to access the
RFM12B as a serial device in much the same way as the Arduino
environment enables access to the hardware UART(s).

# Example sketches

## SerialStream
Sketch to transfer data between the serial port and an RFM12B. This
sketch can be used to emulate an XBee or Ciseco XRF radio link. You
will need to use a pair RFM12B modules with similar software at both
ends of the link. The firmware size is under 8kB when compiled for the
ATtiny84 so it may be possible to run the sketch on the RFM12B to Pi
expansion board, http://shop.openenergymonitor.com/raspberry-pi/.


## RFM12B_autoselect
Example of how the begin() function can be used to check whether the
RFM12B is fitted, and to fall back to using a XRF/XBee radio if it is
not. By using a reference to a Stream object this concept can be used
to easily create a single firmware image which supports both types of
hardware.

# License

The RF12_Stream library is copyright S R Marple, 2013 and is released
under the MIT License. See LICENSE for full details.
