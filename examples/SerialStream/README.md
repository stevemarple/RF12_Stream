# SerialStream

SerialStream enables an RFM12B to be used as a bi-directional
transparent serial link. Text or binary data can be read from or
written to the RFM12B in exactly the same way as the standard Arduino
Serial object. Since it is a Stream object the Streaming class (which
overloads the >> and << operators) can also be used with
RF12_Stream. The RF12_Stream class automatically breaks up the data to
be transmitted into short packets which are sent using a
slightly-modified version of the RF12 library. ACKs and
retransmissions are employed to prevent losing data from dropped
packets. A packet number is included to prevent duplicated data, which
could otherwise occur for the case when the ACK is not received and a
packet is retransmitted. The class keeps track of the number of
packets sent and received, and the number of retransmissions, data
which may be useful to identify the optimal channel on which to
operate.

SerialStream passes data between the Serial and RFM12B interfaces. It
also includes a command-mode which accepts AT-style commands to alter
various parameters relating to the RFM12B and RF12_Stream.

## Entering command mode

To enter command mode pause sending data on the serial port for 1
second (or longer), then send 3 plus signs ("+++") within 500
milliseconds. Pause sending data for a further one second (or
longer). Command mode is exited automatically 5 seconds after the last
character was received, or on receipt of the ATDN command.


## AT commands

Numerical values 

### ATAC
Apply changes made by ATCH, ATCN, ATGR, ATLN, and ATRN. The changes
are saved to EEPROM and the micronctroller rebooted.

### ATCH 
Select channel (band). Unsigned byte value:
  1: 433MHz
  2: 868MHz
  3: 915MHz

Without parameters the current band is printed. Any change made must
be applied with ATAC. Any change made must be applied with ATAC.

### ATCN
Select channel number. 

12 bit unsigned value. See "Frequency Setting" in the Jeelabs RFM12B
command calculator for further details. Without parameters the current
channel number is printed. Any change made must be applied with ATAC.

### ATDN
Exit command mode ("done").

### ATGR

Group ID. Unsigned byte value. Without parameters the current channel
number is printed. Any change made must be applied with ATAC.

### ATLI
Set the behaviour for the LED indicator. Takes one parameter:
  A: display activity
  D: debug mode. LED off when in normal mode. Fully on in command
  mode, increasing brightness as the '+' characters are entered.
  H: heart beat (flashes at 1Hz)
  0: LED off
  1: LED on
  O: Synonym for 0 (zero).

Without parameters the current mode is printed.

### ATLN

Local node ID. Unsigned byte value in the range 1 - 31 inclusive. Node
ID 31 will receive packets for all node IDs. Without parameters the
current node ID is printed. Any change made must be applied with ATAC.

#### ATRB

Reboot the microcontroller.

### ATRN

Remote node ID. Unsigned byte value in the range 1 - 30
inclusive. Without parameters the current node ID is printed. Any
change made must be applied with ATAC.

### ATVR
Print the firmware version string.


## References

JeeLabs RFM12B command calculator, http://tools.jeelabs.org/rfm12b
