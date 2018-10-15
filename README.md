# CANquitto

  CANquitto is a new canbus protocol library designed for teensy 3.x series MCUs. Currently in development stage, it has adopted quite a few features. This library takes advantage of IFCT and Circular_Buffer libraries when dealing with payload re-assembly.
  
  Forum: https://forum.pjrc.com/threads/53776-CANquitto
  

* Multi-master design: nodes can control each other, and 2 or more nodes can control a single node.

* Payload transfers: You can send payloads to other nodes which they'll receive in their callback.

* GPIO controls, read/write/toggle/pinmode,analogread pins on any nodes

* Serial/UART controls, full UART/USBSerial functionality combined with Stream class.
