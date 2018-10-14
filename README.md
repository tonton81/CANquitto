# CANquitto

  CANquitto is a new canbus protocol library designed for teensy 3.x series MCUs. Currently in development stage, it has adopted quite a few features. This library takes advantage of Circular_Buffer library when dealing with payload re-assembly.
  
  Forum: https://forum.pjrc.com/threads/53776-CANquitto
  
   * Multi-master design: nodes can control each other, and 2 or more nodes can control a single node.
   * Payload transfers: You can send payloads to other nodes which they'll receive in their callback.
   * GPIO controls, currently can read/write/toggle pins.
   * Serial/UART controls, currently can write/print/println to any UART or USBSerial of other nodes.
