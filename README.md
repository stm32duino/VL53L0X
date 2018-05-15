# VL53L0X
Arduino library to support the VL53L0X Time-of-Flight and gesture-detection sensor

## API

This sensor uses I2C to communicate. And I2C instance is required to access to the sensor.

The API provides simple distance measure, single swipe gesture detection,
directional (left/right) swipe gesture detection and single tap gesture detection.

## Examples

There are 3 examples with the VL53L0X library.
* DISCO_IOT_DataLogTerminal: This example code is to show how to get proximity
  values of the onboard VL53L0X sensor

* DISCO_IOT_Gesture_Swipe1: This example code is to show how to combine the
  proximity value of the onboard VL53L0X sensor together with the gesture library
  in order to detect a simple swipe gesture without considering the direction.

* DISCO_IOT_Gesture_Tap1: This example code is to show how to combine the
  proximity value of the onboard VL53L0X sensor together with the gesture
  library in order to detect a simple tap gesture.
  
## Dependencies

The VL53L0X library requires the following STM32duino library:

* STM32duino Proximity_Gesture: https://github.com/stm32duino/Proximity_Gesture

## Note

The maximum detection distance is influenced by the color of the target and
the indoor or outdoor situation due to absence or presence of external
infrared.
The detection range can be comprise between ~40cm and ~120cm. (see chapter 5 of
the VL53L0X datasheet).
If you need an higher accuracy (up to +200cm), you should implement your own
function.
The library should work also with standard Arduino boards. In this case you just
need to adjust the code in the sketch in order to use the correct Wire instance and
the correct pin number for XSHUT and GPIO1 pins.

## Documentation

You can find the source files at  
https://github.com/stm32duino/VL53L0X

The VL53L0X datasheet is available at  
http://www.st.com/content/st_com/en/products/imaging-and-photonics-solutions/proximity-sensors/vl53l0x.html
