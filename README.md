# `ti-ina219`

Driver for Texas Instruments INA219 device. This device is a high-side
current shunt and power monitor with i2c interface. [Driver data sheet
here!](https://cdn-shop.adafruit.com/datasheets/ina219.pdf).

Includes a specialization of the driver for the adafruit breakout
board including this [device]
(https://learn.adafruit.com/adafruit-ina219-current-sensor-breakout/overview). The
device is specialized to use the current shunt resistor value and have
preset configurations, similar to the [arduino
library](https://github.com/adafruit/Adafruit_INA219).

Testing is done on a nucleo-f103rb board with the power input
connected to the JP6 jumper on the nucleo board, allowing it to
measure the current and voltage on the nucleo board.


## License

Licensed under either of

- Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or
  http://www.apache.org/licenses/LICENSE-2.0)

- MIT license ([LICENSE-MIT](LICENSE-MIT) or http://opensource.org/licenses/MIT)

at your option.

### Contribution

Unless you explicitly state otherwise, any contribution intentionally submitted
for inclusion in the work by you, as defined in the Apache-2.0 license, shall be
licensed as above, without any additional terms or conditions.
