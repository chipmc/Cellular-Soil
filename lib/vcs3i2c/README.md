# TinoviSoilSensor

A Particle library for TinoviSoilSensor

## Welcome to your library!

To get started, modify the sources in [src](src). Rename the example folder inside [examples](examples) to a more meaningful name and add additional examples in separate folders.

To compile your example you can use `particle compile examples/usage` command in [Particle CLI](https://docs.particle.io/guide/tools-and-features/cli#update-your-device-remotely) or use our [Desktop IDE](https://docs.particle.io/guide/tools-and-features/dev/#compiling-code).

Libraries can also depend on other libraries. To add a dependency use [`particle library add`](https://docs.particle.io/guide/tools-and-features/cli#adding-a-library) or [library management](https://docs.particle.io/guide/tools-and-features/dev/#managing-libraries) in Desktop IDE.

After the library is done you can upload it with `particle library upload` or `Upload` command in the IDE. This will create a private (only visible by you) library that you can use in other projects. If you wish to make your library public, use `particle library publish` or `Publish` command.

_TODO: update this README_

## Usage

Connect XYZ hardware, add the TinoviSoilSensor library to your project and follow this simple example:

```
#include "TinoviSoilSensor.h"
TinoviSoilSensor tinoviSoilSensor;

void setup() {
  tinoviSoilSensor.begin();
}

void loop() {
  tinoviSoilSensor.process();
}
```

See the [examples](examples) folder for more details.

## Documentation

## Interfacing from Arduino
**WARNING!!! use 3.3 voltage levels only, more voltage will damage device**

###wiring to Arduiono:

Arduiono pin #3V3 - sensor red (3.3v)

Arduiono pin #A4 - sensor green (SDA)

Arduiono pin #A5 - sensor white (SCL)

Arduiono pin #GND - sensor black (GND)

pin #GND - shield (GND)

**SDA and SCL lines requires pull-up resitors to 3.3v line, we recommend to use 1.8K resistors, because of long wiring to i2c sensor.**

### API
```
SVCS3();
  //pass i2c addres of sensor, default 0x63
  int init(int address);
  // update i2c address of sesnor
  int newAddress(byte newAddr);
  // hold sesnor in air or put in dry soil and call  (offset DP = 1 or VWC=0%)
  int calibrationAir();
  // submerge sesnor in the water or soil with water (offset DP = 80 or VWC=100%)
  int calibrationWater();
  // submerge sesnor in EC calibration fluid or soil with known EC and supply correc EC vaue in
  int calibrationEC(int16_t valueUs);
  //initate reading, then need to wait for 100ms to let reading to finish
  int newReading();
  float getE25();
  float getEC();
  float getTemp();
  float getVWC();
  //get all values, supply float[4] , return 0-DP;1-EC;2-Temp;3-VWC
  void getData(float retVal[]);
```


### Get software

This sample software demonstrates hot to read data from sensor.

Sensor default I2C address is 0x63.

Download Arduino library from [there.](https://github.com/tinovi/i2cArduino)

<a href="https://www.tindie.com/stores/tinovi/"><img src="https://d2ss6ovg47m0r5.cloudfront.net/badges/tindie-mediums.png" alt="I sell on Tindie" width="150" height="78" /></a>


## Contributing

Here's how you can make changes to this library and eventually contribute those changes back.

To get started, [clone the library from GitHub to your local machine](https://help.github.com/articles/cloning-a-repository/).

Change the name of the library in `library.properties` to something different. You can add your name at then end.

Modify the sources in <src> and <examples> with the new behavior.

To compile an example, use `particle compile examples/usage` command in [Particle CLI](https://docs.particle.io/guide/tools-and-features/cli#update-your-device-remotely) or use our [Desktop IDE](https://docs.particle.io/guide/tools-and-features/dev/#compiling-code).

After your changes are done you can upload them with `particle library upload` or `Upload` command in the IDE. This will create a private (only visible by you) library that you can use in other projects. Do `particle library add TinoviSoilSensor_myname` to add the library to a project on your machine or add the TinoviSoilSensor_myname library to a project on the Web IDE or Desktop IDE.

At this point, you can create a [GitHub pull request](https://help.github.com/articles/about-pull-requests/) with your changes to the original library.

If you wish to make your library public, use `particle library publish` or `Publish` command.

## LICENSE
Copyright 2019 Tinovi

Licensed under the <insert your choice of license here> license
