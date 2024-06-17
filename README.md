# Variwheeler

Variwheeler is a variable geometry omnidirectional car currently in its alpha version. This repository contains the CAD models and a library to drive the car.

## Table of Contents

- [Introduction](#introduction)
- [Features](#features)
- [Installation](#installation)
- [Usage](#usage)
- [CAD Models](#cad-models)
- [Library](#library)
- [License](#license)

## Introduction

Variwheeler is an innovative project aimed at developing a car with variable geometry and omnidirectional capabilities. This alpha version includes the initial CAD designs and a driving library.

## Features

- Variable geometry for dynamic adaptability
- Omnidirectional movement thanks to mechanical wheels
- Comprehensive control library.

## Installation

### Downloaded library

If you want to install the library manually in your project, download the repository and enter the contents of the [library](https://github.com/Inderlard/Variwheeler/tree/main/Library) folder in your library address.

#### Platform io:

```
Project >> lib >> Variwheeler
```

#### Arduino:

```
Documents >> Arduino >> libraries >> Variwheeler.
```

### Library by link

You can use [this link](https://github.com/Inderlard/Variwheeler/tree/main/Library) to include in

```Platformio
lib_deps =
	https://github.com/Inderlard/Variwheeler/tree/main/Library
```

### Dependencies

Variwheeler library requires [Adafruit PWM Servo Driver Library](https://github.com/adafruit/Adafruit-PWM-Servo-Driver-Library).
You can install it in either of the two ways above or with the library manager of your working environment.

## Usage

### Running the Car

To drive the car using the provided library, follow these steps:

1. Assemble the car according to the documentation provided with the CAD files.
2. Import the library into your environment.

```C++
#include variwheeler.h
```

3. Use the library functions to run the car.
   You can get more information about usage of the library in his path

## CAD Models

The CAD models for Variwheeler are available in the `cad` directory. These models include all the necessary components to assemble the car.

- [CAD Models Directory](3D Files)

![](Media\Variwheeler.JPG)

## Library

The [library](https://github.com/Inderlard/Variwheeler/tree/main/Library) provided in this repository allows for easy control of the Variwheeler.

## License

This project is licensed under the GNU General Public License v3.0. See the [LICENSE](./LICENSE) file for details.
