*********************
USB to I2S Playback Demo
************************

.. warning::

   This example is based on the RTOS framework and drivers, and it can lead to latency of up to 20 ms in the system.
   More information can be found in the Overview section of the USB Audio example in the Programming Guide.

This is the XCORE-VOICE USB audio playback example design.

The example system implements a stereo USB Audio Class 2.0 interface and a stereo I2S master output path.
Audio received from the USB host is forwarded directly to the I2S tile and then sent to the DAC over I2S.
There is no ASRC in the playback path.

The I2S output interface is a stereo 32 bit interface running at 48 kHz.

The USB interface is a stereo, 32 bit, 48 kHz, High-Speed, USB Audio Class 2 interface.

Supported Hardware
==================

This example application is supported on the `XK-VOICE-L71 <https://www.digikey.co.uk/en/products/detail/xmos/XK-VOICE-L71/15761172>`_ board.
In addition to the XK-VOICE-L71 board, it requires an XTAG4 to program and debug the device.

To demonstrate the audio path, the XK-VOICE-L71 device uses its onboard codec/DAC and I2S output path.
USB playback data is converted to I2S by the firmware and sent to the codec through the board's audio routing.

For audio output, connect the board's speaker, headphone, or line-out path according to the XK-VOICE-L71 hardware guide.
The codec is initialized over I2C at startup, so no external I2S master is required for the USB playback demo.


Obtaining the app files
=======================

Download the main repo and submodules using:

::

   $ git clone --recurse git@github.com:xmos/sln_voice.git
   $ cd sln_voice/


Building the app
================

First make sure that your XTC tools environment is activated.

Linux or Mac
------------

After having your python environment activated, run the following commands in the root folder to build the firmware:

::
   $ pip install -r requirements.txt
   $ mkdir build
   $ cd build
   $ cmake --toolchain ../xmos_cmake_toolchain/xs3a.cmake  ..
   $ make example_asrc_demo -j

Following initial ``cmake`` build, for subsequent builds, as long as new source files are not added, just type:

::

   $ make example_asrc_demo -j

``cmake`` needs to be rerun to discover any new source files added.

Windows
-------

It is recommended to use `Ninja` or `xmake` as the make system under Windows.
`Ninja` has been observed to be faster than `xmake`, however `xmake` comes natively with XTC tools.
This firmware has been tested with `Ninja` version v1.11.1.

To install Ninja, activate your python environment, and run the following command:

::

   $ pip install ninja

After having your python environment activated, run the following commands in the root folder to build the firmware:

::

   $ pip install -r requirements.txt
   $ md build
   $ cd build
   $ cmake -G "Ninja" --toolchain  ..\xmos_cmake_toolchain\xs3a.cmake ..
   $ ninja example_asrc_demo.xe -j

Following initial ``cmake`` build, for subsequent builds, as long as new source files are not added, just type:

::

   $ ninja example_asrc_demo.xe -j

``cmake`` needs to be rerun to discover any new source files added.

Running the app
===============

To run the app, either xrun or xflash can be used. Connect the XK-VOICE-L71 board to the host and type:

::

   $ xrun example_asrc_demo.xe

Optionally, xrun ``--xscope`` can be used to provide debug output.

or

::

   $ xflash example_asrc_demo.xe



Operation
=========

When the example runs, USB playback data is received from the host, buffered, forwarded over the intertile link, and emitted on the I2S
output that drives the codec/DAC. The receive path from I2S back to USB is still present for monitoring and loopback-style capture use cases.
