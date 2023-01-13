# ESP_ArtnetRelay
Arduino sketch for controlling non-DMX devices via Artnet Protocol

This is a sketch for ESP8266 (and possibly also ESP32) that will listen for Artnet packages
on a network and control one ore two relays connected to configurable GPIO pins.

Artnet is a protocol to send DMX packages over WiFi. 
DMX is an abbreviation for Digital Multiplex. 
It's the standard protocol that is used to remotely control lighting fixtures such as light spots,
mirror balls, fogging machines, moving light heads, scanners etc.
However, not all - espacially old - fixtures are capable of being controller over DMX or Artnet.

When I was finalizing my light setup, I ended up with several devices that did support DMX nor Artnet.
Examples for this were a mirror ball, some UV lamps, an old analoguous stroboscope and more.
I started to write dedicated sketches for all of these when I noticed I was doing the same code over
and over again ... configure WiFi settings, listen for Artnet packages, sort out duplicates, control GPIO
pins and so forth. So I decided to merge those different sketches nd have something more generic that 
I can reuse for different (future) devices as well.

Right now, this sketch can control up to 2 relays connected to GPIO pins that can be configured at runtime.
It supports also runtime WiFi configuration via WiFiManager and is able to create a square wave signal on a
GPIO that can be used to trigger e.g. analogue stroboscopes.
Moreover it supports also temperature monitoring e.g. for fan control or switching off a device when it gets too hot.




