# Holiday Lighting With `WS2812B`s

This is a simple 'ooh, pretty colors' project targeting the cheap-and-cheerful STM32F103C8 chip. It is still a work-in-progress, but the basic idea seems to work.

In a nutshell, the application drives a string of `WS2812B` 'NeoPixel' LEDs from the `SPI1` peripheral connected to pin `B5`.

Nothing special there, but the application expects the LEDs to be split into groups and sandwiched between acrylic shapes, with those shapes connecting to one another in a string. So it generates cycling patterns of lighting effects, given the number of stars and the number of LEDs in each star.

[ TODO: Add a picture ]

'Tis the season! :)

# Future Work

It's late December as I write this, so I'm not going to accomplish any of this until the next holiday season. But:

* Add buttons to change patterns.

* Do a better job of packing the colors for SPI transmission, to reduce RAM usage.

* Add support for STM32G031Jx chips - this is a great use case for a tiny 8-pin ARM chip!
