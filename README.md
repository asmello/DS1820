# DS1820
A driver for the DS1820 temperature sensor using the LPC1768 microcontroller.

## License

*Original License:*

> mbed DS1820 Library, for the Dallas (Maxim) 1-Wire Digital Thermometer
>
> Copyright (c) 2010, Michael Hagberg Michael@RedBoxCode.com
>
> Permission is hereby granted, free of charge, to any person obtaining a copy
> of this software and associated documentation files (the "Software"), to deal
> in the Software without restriction, including without limitation the rights
> to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
> copies of the Software, and to permit persons to whom the Software is
> furnished to do so, subject to the following conditions:
>
> The above copyright notice and this permission notice shall be included in
> all copies or substantial portions of the Software.
>
> THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
> IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
> FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
> AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
> LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
> OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
> THE SOFTWARE.

This version represents a significant rewrite of Michael Hagberg's original library, and can be considered a fork.
The original licensing terms are retained out of necessity.

## Hello World

```C
#include "mbed.h"
#include "DS1820.h"

const int MAX_PROBES = 16;
DS1820* probe[MAX_PROBES];

int main()
{
    int num_devs;
    for (num_devs = 0; num_devs < MAX_PROBES; ++num_devs) {
        probe[num_devs] = new DS1820(p8);
        if (not probe[num_devs]->search_ROM()) {
            delete probe[num_devs];
            break;
        }
    }
    while (true) {
        probe[0]->convert_temperature(true, DS1820::all_devices);
        for (int i = 0; i < num_devs; i++) {
            printf("DEV%d:temperature: %f\n", i, probe[i]->temperature());
        }
    }
}
```
