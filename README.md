# lib_i2cadc
adc board control library for jig (ltc2309)

```
Usage: ./lib_i2cadc [-D:device] [-p:pin name] [-v]

  -D --Device         Control Device node(i2c dev)
  -p --pin name       Header pin name in adc board (con1, con1.1...)
  -v --view all port  ALL Haader pin info display.

  e.g) ./lib_i2cadc -D /dev/i2c-0 -p con1.1
```
