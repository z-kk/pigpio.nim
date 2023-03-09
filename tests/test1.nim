import unittest

import
  std / [os, strutils]

import pigpio
test "test pin up/down":
  var pin: cuint
  while true:
    stdout.write "target pin: "
    try:
      let pinStr = stdin.readLine
      if pinStr.len == 0 or pinStr[0].ord == 3:
        quit("canceled")
      pin = pinStr.parseInt.cuint
      if pin < PI_MIN_GPIO or pin > PI_MAX_GPIO:
        echo "input 0-31 value"
        continue
      break
    except:
      echo "input numeric value"
      continue

  stdout.write "init pin HIGH? [Y/n] "
  let isInitHigh = stdin.readLine.toLower != "n"

  require gpioInitialise() != PI_INIT_FAILED
  require pin.gpioSetMode(PI_OUTPUT) == 0
  let (off, on) =
    if isInitHigh:
      (PI_HIGH, PI_LOW)
    else:
      (PI_LOW, PI_HIGH)

  check pin.gpioWrite(off) == 0

  sleep 100
  check pin.gpioWrite(on) == 0
  sleep 500
  check pin.gpioWrite(off) == 0

  gpioTerminate()
