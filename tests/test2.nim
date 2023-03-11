import unittest

import
  std / [os, strutils]

import pigpio/pigpiod_if2
test "test if2":
  var pi = pigpio_start(nil, "8888")
  require pi != PI_INIT_FAILED

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

  require pi.set_mode(pin, PI_OUTPUT) == 0

  let (off, on) =
    if isInitHigh:
      (PI_HIGH, PI_LOW)
    else:
      (PI_LOW, PI_HIGH)

  check pi.gpio_write(pin, off) == 0

  sleep 100
  check pi.gpio_write(pin, on) == 0
  sleep 500
  check pi.gpio_write(pin, off) == 0

  pi.pigpio_stop
