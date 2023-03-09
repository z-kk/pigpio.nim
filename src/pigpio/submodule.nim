#[
  This is free and unencumbered software released into the public domain.

  Anyone is free to copy, modify, publish, use, compile, sell, or
  distribute this software, either in source code form or as a compiled
  binary, for any purpose, commercial or non-commercial, and by any
  means.

  In jurisdictions that recognize copyright laws, the author or authors
  of this software dedicate any and all copyright interest in the
  software to the public domain. We make this dedication for the benefit
  of the public at large and to the detriment of our heirs and
  successors. We intend this dedication to be an overt act of
  relinquishment in perpetuity of all present and future rights to this
  software under copyright law.
  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
  EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
  MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
  IN NO EVENT SHALL THE AUTHORS BE LIABLE FOR ANY CLAIM, DAMAGES OR
  OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
  ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
  OTHER DEALINGS IN THE SOFTWARE.

  For more information, please refer to <http://unlicense.org/>
]#

{.passL: "-lpigpio".}

const
  PIGPIO_VERSION* = 79

#[
  pigpio is a C library for the Raspberry which allows control of the GPIO.

  GPIO*

  ALL GPIO are identified by their Broadcom number.

  Credits*

  The PWM and servo pulses are timed using the DMA and PWM peripherals.

  This use was inspired by Richard Hirst's servoblaster kernel module.

  Notes*

  All the functions which return an int return < 0 on error.

  [*gpioInitialise*] must be called before all other library functions
  with the following exceptions:

  . .
  [*gpioCfg**]
  [*gpioVersion*]
  [*gpioHardwareRevision*]
  . .

  If the library is not initialised all but the [*gpioCfg**],
  [*gpioVersion*], and [*gpioHardwareRevision*] functions will
  return error PI_NOT_INITIALISED.

  If the library is initialised the [*gpioCfg**] functions will return
  error PI_INITIALISED.
]#

const
  PI_INPFIFO* = "/dev/pigpio"
  PI_OUTFIFO* = "/dev/pigout"
  PI_ERRFIFO* = "/dev/pigerr"
  PI_ENVPORT* = "PIGPIO_PORT"
  PI_ENVADDR* = "PIGPIO_ADDR"
  PI_LOCKFILE* = "/var/run/pigpio.pid"
  PI_I2C_COMBINED* = "/sys/module/i2c_bcm2708/parameters/combined"

type
  int32_t* = int32
  uint8_t* = uint8
  uint16_t* = uint16
  uint32_t* = uint32
  uint64_t* = uint64

type
  gpioHeader_t* {.bycopy.} = object
    `func`*: uint16_t
    size*: uint16_t

  gpioExtent_t* {.bycopy.} = object
    size*: csize_t
    `ptr`*: pointer
    data*: uint32_t

  gpioSample_t* {.bycopy.} = object
    tick*: uint32_t
    level*: uint32_t

  gpioReport_t* {.bycopy.} = object
    seqno*: uint16_t
    flags*: uint16_t
    tick*: uint32_t
    level*: uint32_t

  gpioPulse_t* {.bycopy.} = object
    gpioOn*: uint32_t
    gpioOff*: uint32_t
    usDelay*: uint32_t


const
  WAVE_FLAG_READ* = 1
  WAVE_FLAG_TICK* = 2

type
  rawWave_t* {.bycopy.} = object
    gpioOn*: uint32_t
    gpioOff*: uint32_t
    usDelay*: uint32_t
    flags*: uint32_t

  rawWaveInfo_t* {.bycopy.} = object
    botCB*: uint16_t           ##  first CB used by wave
    topCB*: uint16_t           ##  last CB used by wave
    botOOL*: uint16_t          ##  first bottom OOL used by wave
                               ##  botOOL to botOOL + numBOOL - 1 are in use
    topOOL*: uint16_t          ##  last top OOL used by wave
                               ##  topOOL - numTOOL to topOOL are in use.
    deleted*: uint16_t
    numCB*: uint16_t
    numBOOL*: uint16_t
    numTOOL*: uint16_t

  rawSPI_t* {.bycopy.} = object
    clk*: cint                 ##  GPIO for clock
    mosi*: cint                ##  GPIO for MOSI
    miso*: cint                ##  GPIO for MISO
    ss_pol*: cint              ##  slave select off state
    ss_us*: cint               ##  delay after slave select
    clk_pol*: cint             ##  clock off state
    clk_pha*: cint             ##  clock phase
    clk_us*: cint              ##  clock micros

  rawCbs_t* {.bycopy.} = object
    info*: uint32_t            ##  linux/arch/arm/mach-bcm2708/include/mach/dma.h
    src*: uint32_t
    dst*: uint32_t
    length*: uint32_t
    stride*: uint32_t
    next*: uint32_t
    pad*: array[2, uint32_t]

  pi_i2c_msg_t* {.bycopy.} = object
    `addr`*: uint16_t          ##  slave address
    flags*: uint16_t
    len*: uint16_t             ##  msg length
    buf*: ptr uint8_t          ##  pointer to msg data


##  BSC FIFO size

const
  BSC_FIFO_SIZE* = 512

type
  bsc_xfer_t* {.bycopy.} = object
    control*: uint32_t         ##  Write
    rxCnt*: cint               ##  Read only
    rxBuf*: array[BSC_FIFO_SIZE, char] ##  Read only
    txCnt*: cint               ##  Write
    txBuf*: array[BSC_FIFO_SIZE, char] ##  Write

  gpioAlertFunc_t* = proc (gpio: cint; level: cint; tick: uint32_t)
  gpioAlertFuncEx_t* = proc (gpio: cint; level: cint; tick: uint32_t; userdata: pointer)
  eventFunc_t* = proc (event: cint; tick: uint32_t)
  eventFuncEx_t* = proc (event: cint; tick: uint32_t; userdata: pointer)
  gpioISRFunc_t* = proc (gpio: cint; level: cint; tick: uint32_t)
  gpioISRFuncEx_t* = proc (gpio: cint; level: cint; tick: uint32_t; userdata: pointer)
  gpioTimerFunc_t* = proc ()
  gpioTimerFuncEx_t* = proc (userdata: pointer)
  gpioSignalFunc_t* = proc (signum: cint)
  gpioSignalFuncEx_t* = proc (signum: cint; userdata: pointer)
  gpioGetSamplesFunc_t* = proc (samples: ptr gpioSample_t; numSamples: cint)
  gpioGetSamplesFuncEx_t* = proc (samples: ptr gpioSample_t; numSamples: cint;
                               userdata: pointer)
  gpioThreadFunc_t* = proc (a1: pointer): pointer

##  gpio: 0-53

const
  PI_MIN_GPIO* = 0
  PI_MAX_GPIO* = 53

##  user_gpio: 0-31

const
  PI_MAX_USER_GPIO* = 31

##  level: 0-1

const
  PI_OFF*: cuint = 0
  PI_ON*: cuint = 1
  PI_CLEAR*: cuint = 0
  PI_SET*: cuint = 1
  PI_LOW*: cuint = 0
  PI_HIGH*: cuint = 1

##  level: only reported for GPIO time-out, see gpioSetWatchdog

const
  PI_TIMEOUT*: cuint = 2

##  mode: 0-7

const
  PI_INPUT*: cuint = 0
  PI_OUTPUT*: cuint = 1
  PI_ALT0*: cuint = 4
  PI_ALT1*: cuint = 5
  PI_ALT2*: cuint = 6
  PI_ALT3*: cuint = 7
  PI_ALT4*: cuint = 3
  PI_ALT5*: cuint = 2

##  pud: 0-2

const
  PI_PUD_OFF*: cuint = 0
  PI_PUD_DOWN*: cuint = 1
  PI_PUD_UP*: cuint = 2

##  dutycycle: 0-range

const
  PI_DEFAULT_DUTYCYCLE_RANGE*: cuint = 255

##  range: 25-40000

const
  PI_MIN_DUTYCYCLE_RANGE* = 25
  PI_MAX_DUTYCYCLE_RANGE* = 40000

##  pulsewidth: 0, 500-2500

const
  PI_SERVO_OFF*: cuint = 0
  PI_MIN_SERVO_PULSEWIDTH* = 500
  PI_MAX_SERVO_PULSEWIDTH* = 2500

##  hardware PWM

const
  PI_HW_PWM_MIN_FREQ* = 1
  PI_HW_PWM_MAX_FREQ* = 125000000
  PI_HW_PWM_MAX_FREQ_2711* = 187500000
  PI_HW_PWM_RANGE* = 1000000

##  hardware clock

const
  PI_HW_CLK_MIN_FREQ* = 4689
  PI_HW_CLK_MIN_FREQ_2711* = 13184
  PI_HW_CLK_MAX_FREQ* = 250000000
  PI_HW_CLK_MAX_FREQ_2711* = 375000000
  PI_NOTIFY_SLOTS* = 32
  PI_NTFY_FLAGS_EVENT* = (1 shl 7)
  PI_NTFY_FLAGS_ALIVE* = (1 shl 6)
  PI_NTFY_FLAGS_WDOG* = (1 shl 5)

template PI_NTFY_FLAGS_BIT*(x: untyped): untyped =
  (((x) shl 0) and 31)

const
  PI_WAVE_BLOCKS* = 4
  PI_WAVE_MAX_PULSES* = (PI_WAVE_BLOCKS * 3000)
  PI_WAVE_MAX_CHARS* = (PI_WAVE_BLOCKS * 300)
  PI_BB_I2C_MIN_BAUD* = 50
  PI_BB_I2C_MAX_BAUD* = 500000
  PI_BB_SPI_MIN_BAUD* = 50
  PI_BB_SPI_MAX_BAUD* = 250000
  PI_BB_SER_MIN_BAUD* = 50
  PI_BB_SER_MAX_BAUD* = 250000
  PI_BB_SER_NORMAL* = 0
  PI_BB_SER_INVERT* = 1
  PI_WAVE_MIN_BAUD* = 50
  PI_WAVE_MAX_BAUD* = 1000000
  PI_SPI_MIN_BAUD* = 32000
  PI_SPI_MAX_BAUD* = 125000000
  PI_MIN_WAVE_DATABITS* = 1
  PI_MAX_WAVE_DATABITS* = 32
  PI_MIN_WAVE_HALFSTOPBITS* = 2
  PI_MAX_WAVE_HALFSTOPBITS* = 8
  PI_WAVE_MAX_MICROS* = (30 * 60 * 1000000) ##  half an hour
  PI_MAX_WAVES* = 250
  PI_MAX_WAVE_CYCLES* = 65535
  PI_MAX_WAVE_DELAY* = 65535
  PI_WAVE_COUNT_PAGES* = 10

##  wave tx mode

const
  PI_WAVE_MODE_ONE_SHOT*: cuint = 0
  PI_WAVE_MODE_REPEAT*: cuint = 1
  PI_WAVE_MODE_ONE_SHOT_SYNC*: cuint = 2
  PI_WAVE_MODE_REPEAT_SYNC*: cuint = 3

##  special wave at return values

const
  PI_WAVE_NOT_FOUND* = 9998
  PI_NO_TX_WAVE* = 9999

##  Files, I2C, SPI, SER

const
  PI_FILE_SLOTS* = 16
  PI_I2C_SLOTS* = 512
  PI_SPI_SLOTS* = 32
  PI_SER_SLOTS* = 16
  PI_MAX_I2C_ADDR* = 0x7F
  PI_NUM_AUX_SPI_CHANNEL* = 3
  PI_NUM_STD_SPI_CHANNEL* = 2
  PI_MAX_I2C_DEVICE_COUNT* = (1 shl 16)
  PI_MAX_SPI_DEVICE_COUNT* = (1 shl 16)

##  max pi_i2c_msg_t per transaction

const
  PI_I2C_RDRW_IOCTL_MAX_MSGS* = 42

##  flags for i2cTransaction, pi_i2c_msg_t

const
  PI_I2C_M_WR* = 0x0000
  PI_I2C_M_RD* = 0x0001
  PI_I2C_M_TEN* = 0x0010
  PI_I2C_M_RECV_LEN* = 0x0400
  PI_I2C_M_NO_RD_ACK* = 0x0800
  PI_I2C_M_IGNORE_NAK* = 0x1000
  PI_I2C_M_REV_DIR_ADDR* = 0x2000
  PI_I2C_M_NOSTART* = 0x4000

##  bbI2CZip and i2cZip commands

const
  PI_I2C_END* = 0
  PI_I2C_ESC* = 1
  PI_I2C_START* = 2
  PI_I2C_COMBINED_ON* = 2
  PI_I2C_STOP* = 3
  PI_I2C_COMBINED_OFF* = 3
  PI_I2C_ADDR* = 4
  PI_I2C_FLAGS* = 5
  PI_I2C_READ* = 6
  PI_I2C_WRITE* = 7

##  SPI

template PI_SPI_FLAGS_BITLEN*(x: untyped): untyped =
  ((x and 63) shl 16)

template PI_SPI_FLAGS_RX_LSB*(x: untyped): untyped =
  ((x and 1) shl 15)

template PI_SPI_FLAGS_TX_LSB*(x: untyped): untyped =
  ((x and 1) shl 14)

template PI_SPI_FLAGS_3WREN*(x: untyped): untyped =
  ((x and 15) shl 10)

template PI_SPI_FLAGS_3WIRE*(x: untyped): untyped =
  ((x and 1) shl 9)

template PI_SPI_FLAGS_AUX_SPI*(x: untyped): untyped =
  ((x and 1) shl 8)

template PI_SPI_FLAGS_RESVD*(x: untyped): untyped =
  ((x and 7) shl 5)

template PI_SPI_FLAGS_CSPOLS*(x: untyped): untyped =
  ((x and 7) shl 2)

template PI_SPI_FLAGS_MODE*(x: untyped): untyped =
  ((x and 3))

##  BSC registers

const
  BSC_DR* = 0
  BSC_RSR* = 1
  BSC_SLV* = 2
  BSC_CR* = 3
  BSC_FR* = 4
  BSC_IFLS* = 5
  BSC_IMSC* = 6
  BSC_RIS* = 7
  BSC_MIS* = 8
  BSC_ICR* = 9
  BSC_DMACR* = 10
  BSC_TDR* = 11
  BSC_GPUSTAT* = 12
  BSC_HCTRL* = 13
  BSC_DEBUG_I2C* = 14
  BSC_DEBUG_SPI* = 15
  BSC_CR_TESTFIFO* = 2048
  BSC_CR_RXE* = 512
  BSC_CR_TXE* = 256
  BSC_CR_BRK* = 128
  BSC_CR_CPOL* = 16
  BSC_CR_CPHA* = 8
  BSC_CR_I2C* = 4
  BSC_CR_SPI* = 2
  BSC_CR_EN* = 1
  BSC_FR_RXBUSY* = 32
  BSC_FR_TXFE* = 16
  BSC_FR_RXFF* = 8
  BSC_FR_TXFF* = 4
  BSC_FR_RXFE* = 2
  BSC_FR_TXBUSY* = 1

##  BSC GPIO

const
  BSC_SDA* = 18
  BSC_MOSI* = 20
  BSC_SCL_SCLK* = 19
  BSC_MISO* = 18
  BSC_CE_N* = 21
  BSC_SDA_2711* = 10
  BSC_MOSI_2711* = 9
  BSC_SCL_SCLK_2711* = 11
  BSC_MISO_2711* = 10
  BSC_CE_N_2711* = 8

##  Longest busy delay

const
  PI_MAX_BUSY_DELAY* = 100

##  timeout: 0-60000

const
  PI_MIN_WDOG_TIMEOUT* = 0
  PI_MAX_WDOG_TIMEOUT* = 60000

##  timer: 0-9

const
  PI_MIN_TIMER* = 0
  PI_MAX_TIMER* = 9

##  millis: 10-60000

const
  PI_MIN_MS* = 10
  PI_MAX_MS* = 60000
  PI_MAX_SCRIPTS* = 32
  PI_MAX_SCRIPT_TAGS* = 50
  PI_MAX_SCRIPT_VARS* = 150
  PI_MAX_SCRIPT_PARAMS* = 10

##  script status

const
  PI_SCRIPT_INITING* = 0
  PI_SCRIPT_HALTED* = 1
  PI_SCRIPT_RUNNING* = 2
  PI_SCRIPT_WAITING* = 3
  PI_SCRIPT_FAILED* = 4

##  signum: 0-63

const
  PI_MIN_SIGNUM* = 0
  PI_MAX_SIGNUM* = 63

##  timetype: 0-1

const
  PI_TIME_RELATIVE* = 0
  PI_TIME_ABSOLUTE* = 1
  PI_MAX_MICS_DELAY* = 1000000
  PI_MAX_MILS_DELAY* = 60000

##  cfgMillis

const
  PI_BUF_MILLIS_MIN* = 100
  PI_BUF_MILLIS_MAX* = 10000

##  cfgMicros: 1, 2, 4, 5, 8, or 10
##  cfgPeripheral: 0-1

const
  PI_CLOCK_PWM* = 0
  PI_CLOCK_PCM* = 1

##  DMA channel: 0-15, 15 is unset

const
  PI_MIN_DMA_CHANNEL* = 0
  PI_MAX_DMA_CHANNEL* = 15

##  port

const
  PI_MIN_SOCKET_PORT* = 1024
  PI_MAX_SOCKET_PORT* = 32000

##  ifFlags:

const
  PI_DISABLE_FIFO_IF* = 1
  PI_DISABLE_SOCK_IF* = 2
  PI_LOCALHOST_SOCK_IF* = 4
  PI_DISABLE_ALERT* = 8

##  memAllocMode

const
  PI_MEM_ALLOC_AUTO* = 0
  PI_MEM_ALLOC_PAGEMAP* = 1
  PI_MEM_ALLOC_MAILBOX* = 2

##  filters

const
  PI_MAX_STEADY* = 300000
  PI_MAX_ACTIVE* = 1000000

##  gpioCfgInternals

const
  PI_CFG_DBG_LEVEL* = 0
  PI_CFG_ALERT_FREQ* = 4
  PI_CFG_RT_PRIORITY* = (1 shl 8)
  PI_CFG_STATS* = (1 shl 9)
  PI_CFG_NOSIGHANDLER* = (1 shl 10)
  PI_CFG_ILLEGAL_VAL* = (1 shl 11)

##  gpioISR

const
  RISING_EDGE* = 0
  FALLING_EDGE* = 1
  EITHER_EDGE* = 2

##  pads

const
  PI_MAX_PAD* = 2
  PI_MIN_PAD_STRENGTH* = 1
  PI_MAX_PAD_STRENGTH* = 16

##  files

const
  PI_FILE_NONE* = 0
  PI_FILE_MIN* = 1
  PI_FILE_READ* = 1
  PI_FILE_WRITE* = 2
  PI_FILE_RW* = 3
  PI_FILE_APPEND* = 4
  PI_FILE_CREATE* = 8
  PI_FILE_TRUNC* = 16
  PI_FILE_MAX* = 31
  PI_FROM_START* = 0
  PI_FROM_CURRENT* = 1
  PI_FROM_END* = 2

##  Allowed socket connect addresses

const
  MAX_CONNECT_ADDRESSES* = 256

##  events

const
  PI_MAX_EVENT* = 31

##  Event auto generated on BSC slave activity

const
  PI_EVENT_BSC* = 31

proc gpioInitialise*(): cint {.importc.}
proc gpioTerminate*() {.importc.}

proc gpioSetMode*(gpio: cuint; mode: cuint): cint {.importc.}
proc gpioGetMode*(gpio: cuint): cint {.importc.}

proc gpioSetPullUpDown*(gpio: cuint; pud: cuint): cint {.importc.}

proc gpioRead*(gpio: cuint): cint {.importc.}
proc gpioWrite*(gpio: cuint; level: cuint): cint {.importc.}

proc gpioPWM*(user_gpio: cuint; dutycycle: cuint): cint {.importc.}
proc gpioGetPWMdutycycle*(user_gpio: cuint): cint {.importc.}
proc gpioSetPWMrange*(user_gpio: cuint; range: cuint): cint {.importc.}
proc gpioGetPWMrange*(user_gpio: cuint): cint {.importc.}
proc gpioGetPWMrealRange*(user_gpio: cuint): cint {.importc.}
proc gpioSetPWMfrequency*(user_gpio: cuint; frequency: cuint): cint {.importc.}
proc gpioGetPWMfrequency*(user_gpio: cuint): cint {.importc.}

proc gpioServo*(user_gpio: cuint; pulsewidth: cuint): cint {.importc.}
proc gpioGetServoPulsewidth*(user_gpio: cuint): cint {.importc.}

proc gpioSetAlertFunc*(user_gpio: cuint; f: gpioAlertFunc_t): cint {.importc.}
proc gpioSetAlertFuncEx*(user_gpio: cuint; f: gpioAlertFuncEx_t; userdata: pointer): cint {.importc.}
proc gpioSetISRFunc*(gpio: cuint; edge: cuint; timeout: cint; f: gpioISRFunc_t): cint {.importc.}

proc gpioSetISRFuncEx*(gpio: cuint; edge: cuint; timeout: cint; f: gpioISRFuncEx_t;
                      userdata: pointer): cint {.importc.}

proc gpioNotifyOpen*(): cint {.importc.}
proc gpioNotifyOpenWithSize*(bufSize: cint): cint {.importc.}
proc gpioNotifyBegin*(handle: cuint; bits: uint32_t): cint {.importc.}
proc gpioNotifyPause*(handle: cuint): cint {.importc.}
proc gpioNotifyClose*(handle: cuint): cint {.importc.}

proc gpioWaveClear*(): cint {.importc.}
proc gpioWaveAddNew*(): cint {.importc.}
proc gpioWaveAddGeneric*(numPulses: cuint; pulses: ptr gpioPulse_t): cint {.importc.}
proc gpioWaveAddSerial*(user_gpio: cuint; baud: cuint; data_bits: cuint; stop_bits: cuint;
                       offset: cuint; numBytes: cuint; str: cstring): cint {.importc.}
proc gpioWaveCreate*(): cint {.importc.}
proc gpioWaveCreatePad*(pctCB: cint; pctBOOL: cint; pctTOOL: cint): cint {.importc.}
proc gpioWaveDelete*(wave_id: cuint): cint {.importc.}
proc gpioWaveTxSend*(wave_id: cuint; wave_mode: cuint): cint {.importc.}
proc gpioWaveChain*(buf: cstring; bufSize: cuint): cint {.importc.}
proc gpioWaveTxAt*(): cint {.importc.}
proc gpioWaveTxBusy*(): cint {.importc.}
proc gpioWaveTxStop*(): cint {.importc.}
proc gpioWaveGetMicros*(): cint {.importc.}
proc gpioWaveGetHighMicros*(): cint {.importc.}
proc gpioWaveGetMaxMicros*(): cint {.importc.}
proc gpioWaveGetPulses*(): cint {.importc.}
proc gpioWaveGetHighPulses*(): cint {.importc.}
proc gpioWaveGetMaxPulses*(): cint {.importc.}
proc gpioWaveGetCbs*(): cint {.importc.}
proc gpioWaveGetHighCbs*(): cint {.importc.}
proc gpioWaveGetMaxCbs*(): cint {.importc.}

proc gpioSerialReadOpen*(user_gpio: cuint; baud: cuint; data_bits: cuint): cint {.importc.}
proc gpioSerialReadInvert*(user_gpio: cuint; invert: cuint): cint {.importc.}
proc gpioSerialRead*(user_gpio: cuint; buf: pointer; bufSize: csize_t): cint {.importc.}
proc gpioSerialReadClose*(user_gpio: cuint): cint {.importc.}

proc i2cOpen*(i2cBus: cuint; i2cAddr: cuint; i2cFlags: cuint): cint {.importc.}
proc i2cClose*(handle: cuint): cint {.importc.}
proc i2cWriteQuick*(handle: cuint; bit: cuint): cint {.importc.}
proc i2cWriteByte*(handle: cuint; bVal: cuint): cint {.importc.}
proc i2cReadByte*(handle: cuint): cint {.importc.}
proc i2cWriteByteData*(handle: cuint; i2cReg: cuint; bVal: cuint): cint {.importc.}
proc i2cWriteWordData*(handle: cuint; i2cReg: cuint; wVal: cuint): cint {.importc.}
proc i2cReadByteData*(handle: cuint; i2cReg: cuint): cint {.importc.}
proc i2cReadWordData*(handle: cuint; i2cReg: cuint): cint {.importc.}
proc i2cProcessCall*(handle: cuint; i2cReg: cuint; wVal: cuint): cint {.importc.}
proc i2cWriteBlockData*(handle: cuint; i2cReg: cuint; buf: cstring; count: cuint): cint {.importc.}
proc i2cReadBlockData*(handle: cuint; i2cReg: cuint; buf: cstring): cint {.importc.}
proc i2cBlockProcessCall*(handle: cuint; i2cReg: cuint; buf: cstring; count: cuint): cint {.importc.}
proc i2cReadI2CBlockData*(handle: cuint; i2cReg: cuint; buf: cstring; count: cuint): cint {.importc.}
proc i2cWriteI2CBlockData*(handle: cuint; i2cReg: cuint; buf: cstring; count: cuint): cint {.importc.}
proc i2cReadDevice*(handle: cuint; buf: cstring; count: cuint): cint {.importc.}
proc i2cWriteDevice*(handle: cuint; buf: cstring; count: cuint): cint {.importc.}
proc i2cSwitchCombined*(setting: cint) {.importc.}
proc i2cSegments*(handle: cuint; segs: ptr pi_i2c_msg_t; numSegs: cuint): cint {.importc.}
proc i2cZip*(handle: cuint; inBuf: cstring; inLen: cuint; outBuf: cstring; outLen: cuint): cint {.importc.}

proc bbI2COpen*(SDA: cuint; SCL: cuint; baud: cuint): cint {.importc.}
proc bbI2CClose*(SDA: cuint): cint {.importc.}
proc bbI2CZip*(SDA: cuint; inBuf: cstring; inLen: cuint; outBuf: cstring; outLen: cuint): cint {.importc.}

proc bscXfer*(bsc_xfer: ptr bsc_xfer_t): cint {.importc.}

proc bbSPIOpen*(CS: cuint; MISO: cuint; MOSI: cuint; SCLK: cuint; baud: cuint;
               spiFlags: cuint): cint {.importc.}
proc bbSPIClose*(CS: cuint): cint {.importc.}
proc bbSPIXfer*(CS: cuint; inBuf: cstring; outBuf: cstring; count: cuint): cint {.importc.}

proc spiOpen*(spiChan: cuint; baud: cuint; spiFlags: cuint): cint {.importc.}
proc spiClose*(handle: cuint): cint {.importc.}
proc spiRead*(handle: cuint; buf: cstring; count: cuint): cint {.importc.}
proc spiWrite*(handle: cuint; buf: cstring; count: cuint): cint {.importc.}
proc spiXfer*(handle: cuint; txBuf: cstring; rxBuf: cstring; count: cuint): cint {.importc.}

proc serOpen*(sertty: cstring; baud: cuint; serFlags: cuint): cint {.importc.}
proc serClose*(handle: cuint): cint {.importc.}
proc serWriteByte*(handle: cuint; bVal: cuint): cint {.importc.}
proc serReadByte*(handle: cuint): cint {.importc.}
proc serWrite*(handle: cuint; buf: cstring; count: cuint): cint {.importc.}
proc serRead*(handle: cuint; buf: cstring; count: cuint): cint {.importc.}
proc serDataAvailable*(handle: cuint): cint {.importc.}

proc gpioTrigger*(user_gpio: cuint; pulseLen: cuint; level: cuint): cint {.importc.}

proc gpioSetWatchdog*(user_gpio: cuint; timeout: cuint): cint {.importc.}

proc gpioNoiseFilter*(user_gpio: cuint; steady: cuint; active: cuint): cint {.importc.}

proc gpioGlitchFilter*(user_gpio: cuint; steady: cuint): cint {.importc.}

proc gpioSetGetSamplesFunc*(f: gpioGetSamplesFunc_t; bits: uint32_t): cint {.importc.}
proc gpioSetGetSamplesFuncEx*(f: gpioGetSamplesFuncEx_t; bits: uint32_t;
                             userdata: pointer): cint {.importc.}
proc gpioSetTimerFunc*(timer: cuint; millis: cuint; f: gpioTimerFunc_t): cint {.importc.}
proc gpioSetTimerFuncEx*(timer: cuint; millis: cuint; f: gpioTimerFuncEx_t;
                        userdata: pointer): cint {.importc.}

#proc gpioStartThread*(f: gpioThreadFunc_t; userdata: pointer): ptr pthread_t {.importc.}
#proc gpioStopThread*(pth: ptr pthread_t) {.importc.}

proc gpioStoreScript*(script: cstring): cint {.importc.}
proc gpioRunScript*(script_id: cuint; numPar: cuint; param: ptr uint32_t): cint {.importc.}
proc gpioUpdateScript*(script_id: cuint; numPar: cuint; param: ptr uint32_t): cint {.importc.}
proc gpioScriptStatus*(script_id: cuint; param: ptr uint32_t): cint {.importc.}
proc gpioStopScript*(script_id: cuint): cint {.importc.}
proc gpioDeleteScript*(script_id: cuint): cint {.importc.}

proc gpioSetSignalFunc*(signum: cuint; f: gpioSignalFunc_t): cint {.importc.}
proc gpioSetSignalFuncEx*(signum: cuint; f: gpioSignalFuncEx_t; userdata: pointer): cint {.importc.}

proc gpioRead_Bits_0_31*(): uint32_t {.importc.}
proc gpioRead_Bits_32_53*(): uint32_t {.importc.}
proc gpioWrite_Bits_0_31_Clear*(bits: uint32_t): cint {.importc.}
proc gpioWrite_Bits_32_53_Clear*(bits: uint32_t): cint {.importc.}
proc gpioWrite_Bits_0_31_Set*(bits: uint32_t): cint {.importc.}
proc gpioWrite_Bits_32_53_Set*(bits: uint32_t): cint {.importc.}

proc gpioHardwareClock*(gpio: cuint; clkfreq: cuint): cint {.importc.}
proc gpioHardwarePWM*(gpio: cuint; PWMfreq: cuint; PWMduty: cuint): cint {.importc.}

proc gpioTime*(timetype: cuint; seconds: ptr cint; micros: ptr cint): cint {.importc.}
proc gpioSleep*(timetype: cuint; seconds: cint; micros: cint): cint {.importc.}
proc gpioDelay*(micros: uint32_t): uint32_t {.importc.}
proc gpioTick*(): uint32_t {.importc.}

proc gpioHardwareRevision*(): cuint {.importc.}

proc gpioVersion*(): cuint {.importc.}

proc gpioGetPad*(pad: cuint): cint {.importc.}
proc gpioSetPad*(pad: cuint; padStrength: cuint): cint {.importc.}

proc eventMonitor*(handle: cuint; bits: uint32_t): cint {.importc.}
proc eventSetFunc*(event: cuint; f: eventFunc_t): cint {.importc.}
proc eventSetFuncEx*(event: cuint; f: eventFuncEx_t; userdata: pointer): cint {.importc.}
proc eventTrigger*(event: cuint): cint {.importc.}

proc shell*(scriptName: cstring; scriptString: cstring): cint {.importc.}

proc fileOpen*(file: cstring; mode: cuint): cint {.importc.}
proc fileClose*(handle: cuint): cint {.importc.}
proc fileWrite*(handle: cuint; buf: cstring; count: cuint): cint {.importc.}
proc fileRead*(handle: cuint; buf: cstring; count: cuint): cint {.importc.}
proc fileSeek*(handle: cuint; seekOffset: int32_t; seekFrom: cint): cint {.importc.}
proc fileList*(fpat: cstring; buf: cstring; count: cuint): cint {.importc.}

proc gpioCfgBufferSize*(cfgMillis: cuint): cint {.importc.}
proc gpioCfgClock*(cfgMicros: cuint; cfgPeripheral: cuint; cfgSource: cuint): cint {.importc.}
proc gpioCfgDMAchannel*(DMAchannel: cuint): cint {.importc.}
proc gpioCfgDMAchannels*(primaryChannel: cuint; secondaryChannel: cuint): cint {.importc.}
proc gpioCfgPermissions*(updateMask: uint64_t): cint {.importc.}
proc gpioCfgSocketPort*(port: cuint): cint {.importc.}
proc gpioCfgInterfaces*(ifFlags: cuint): cint {.importc.}
proc gpioCfgMemAlloc*(memAllocMode: cuint): cint {.importc.}
proc gpioCfgNetAddr*(numSockAddr: cint; sockAddr: ptr uint32_t): cint {.importc.}
proc gpioCfgGetInternals*(): uint32_t {.importc.}
proc gpioCfgSetInternals*(cfgVal: uint32_t): cint {.importc.}

proc gpioCustom1*(arg1: cuint; arg2: cuint; argx: cstring; argc: cuint): cint {.importc.}
proc gpioCustom2*(arg1: cuint; argx: cstring; argc: cuint; retBuf: cstring; retMax: cuint): cint {.importc.}

proc rawWaveAddSPI*(spi: ptr rawSPI_t; offset: cuint; spiSS: cuint; buf: cstring;
                   spiTxBits: cuint; spiBitFirst: cuint; spiBitLast: cuint;
                   spiBits: cuint): cint {.importc.}
proc rawWaveAddGeneric*(numPulses: cuint; pulses: ptr rawWave_t): cint {.importc.}
proc rawWaveCB*(): cuint {.importc.}
proc rawWaveCBAdr*(cbNum: cint): ptr rawCbs_t {.importc.}
proc rawWaveGetOOL*(pos: cint): uint32_t {.importc.}
proc rawWaveSetOOL*(pos: cint; lVal: uint32_t) {.importc.}
proc rawWaveGetOut*(pos: cint): uint32_t {.importc.}
proc rawWaveSetOut*(pos: cint; lVal: uint32_t) {.importc.}
proc rawWaveGetIn*(pos: cint): uint32_t {.importc.}
proc rawWaveSetIn*(pos: cint; lVal: uint32_t) {.importc.}
proc rawWaveInfo*(wave_id: cint): rawWaveInfo_t {.importc.}

proc getBitInBytes*(bitPos: cint; buf: cstring; numBits: cint): cint {.importc.}
proc putBitInBytes*(bitPos: cint; buf: cstring; bit: cint) {.importc.}

proc time_time*(): cdouble {.importc.}
proc time_sleep*(seconds: cdouble) {.importc.}

proc rawDumpWave*() {.importc.}
proc rawDumpScript*(script_id: cuint) {.importc.}

## DEF_S Socket Command Codes

const
  PI_CMD_MODES* = 0
  PI_CMD_MODEG* = 1
  PI_CMD_PUD* = 2
  PI_CMD_READ* = 3
  PI_CMD_WRITE* = 4
  PI_CMD_PWM* = 5
  PI_CMD_PRS* = 6
  PI_CMD_PFS* = 7
  PI_CMD_SERVO* = 8
  PI_CMD_WDOG* = 9
  PI_CMD_BR1* = 10
  PI_CMD_BR2* = 11
  PI_CMD_BC1* = 12
  PI_CMD_BC2* = 13
  PI_CMD_BS1* = 14
  PI_CMD_BS2* = 15
  PI_CMD_TICK* = 16
  PI_CMD_HWVER* = 17
  PI_CMD_NO* = 18
  PI_CMD_NB* = 19
  PI_CMD_NP* = 20
  PI_CMD_NC* = 21
  PI_CMD_PRG* = 22
  PI_CMD_PFG* = 23
  PI_CMD_PRRG* = 24
  PI_CMD_HELP* = 25
  PI_CMD_PIGPV* = 26
  PI_CMD_WVCLR* = 27
  PI_CMD_WVAG* = 28
  PI_CMD_WVAS* = 29
  PI_CMD_WVGO* = 30
  PI_CMD_WVGOR* = 31
  PI_CMD_WVBSY* = 32
  PI_CMD_WVHLT* = 33
  PI_CMD_WVSM* = 34
  PI_CMD_WVSP* = 35
  PI_CMD_WVSC* = 36
  PI_CMD_TRIG* = 37
  PI_CMD_PROC* = 38
  PI_CMD_PROCD* = 39
  PI_CMD_PROCR* = 40
  PI_CMD_PROCS* = 41
  PI_CMD_SLRO* = 42
  PI_CMD_SLR* = 43
  PI_CMD_SLRC* = 44
  PI_CMD_PROCP* = 45
  PI_CMD_MICS* = 46
  PI_CMD_MILS* = 47
  PI_CMD_PARSE* = 48
  PI_CMD_WVCRE* = 49
  PI_CMD_WVDEL* = 50
  PI_CMD_WVTX* = 51
  PI_CMD_WVTXR* = 52
  PI_CMD_WVNEW* = 53
  PI_CMD_I2CO* = 54
  PI_CMD_I2CC* = 55
  PI_CMD_I2CRD* = 56
  PI_CMD_I2CWD* = 57
  PI_CMD_I2CWQ* = 58
  PI_CMD_I2CRS* = 59
  PI_CMD_I2CWS* = 60
  PI_CMD_I2CRB* = 61
  PI_CMD_I2CWB* = 62
  PI_CMD_I2CRW* = 63
  PI_CMD_I2CWW* = 64
  PI_CMD_I2CRK* = 65
  PI_CMD_I2CWK* = 66
  PI_CMD_I2CRI* = 67
  PI_CMD_I2CWI* = 68
  PI_CMD_I2CPC* = 69
  PI_CMD_I2CPK* = 70
  PI_CMD_SPIO* = 71
  PI_CMD_SPIC* = 72
  PI_CMD_SPIR* = 73
  PI_CMD_SPIW* = 74
  PI_CMD_SPIX* = 75
  PI_CMD_SERO* = 76
  PI_CMD_SERC* = 77
  PI_CMD_SERRB* = 78
  PI_CMD_SERWB* = 79
  PI_CMD_SERR* = 80
  PI_CMD_SERW* = 81
  PI_CMD_SERDA* = 82
  PI_CMD_GDC* = 83
  PI_CMD_GPW* = 84
  PI_CMD_HC* = 85
  PI_CMD_HP* = 86
  PI_CMD_CF1* = 87
  PI_CMD_CF2* = 88
  PI_CMD_BI2CC* = 89
  PI_CMD_BI2CO* = 90
  PI_CMD_BI2CZ* = 91
  PI_CMD_I2CZ* = 92
  PI_CMD_WVCHA* = 93
  PI_CMD_SLRI* = 94
  PI_CMD_CGI* = 95
  PI_CMD_CSI* = 96
  PI_CMD_FG* = 97
  PI_CMD_FN* = 98
  PI_CMD_NOIB* = 99
  PI_CMD_WVTXM* = 100
  PI_CMD_WVTAT* = 101
  PI_CMD_PADS* = 102
  PI_CMD_PADG* = 103
  PI_CMD_FO* = 104
  PI_CMD_FC* = 105
  PI_CMD_FR* = 106
  PI_CMD_FW* = 107
  PI_CMD_FS* = 108
  PI_CMD_FL* = 109
  PI_CMD_SHELL* = 110
  PI_CMD_BSPIC* = 111
  PI_CMD_BSPIO* = 112
  PI_CMD_BSPIX* = 113
  PI_CMD_BSCX* = 114
  PI_CMD_EVM* = 115
  PI_CMD_EVT* = 116
  PI_CMD_PROCU* = 117
  PI_CMD_WVCAP* = 118

## DEF_E

#[
  PI CMD_NOIB only works on the socket interface.
  It returns a spare notification handle.  Notifications for
  that handle will be sent to the socket (rather than a
  /dev/pigpiox pipe).

  The socket should be dedicated to receiving notifications
  after this command is issued.
]#

##  pseudo commands

const
  PI_CMD_SCRIPT* = 800
  PI_CMD_ADD* = 800
  PI_CMD_AND* = 801
  PI_CMD_CALL* = 802
  PI_CMD_CMDR* = 803
  PI_CMD_CMDW* = 804
  PI_CMD_CMP* = 805
  PI_CMD_DCR* = 806
  PI_CMD_DCRA* = 807
  PI_CMD_DIV* = 808
  PI_CMD_HALT* = 809
  PI_CMD_INR* = 810
  PI_CMD_INRA* = 811
  PI_CMD_JM* = 812
  PI_CMD_JMP* = 813
  PI_CMD_JNZ* = 814
  PI_CMD_JP* = 815
  PI_CMD_JZ* = 816
  PI_CMD_TAG* = 817
  PI_CMD_LD* = 818
  PI_CMD_LDA* = 819
  PI_CMD_LDAB* = 820
  PI_CMD_MLT* = 821
  PI_CMD_MOD* = 822
  PI_CMD_NOP* = 823
  PI_CMD_OR* = 824
  PI_CMD_POP* = 825
  PI_CMD_POPA* = 826
  PI_CMD_PUSH* = 827
  PI_CMD_PUSHA* = 828
  PI_CMD_RET* = 829
  PI_CMD_RL* = 830
  PI_CMD_RLA* = 831
  PI_CMD_RR* = 832
  PI_CMD_RRA* = 833
  PI_CMD_STA* = 834
  PI_CMD_STAB* = 835
  PI_CMD_SUB* = 836
  PI_CMD_SYS* = 837
  PI_CMD_WAIT* = 838
  PI_CMD_X* = 839
  PI_CMD_XA* = 840
  PI_CMD_XOR* = 841
  PI_CMD_EVTWT* = 842

## DEF_S Error Codes

const
  PI_INIT_FAILED* = -1
  PI_BAD_USER_GPIO* = -2
  PI_BAD_GPIO* = -3
  PI_BAD_MODE* = -4
  PI_BAD_LEVEL* = -5
  PI_BAD_PUD* = -6
  PI_BAD_PULSEWIDTH* = -7
  PI_BAD_DUTYCYCLE* = -8
  PI_BAD_TIMER* = -9
  PI_BAD_MS* = -10
  PI_BAD_TIMETYPE* = -11
  PI_BAD_SECONDS* = -12
  PI_BAD_MICROS* = -13
  PI_TIMER_FAILED* = -14
  PI_BAD_WDOG_TIMEOUT* = -15
  PI_NO_ALERT_FUNC* = -16
  PI_BAD_CLK_PERIPH* = -17
  PI_BAD_CLK_SOURCE* = -18
  PI_BAD_CLK_MICROS* = -19
  PI_BAD_BUF_MILLIS* = -20
  PI_BAD_DUTYRANGE* = -21
  PI_BAD_SIGNUM* = -22
  PI_BAD_PATHNAME* = -23
  PI_NO_HANDLE* = -24
  PI_BAD_HANDLE* = -25
  PI_BAD_IF_FLAGS* = -26
  PI_BAD_CHANNEL* = -27
  PI_BAD_PRIM_CHANNEL* = -27
  PI_BAD_SOCKET_PORT* = -28
  PI_BAD_FIFO_COMMAND* = -29
  PI_BAD_SECO_CHANNEL* = -30
  PI_NOT_INITIALISED* = -31
  PI_INITIALISED* = -32
  PI_BAD_WAVE_MODE* = -33
  PI_BAD_CFG_INTERNAL* = -34
  PI_BAD_WAVE_BAUD* = -35
  PI_TOO_MANY_PULSES* = -36
  PI_TOO_MANY_CHARS* = -37
  PI_NOT_SERIAL_GPIO* = -38
  PI_BAD_SERIAL_STRUC* = -39
  PI_BAD_SERIAL_BUF* = -40
  PI_NOT_PERMITTED* = -41
  PI_SOME_PERMITTED* = -42
  PI_BAD_WVSC_COMMND* = -43
  PI_BAD_WVSM_COMMND* = -44
  PI_BAD_WVSP_COMMND* = -45
  PI_BAD_PULSELEN* = -46
  PI_BAD_SCRIPT* = -47
  PI_BAD_SCRIPT_ID* = -48
  PI_BAD_SER_OFFSET* = -49
  PI_GPIO_IN_USE* = -50
  PI_BAD_SERIAL_COUNT* = -51
  PI_BAD_PARAM_NUM* = -52
  PI_DUP_TAG* = -53
  PI_TOO_MANY_TAGS* = -54
  PI_BAD_SCRIPT_CMD* = -55
  PI_BAD_VAR_NUM* = -56
  PI_NO_SCRIPT_ROOM* = -57
  PI_NO_MEMORY* = -58
  PI_SOCK_READ_FAILED* = -59
  PI_SOCK_WRIT_FAILED* = -60
  PI_TOO_MANY_PARAM* = -61
  PI_NOT_HALTED* = -62
  PI_SCRIPT_NOT_READY* = -62
  PI_BAD_TAG* = -63
  PI_BAD_MICS_DELAY* = -64
  PI_BAD_MILS_DELAY* = -65
  PI_BAD_WAVE_ID* = -66
  PI_TOO_MANY_CBS* = -67
  PI_TOO_MANY_OOL* = -68
  PI_EMPTY_WAVEFORM* = -69
  PI_NO_WAVEFORM_ID* = -70
  PI_I2C_OPEN_FAILED* = -71
  PI_SER_OPEN_FAILED* = -72
  PI_SPI_OPEN_FAILED* = -73
  PI_BAD_I2C_BUS* = -74
  PI_BAD_I2C_ADDR* = -75
  PI_BAD_SPI_CHANNEL* = -76
  PI_BAD_FLAGS* = -77
  PI_BAD_SPI_SPEED* = -78
  PI_BAD_SER_DEVICE* = -79
  PI_BAD_SER_SPEED* = -80
  PI_BAD_PARAM* = -81
  PI_I2C_WRITE_FAILED* = -82
  PI_I2C_READ_FAILED* = -83
  PI_BAD_SPI_COUNT* = -84
  PI_SER_WRITE_FAILED* = -85
  PI_SER_READ_FAILED* = -86
  PI_SER_READ_NO_DATA* = -87
  PI_UNKNOWN_COMMAND* = -88
  PI_SPI_XFER_FAILED* = -89
  PI_BAD_POINTER* = -90
  PI_NO_AUX_SPI* = -91
  PI_NOT_PWM_GPIO* = -92
  PI_NOT_SERVO_GPIO* = -93
  PI_NOT_HCLK_GPIO* = -94
  PI_NOT_HPWM_GPIO* = -95
  PI_BAD_HPWM_FREQ* = -96
  PI_BAD_HPWM_DUTY* = -97
  PI_BAD_HCLK_FREQ* = -98
  PI_BAD_HCLK_PASS* = -99
  PI_HPWM_ILLEGAL* = -100
  PI_BAD_DATABITS* = -101
  PI_BAD_STOPBITS* = -102
  PI_MSG_TOOBIG* = -103
  PI_BAD_MALLOC_MODE* = -104
  PI_TOO_MANY_SEGS* = -105
  PI_BAD_I2C_SEG* = -106
  PI_BAD_SMBUS_CMD* = -107
  PI_NOT_I2C_GPIO* = -108
  PI_BAD_I2C_WLEN* = -109
  PI_BAD_I2C_RLEN* = -110
  PI_BAD_I2C_CMD* = -111
  PI_BAD_I2C_BAUD* = -112
  PI_CHAIN_LOOP_CNT* = -113
  PI_BAD_CHAIN_LOOP* = -114
  PI_CHAIN_COUNTER* = -115
  PI_BAD_CHAIN_CMD* = -116
  PI_BAD_CHAIN_DELAY* = -117
  PI_CHAIN_NESTING* = -118
  PI_CHAIN_TOO_BIG* = -119
  PI_DEPRECATED* = -120
  PI_BAD_SER_INVERT* = -121
  PI_BAD_EDGE* = -122
  PI_BAD_ISR_INIT* = -123
  PI_BAD_FOREVER* = -124
  PI_BAD_FILTER* = -125
  PI_BAD_PAD* = -126
  PI_BAD_STRENGTH* = -127
  PI_FIL_OPEN_FAILED* = -128
  PI_BAD_FILE_MODE* = -129
  PI_BAD_FILE_FLAG* = -130
  PI_BAD_FILE_READ* = -131
  PI_BAD_FILE_WRITE* = -132
  PI_FILE_NOT_ROPEN* = -133
  PI_FILE_NOT_WOPEN* = -134
  PI_BAD_FILE_SEEK* = -135
  PI_NO_FILE_MATCH* = -136
  PI_NO_FILE_ACCESS* = -137
  PI_FILE_IS_A_DIR* = -138
  PI_BAD_SHELL_STATUS* = -139
  PI_BAD_SCRIPT_NAME* = -140
  PI_BAD_SPI_BAUD* = -141
  PI_NOT_SPI_GPIO* = -142
  PI_BAD_EVENT_ID* = -143
  PI_CMD_INTERRUPTED* = -144
  PI_NOT_ON_BCM2711* = -145
  PI_ONLY_ON_BCM2711* = -146
  PI_PIGIF_ERR_0* = -2000
  PI_PIGIF_ERR_99* = -2099
  PI_CUSTOM_ERR_0* = -3000
  PI_CUSTOM_ERR_999* = -3999

## DEF_E

## DEF_S Defaults

const
  PI_DEFAULT_BUFFER_MILLIS* = 120
  PI_DEFAULT_CLK_MICROS* = 5
  PI_DEFAULT_CLK_PERIPHERAL* = PI_CLOCK_PCM
  PI_DEFAULT_IF_FLAGS* = 0
  PI_DEFAULT_FOREGROUND* = 0
  PI_DEFAULT_DMA_CHANNEL* = 14
  PI_DEFAULT_DMA_PRIMARY_CHANNEL* = 14
  PI_DEFAULT_DMA_SECONDARY_CHANNEL* = 6
  PI_DEFAULT_DMA_PRIMARY_CH_2711* = 7
  PI_DEFAULT_DMA_SECONDARY_CH_2711* = 6
  PI_DEFAULT_DMA_NOT_SET* = 15
  PI_DEFAULT_SOCKET_PORT* = 8888
  PI_DEFAULT_SOCKET_PORT_STR* = "8888"
  PI_DEFAULT_SOCKET_ADDR_STR* = "localhost"
  PI_DEFAULT_UPDATE_MASK_UNKNOWN* = 0x0000000FFFFFFC'i64
  PI_DEFAULT_UPDATE_MASK_B1* = 0x03E7CF93
  PI_DEFAULT_UPDATE_MASK_A_B2* = 0xFBC7CF9C
  PI_DEFAULT_UPDATE_MASK_APLUS_BPLUS* = 0x0080480FFFFFFC'i64
  PI_DEFAULT_UPDATE_MASK_ZERO* = 0x0080000FFFFFFC'i64
  PI_DEFAULT_UPDATE_MASK_PI2B* = 0x0080480FFFFFFC'i64
  PI_DEFAULT_UPDATE_MASK_PI3B* = 0x0000000FFFFFFC'i64
  PI_DEFAULT_UPDATE_MASK_PI4B* = 0x0000000FFFFFFC'i64
  PI_DEFAULT_UPDATE_MASK_COMPUTE* = 0x00FFFFFFFFFFFF'i64
  PI_DEFAULT_MEM_ALLOC_MODE* = PI_MEM_ALLOC_AUTO
  PI_DEFAULT_CFG_INTERNALS* = 0

## DEF_E
