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

import
  submodule

export
  submodule

{.passL: "-lpigpiod_if2".}

const
  PIGPIOD_IF2_VERSION* = 17

#[
  pigpiod_if2 is a C library for the Raspberry which allows control
  of the GPIO via the socket interface to the pigpio daemon.
  
  GPIO*
  
  ALL GPIO are identified by their Broadcom number.
  
  Notes*
  
  The PWM and servo pulses are timed using the DMA and PWM/PCM peripherals.
  
  to run make sure the pigpio daemon is running
  
  . .
  sudo pigpiod
  # sudo is not required to run programs linked to pigpiod_if2
  . .
  
  All the functions which return an int return < 0 on error
]#

type
  CBFunc_t* = proc (pi: cint; user_gpio: cuint; level: cuint; tick: uint32_t)
  CBFuncEx_t* = proc (pi: cint; user_gpio: cuint; level: cuint; tick: uint32_t;
                   userdata: pointer)
  #callback_t* = callback_s
  evtCBFunc_t* = proc (pi: cint; event: cuint; tick: uint32_t)
  evtCBFuncEx_t* = proc (pi: cint; event: cuint; tick: uint32_t; userdata: pointer)
  #evtCallback_t* = evtCallback_s

proc time_time*(): cdouble {.importc.}
proc time_sleep*(seconds: cdouble) {.importc.}

proc pigpio_error*(errnum: cint): cstring {.importc.}

proc pigpiod_if_version*(): cuint {.importc.}

#proc start_thread*(thread_func: gpioThreadFunc_t; userdata: pointer): ptr pthread_t {.importc.}
#proc stop_thread*(pth: ptr pthread_t) {.importc.}

proc pigpio_start*(addrStr: cstring; portStr: cstring): cint {.importc.}
proc pigpio_stop*(pi: cint) {.importc.}

proc set_mode*(pi: cint; gpio: cuint; mode: cuint): cint {.importc.}
proc get_mode*(pi: cint; gpio: cuint): cint {.importc.}

proc set_pull_up_down*(pi: cint; gpio: cuint; pud: cuint): cint {.importc.}

proc gpio_read*(pi: cint; gpio: cuint): cint {.importc.}
proc gpio_write*(pi: cint; gpio: cuint; level: cuint): cint {.importc.}

proc set_PWM_dutycycle*(pi: cint; user_gpio: cuint; dutycycle: cuint): cint {.importc.}
proc get_PWM_dutycycle*(pi: cint; user_gpio: cuint): cint {.importc.}
proc set_PWM_range*(pi: cint; user_gpio: cuint; range: cuint): cint {.importc.}
proc get_PWM_range*(pi: cint; user_gpio: cuint): cint {.importc.}
proc get_PWM_real_range*(pi: cint; user_gpio: cuint): cint {.importc.}
proc set_PWM_frequency*(pi: cint; user_gpio: cuint; frequency: cuint): cint {.importc.}
proc get_PWM_frequency*(pi: cint; user_gpio: cuint): cint {.importc.}

proc set_servo_pulsewidth*(pi: cint; user_gpio: cuint; pulsewidth: cuint): cint {.importc.}
proc get_servo_pulsewidth*(pi: cint; user_gpio: cuint): cint {.importc.}

proc notify_open*(pi: cint): cint {.importc.}
proc notify_begin*(pi: cint; handle: cuint; bits: uint32_t): cint {.importc.}
proc notify_pause*(pi: cint; handle: cuint): cint {.importc.}
proc notify_close*(pi: cint; handle: cuint): cint {.importc.}

proc set_watchdog*(pi: cint; user_gpio: cuint; timeout: cuint): cint {.importc.}
proc set_glitch_filter*(pi: cint; user_gpio: cuint; steady: cuint): cint {.importc.}
proc set_noise_filter*(pi: cint; user_gpio: cuint; steady: cuint; active: cuint): cint {.importc.}

proc read_bank_1*(pi: cint): uint32_t {.importc.}
proc read_bank_2*(pi: cint): uint32_t {.importc.}
proc clear_bank_1*(pi: cint; bits: uint32_t): cint {.importc.}
proc clear_bank_2*(pi: cint; bits: uint32_t): cint {.importc.}
proc set_bank_1*(pi: cint; bits: uint32_t): cint {.importc.}
proc set_bank_2*(pi: cint; bits: uint32_t): cint {.importc.}

proc hardware_clock*(pi: cint; gpio: cuint; clkfreq: cuint): cint {.importc.}
proc hardware_PWM*(pi: cint; gpio: cuint; PWMfreq: cuint; PWMduty: uint32_t): cint {.importc.}

proc get_current_tick*(pi: cint): uint32_t {.importc.}

proc get_hardware_revision*(pi: cint): uint32_t {.importc.}

proc get_pigpio_version*(pi: cint): uint32_t {.importc.}

proc wave_clear*(pi: cint): cint {.importc.}
proc wave_add_new*(pi: cint): cint {.importc.}
proc wave_add_generic*(pi: cint; numPulses: cuint; pulses: ptr gpioPulse_t): cint {.importc.}
proc wave_add_serial*(pi: cint; user_gpio: cuint; baud: cuint; data_bits: cuint;
                     stop_bits: cuint; offset: cuint; numBytes: cuint; str: cstring): cint {.importc.}
proc wave_create*(pi: cint): cint {.importc.}
proc wave_create_and_pad*(pi: cint; percent: cint): cint {.importc.}
proc wave_delete*(pi: cint; wave_id: cuint): cint {.importc.}
proc wave_send_once*(pi: cint; wave_id: cuint): cint {.importc.}
proc wave_send_repeat*(pi: cint; wave_id: cuint): cint {.importc.}
proc wave_send_using_mode*(pi: cint; wave_id: cuint; mode: cuint): cint {.importc.}
proc wave_chain*(pi: cint; buf: cstring; bufSize: cuint): cint {.importc.}
proc wave_tx_at*(pi: cint): cint {.importc.}
proc wave_tx_busy*(pi: cint): cint {.importc.}
proc wave_tx_stop*(pi: cint): cint {.importc.}
proc wave_get_micros*(pi: cint): cint {.importc.}
proc wave_get_high_micros*(pi: cint): cint {.importc.}
proc wave_get_max_micros*(pi: cint): cint {.importc.}
proc wave_get_pulses*(pi: cint): cint {.importc.}
proc wave_get_high_pulses*(pi: cint): cint {.importc.}
proc wave_get_max_pulses*(pi: cint): cint {.importc.}
proc wave_get_cbs*(pi: cint): cint {.importc.}
proc wave_get_high_cbs*(pi: cint): cint {.importc.}
proc wave_get_max_cbs*(pi: cint): cint {.importc.}

proc gpio_trigger*(pi: cint; user_gpio: cuint; pulseLen: cuint; level: cuint): cint {.importc.}

proc store_script*(pi: cint; script: cstring): cint {.importc.}
proc run_script*(pi: cint; script_id: cuint; numPar: cuint; param: ptr uint32_t): cint {.importc.}
proc update_script*(pi: cint; script_id: cuint; numPar: cuint; param: ptr uint32_t): cint {.importc.}
proc script_status*(pi: cint; script_id: cuint; param: ptr uint32_t): cint {.importc.}
proc stop_script*(pi: cint; script_id: cuint): cint {.importc.}
proc delete_script*(pi: cint; script_id: cuint): cint {.importc.}

proc bb_serial_read_open*(pi: cint; user_gpio: cuint; baud: cuint; data_bits: cuint): cint {.importc.}
proc bb_serial_read*(pi: cint; user_gpio: cuint; buf: pointer; bufSize: csize_t): cint {.importc.}
proc bb_serial_read_close*(pi: cint; user_gpio: cuint): cint {.importc.}
proc bb_serial_invert*(pi: cint; user_gpio: cuint; invert: cuint): cint {.importc.}

proc i2c_open*(pi: cint; i2c_bus: cuint; i2c_addr: cuint; i2c_flags: cuint): cint {.importc.}
proc i2c_close*(pi: cint; handle: cuint): cint {.importc.}
proc i2c_write_quick*(pi: cint; handle: cuint; bit: cuint): cint {.importc.}
proc i2c_write_byte*(pi: cint; handle: cuint; bVal: cuint): cint {.importc.}
proc i2c_read_byte*(pi: cint; handle: cuint): cint {.importc.}
proc i2c_write_byte_data*(pi: cint; handle: cuint; i2c_reg: cuint; bVal: cuint): cint {.importc.}
proc i2c_write_word_data*(pi: cint; handle: cuint; i2c_reg: cuint; wVal: cuint): cint {.importc.}
proc i2c_read_byte_data*(pi: cint; handle: cuint; i2c_reg: cuint): cint {.importc.}
proc i2c_read_word_data*(pi: cint; handle: cuint; i2c_reg: cuint): cint {.importc.}
proc i2c_process_call*(pi: cint; handle: cuint; i2c_reg: cuint; wVal: cuint): cint {.importc.}
proc i2c_write_block_data*(pi: cint; handle: cuint; i2c_reg: cuint; buf: cstring;
                          count: cuint): cint {.importc.}
proc i2c_read_block_data*(pi: cint; handle: cuint; i2c_reg: cuint; buf: cstring): cint {.importc.}
proc i2c_block_process_call*(pi: cint; handle: cuint; i2c_reg: cuint; buf: cstring;
                            count: cuint): cint {.importc.}
proc i2c_read_i2c_block_data*(pi: cint; handle: cuint; i2c_reg: cuint; buf: cstring;
                             count: cuint): cint {.importc.}
proc i2c_write_i2c_block_data*(pi: cint; handle: cuint; i2c_reg: cuint; buf: cstring;
                              count: cuint): cint {.importc.}
proc i2c_read_device*(pi: cint; handle: cuint; buf: cstring; count: cuint): cint {.importc.}
proc i2c_write_device*(pi: cint; handle: cuint; buf: cstring; count: cuint): cint {.importc.}
proc i2c_zip*(pi: cint; handle: cuint; inBuf: cstring; inLen: cuint; outBuf: cstring;
             outLen: cuint): cint {.importc.}

proc bb_i2c_open*(pi: cint; SDA: cuint; SCL: cuint; baud: cuint): cint {.importc.}
proc bb_i2c_close*(pi: cint; SDA: cuint): cint {.importc.}
proc bb_i2c_zip*(pi: cint; SDA: cuint; inBuf: cstring; inLen: cuint; outBuf: cstring;
                outLen: cuint): cint {.importc.}

proc bb_spi_open*(pi: cint; CS: cuint; MISO: cuint; MOSI: cuint; SCLK: cuint; baud: cuint;
                 spi_flags: cuint): cint {.importc.}
proc bb_spi_close*(pi: cint; CS: cuint): cint {.importc.}
proc bb_spi_xfer*(pi: cint; CS: cuint; txBuf: cstring; rxBuf: cstring; count: cuint): cint {.importc.}

proc spi_open*(pi: cint; spi_channel: cuint; baud: cuint; spi_flags: cuint): cint {.importc.}
proc spi_close*(pi: cint; handle: cuint): cint {.importc.}
proc spi_read*(pi: cint; handle: cuint; buf: cstring; count: cuint): cint {.importc.}
proc spi_write*(pi: cint; handle: cuint; buf: cstring; count: cuint): cint {.importc.}
proc spi_xfer*(pi: cint; handle: cuint; txBuf: cstring; rxBuf: cstring; count: cuint): cint {.importc.}

proc serial_open*(pi: cint; ser_tty: cstring; baud: cuint; ser_flags: cuint): cint {.importc.}
proc serial_close*(pi: cint; handle: cuint): cint {.importc.}
proc serial_write_byte*(pi: cint; handle: cuint; bVal: cuint): cint {.importc.}
proc serial_read_byte*(pi: cint; handle: cuint): cint {.importc.}
proc serial_write*(pi: cint; handle: cuint; buf: cstring; count: cuint): cint {.importc.}
proc serial_read*(pi: cint; handle: cuint; buf: cstring; count: cuint): cint {.importc.}
proc serial_data_available*(pi: cint; handle: cuint): cint {.importc.}

proc custom_1*(pi: cint; arg1: cuint; arg2: cuint; argx: cstring; argc: cuint): cint {.importc.}
proc custom_2*(pi: cint; arg1: cuint; argx: cstring; argc: cuint; retBuf: cstring;
              retMax: cuint): cint {.importc.}

proc get_pad_strength*(pi: cint; pad: cuint): cint {.importc.}
proc set_pad_strength*(pi: cint; pad: cuint; padStrength: cuint): cint {.importc.}

proc shell*(pi: cint; scriptName: cstring; scriptString: cstring): cint {.importc.}

proc file_open*(pi: cint; file: cstring; mode: cuint): cint {.importc.}
proc file_close*(pi: cint; handle: cuint): cint {.importc.}
proc file_write*(pi: cint; handle: cuint; buf: cstring; count: cuint): cint {.importc.}
proc file_read*(pi: cint; handle: cuint; buf: cstring; count: cuint): cint {.importc.}
proc file_seek*(pi: cint; handle: cuint; seekOffset: int32_t; seekFrom: cint): cint {.importc.}
proc file_list*(pi: cint; fpat: cstring; buf: cstring; count: cuint): cint {.importc.}

proc callback*(pi: cint; user_gpio: cuint; edge: cuint; f: CBFunc_t): cint {.importc.}
proc callback_ex*(pi: cint; user_gpio: cuint; edge: cuint; f: CBFuncEx_t;
                 userdata: pointer): cint {.importc.}
proc callback_cancel*(callback_id: cuint): cint {.importc.}

proc wait_for_edge*(pi: cint; user_gpio: cuint; edge: cuint; timeout: cdouble): cint {.importc.}

proc bsc_xfer*(pi: cint; bscxfer: ptr bsc_xfer_t): cint {.importc.}
proc bsc_i2c*(pi: cint; i2c_addr: cint; bscxfer: ptr bsc_xfer_t): cint {.importc.}

proc event_callback*(pi: cint; event: cuint; f: evtCBFunc_t): cint {.importc.}
proc event_callback_ex*(pi: cint; event: cuint; f: evtCBFuncEx_t; userdata: pointer): cint {.importc.}
proc event_callback_cancel*(callback_id: cuint): cint {.importc.}

proc wait_for_event*(pi: cint; event: cuint; timeout: cdouble): cint {.importc.}

proc event_trigger*(pi: cint; event: cuint): cint {.importc.}

## DEF_S pigpiod_if2 Error Codes

type
  pigifError_t* = enum
    pigif_too_many_pis = -2012
    pigif_unconnected_pi = -2011
    pigif_callback_not_found = -2010
    pigif_notify_failed = -2009
    pigif_bad_callback = -2008
    pigif_bad_malloc = -2007
    pigif_duplicate_callback = -2006
    pigif_bad_noib = -2005
    pigif_bad_socket = -2004
    pigif_bad_connect = -2003
    pigif_bad_getaddrinfo = -2002
    pigif_bad_recv = -2001
    pigif_bad_send = -2000


## DEF_E
