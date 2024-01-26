  # myBluePillThrottle7.ino

  Simple DCC-EX tethered throttle using STM32 Blue Pill,
  5x4 keypad, and SSD1306 I2C OLED display.  It is primarily
  for operating one loco at a time and does not include
  consisting or other advanced features.

  This version has added code to handle an optional rotary
  encoder for speed changes in addition to the keypad UP and
  DOWN keys.  If an encoder is not used, comment out the
  statement #define ENCODER below before compiling and
  uploading to the throttle.

  This version has several modifications and refactoring
  of the code to improve switching between locos and
  handling of automatic speed changes when going into and
  out of the Brake/Pause sub-mode.  The emergency brake now
  stops only the current loco being operated.

  Loco speed and direction are read from CS broadcasts
  only to update the display in case a throttle command
  is corrupted for some reason or if a second throttle
  is being used to control the same loco.

  The action of the "Brake" mode emulates the Pause button
  in Engine Driver.  The Brake key is the ESC key in Run mode.

  Decoder functions Fn 0 to Fn 9 are activated by the digit
  keys in Run mode.  The functions 10 to 19 are activated by
  the digit keys in Fn+10 mode, and functions 20 to 27 are
  activated similarly in Fn+20 mode.  In Fn+20 mode the 8
  digit exchanges the direction keys and the 9 digit does
  an emergency stop for the current loco.

  Andrew Palm, 1/26/2024

  Operation:  There are four modes, Run, Fn+10, Fn+20, and Addr, as follows:

  Run mode:  Primary mode of operation with keypad assignments:

    F1 - Enter Fn+10 mode
    F2 - Enter Fn+20 mode
    # - Toggle track power
    * - Enter Addr mode
    UP - Increase speed (inactive while braked)
    DOWN - Decrease speed (inactive while braked)
    Esc - Toggle brake (pause)
    Ent - Stop, set speed to zero
    LEFT - Go Forward direction
    RIGHT - Go Reverse direction
    0 to 9 - Toggle decoder function 0 to 9

  Fn+10 mode:  Toggle decoder functions 10 to 19

    0 to 9 - Toggle function 10 + digit
    Esc - Exit Fn mode without entering digit

  Fn+20 mode:  Toggle decoder functions 20 to 28

    0 to 7 - Toggle function 20 + digit
    Esc - Exit Fn mode without entering digit
    8 - Exchange directions of LEFT and RIGHT keys
    9 - Emergency stop (current loco only)

  Addr mode:  Enter loco address (short or long)

    0 to 9 - Used to enter address into display
    Ent - Finished entering address, set address
    Esc - Exit Addr mode without setting address
