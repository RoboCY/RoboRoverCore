# 08_CommandFusionHub_V3

The WiFi dashboard: drive the rover from a phone over WiFi, and program what
the phone's buttons do.

This project runs on two chips, so it has two sketches:

| Folder | Runs on | Who changes it |
|--------|---------|----------------|
| [WiFiModule_V3](../WiFiModule_V3/) | The WiFi module (ESP-01S) | Nobody - it comes already programmed |
| [Kypruino_Rover](Kypruino_Rover/) | The Kypruino | **You** - this is the one to play with |

The WiFi module's sketch has its own folder because
[11_UnboxingDemo_V3](../11_UnboxingDemo_V3/) uses the same one.

## Getting started

1. Upload `Kypruino_Rover` to the Kypruino (same as every other project).
2. Switch the rover on and join the WiFi network `RoboRover-XXXX` with your
   phone, then open `http://192.168.4.1` in Chrome (Android) or Safari
   (iPhone). There is no sign-in pop-up on purpose: that window cannot turn to
   landscape. The page goes fullscreen at the first touch and stays there.
3. Drive with the joystick. Try the buttons - **Piano** turns them into a
   keyboard for the rover's buzzer - and type `help` in the Console (the `>_`
   button).

The joystick and STOP always work. What **Lights, Dance, Beep, Piano, Custom A
and Custom B** do, and what the rover answers in the Console, is written in
`Kypruino_Rover.ino`. Change it and upload again.

`WiFiLink.h` is the connection to the WiFi module. You do not need to change it.

## Driving

- Push the half-circle joystick up to drive. The green strip in the middle
  drives dead straight - the rover's screen says *Straight* and the wheel
  encoders keep both wheels in step; along the flat edge the rover spins on
  the spot.
- **Reverse** (top left): pushing up then drives backwards.
- **Avoid** (top right): the rover will not drive forwards when an obstacle
  sensor sees something or the ultrasonic reads under 15 cm. Reversing and
  spinning still work.
- The top bar shows the **speed** and the **distance driven**, measured by
  the wheel encoders. Tap it to start the distance from 0 (or type `trip` in
  the Console).
- **Gamepad** (switch it on in the Console, `>_`): the left stick drives,
  pulled back it reverses. B = STOP, A = Beep, X = Lights, Y = Dance. Press
  any button on the gamepad once so the browser finds it.
- **Tilt drive** (Console): hold the pad and tilt the phone - the angle when
  your finger goes down counts as level. Browsers only allow tilt on https
  pages, so on `http://192.168.4.1` it needs a one-time Chrome setting on
  Android: open `chrome://flags`, find *Insecure origins treated as secure*,
  add `http://192.168.4.1`, enable it and relaunch. On an iPhone it does not
  work on the rover's own page. The internet remote, opened from an https
  address, has it without that.

## Safety

- The rover stops when you press STOP, when the phone disconnects, and when the
  joystick goes quiet for 0.8 s.
- Pressing any button while Dance (or anything else) is running stops it.
- Use `waitFor()` instead of `delay()` in your code. While `delay()` runs the
  Kypruino is not listening to the phone.

## Programming the WiFi module (only if it is blank)

See [WiFiModule_V3](../WiFiModule_V3/).
