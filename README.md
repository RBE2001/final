# RBE 2001 final
Code for final project for WPI 2015 C-term RBE2001 group 8.
Code can be found at https://github.com/RBE2001/final

## Installation
In order to use pololu line following sensor, grab code from:
https://github.com/pololu/qtr-sensors-arduino

You must also install the RBE2001 arduino libraries (usually available through myWPI) and replace all instances of `Serial1` with `Serial3`. The reason for this is that, in order to use the builtin bluetooth port on the RBE2001 Arduino Sheild, you must use Serial3.

## Files
All of the code that actually ran on the final robot is in the final/ folder. turntest/, fieldtest/, and armtest/ contain .ino files for testing the turning, bluetooth, and arm respectively; in order to function, you must make sure that the arduino compiler includes the appropriate .h and .cpp files from final/ in the compiling/linking process.

## Style
Where possible, the [Google C++ Style Guide](http://google-styleguide.googlecode.com/svn/trunk/cppguide.html) should be followed.
