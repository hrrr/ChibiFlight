# ChibiFlight

ChibiFlight is basic, small, experimetal, acro-only flight cotroller software for quadcopters.

Restrictions:

- Acro only. No stabilized flight modes. No autoleveling. Gyro only .Accelerometer is not used at all.
- Currently it runs only on Sparky2 and MotoF3 boards.
- Only receiver spported: Sepktrum satellites in 11bit-2048 mode
- One shot only: ESCs must support One shot (125-250us) mode.

There is no setup utility. Everything is fixed at compilation.

Important: ChibiFlight is very experimetal. Use at your own risk.

Chibilfight is very uncorfortable to use. It is very far from being plug and play. It is definetly not for you if you don't know what is c-compiler and are not used to compile and debug c-code.

Check the 'doc' directory for instructions.