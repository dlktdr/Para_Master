# Para_Master
Acts as a PARA Master to Communicate with FRSky Radios.

Program this code to an Arduino BLE 33 Nano. Put your remote in Slave/Bluetooth.

It will look for the first available Slave device, connect to it and output the PPM data on Pin D10.

# htcustom Branch

This branch is dedicated to the Headtracker project. It adds the ability to remote reset center and adds PPMinput capability to the board.

## Pinout of the HTCustom Branch

* D2 - Pull Low(GnD) to reset
* D3 - Pull Low(GnD) to reset (alternate)
* D9 - PPM Input Pin (Requries 4-8Ch's to be sent)
* D10 - PPM Output
