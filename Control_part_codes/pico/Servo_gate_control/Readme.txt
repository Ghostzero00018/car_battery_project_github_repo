In this folder the codes are mainly focused on the control of the servo, and the servo is used to control the open and close of the gate in this project.
In case you don't know what the gate is for, you can simply consider it as the valve that control whether the air enters the box, and if the air does enter, the air flow rate.
There are 4 versions of the code, each of them are using PID control. You can run on your Raspberry board to see the difference.

To test the code you need some basic hardwares: 1 basic Raspberry board (for example I use Raspberry Pi Pico W in this case, and don't forget about the USB cable so you can connect it to your PC); a breadboard to simplify the wiring; some wires of jumpers; a 5V power source.
The reason I use 5V power source is that if you use the 3.3V voltage that the Rspberry board provided, usually it is not enough to power up even the mini servo motor you can find on the market.
About the detialed wiring, you can check on this link: https://picozero.readthedocs.io/en/latest/recipes.html#servo