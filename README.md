<H1>Hydroponics Project</H1>
<h2>Welcome to our main github page!</h2>

  <p>Here you will see the following files</p>
  <ol>
  <li><b>code/hydro_mcu</b> our main code</li>
  <li><b>documentation</b> our documenation, bill of materials, info. about boards, circuit digrams, and more. </li>
    <li><b>fusion</b> this contains the cad files used for building this project</li>
    <li><b>uart_display</b> which contains the UART display code</li>
  </ol>
  </br>
  <p>
    This project uses an STM32F072RBT6 and a Raspberry Pi Pico.
The following is the pinout for the STM32:

//-------------------------------------
//---------Current Pins Used-----------
</br>
// ADC -> PC1
</br>
// Push Button -> PA0
</br>
// Pump -> PB4
</br>
// Lighting -> PB5
</br>
// USART -> TX - PC4, RX - PC5
</br>
// LEDS -> PC9, PC8, PC7, PC6
</br>
// Unused but enabled -> PB6, PB7
</br>
//-------------------------------------
</br>

//-------------------------------------
</br>

To begin setup build voltage regulator circuit in documentation. Use any type of water tight containers for plant setup and water reservoir. Setup pump to voltage regulator circuit and relay as in documentation. Set the pump in the reservoir and hose to the drip tray. Lighting circuit will consist of LED an 1k resistor in series. The USART will connect to pins 0 (TX) and 1 (RX) and pin 16 and 17 for display on the Raspberry Pi. Once all connected fire up and have fun growing!
  </p>
#### Contributors 
<!--
Name - UID
-->
Jared Nay - u1167483
, Tyler Evans - u1313811
, Inhyup Lee - u1214753
, Woojin Lee - u1398084
