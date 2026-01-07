/*
C:\Users\tzerb\AppData\Local\Arduino15\packages\arduino\tools\avrdude\6.3.0-arduino17\bin>avrdude -c arduino -C C:\Users\tzerb\AppData\Local\Arduino15\packages\arduino\tools\avrdude\6.3.0-arduino17\etc\avrdude.conf -p t85 -v -P COM7 -b 19200
*/

#define LED_PIN 0
#define NUM_LEDS 300
#define LED_BUILTIN 1

// Proper cycle-counted assembly for 8MHz ATtiny85
void sendByte(uint8_t byte) {
  asm volatile(
    "       ldi  r18, 8      \n\t"  // 8 bits to send
    "loop:                   \n\t"
    "       sbi  %[port], %[pin] \n\t"  // 2 cycles - pin HIGH
    "       sbrs %[byte], 7  \n\t"  // 1-2 cycles - skip if bit 7 set
    "       cbi  %[port], %[pin] \n\t"  // 2 cycles - '0': LOW after ~3 cycles (375ns)
    "       lsl  %[byte]     \n\t"  // 1 cycle - shift left
    "       dec  r18         \n\t"  // 1 cycle - decrement counter
    "       cbi  %[port], %[pin] \n\t"  // 2 cycles - '1': LOW after ~7 cycles (875ns)
    "       brne loop        \n\t"  // 2 cycles - loop if not zero
    :
    : [byte] "r" (byte),
      [port] "I" (_SFR_IO_ADDR(PORTB)),
      [pin] "I" (LED_PIN)
    : "r18"
  );
}

void setAllLEDs(uint8_t r, uint8_t g, uint8_t b, uint8_t w) {
  cli(); // Disable interrupts
  
  for (uint16_t i = 0; i < NUM_LEDS; i++) {
    sendByte(w); // WS2814 is WRGB order (not GRBW!)
    sendByte(r);
    sendByte(g);
    sendByte(b);
  }
  
  sei(); // Re-enable interrupts
  _delay_us(80); // Latch
}

void setup() {
  pinMode(LED_PIN, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  
  delay(50); // Wait for LEDs to stabilize after power-up
  
  // Send a "reset" by holding line low
  digitalWrite(LED_PIN, LOW);
  delayMicroseconds(100);
  
  // Optional: Clear the strip first
  setAllLEDs(0, 0, 0, 0); // All off
  delay(10);  
}

void loop() {
  // Pure red (no white)
  setAllLEDs(255, 0, 0, 0);
  delay(1000);
  digitalWrite(LED_BUILTIN, LOW);   // turn the LED off by making the voltage LOW
 
  // Pure green
  setAllLEDs(0, 255, 0, 0);
  delay(1000);
  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on
  
  // Pure blue
  setAllLEDs(0, 0, 255, 0);
  delay(1000);
  digitalWrite(LED_BUILTIN, LOW);   // turn the LED off by making the voltage LOW
  
  // Warm white (using W channel)
  setAllLEDs(0, 0, 0, 255);
  delay(1000);
  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on
    
  setAllLEDs(255, 255, 255, 0);
  delay(1000);
  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on

  setAllLEDs(127, 127, 127, 0);
  delay(1000);
  digitalWrite(LED_BUILTIN, LOW);   // turn the LED on'

  // Warm white (using W channel)
  setAllLEDs(0, 0, 0, 127);
  delay(1000);
  digitalWrite(LED_BUILTIN, LOW);   // turn the LED on'
  
  // Warm white (using W channel)
  setAllLEDs(0, 0, 0, 63);
  delay(1000);
  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on
    
  // Warm white (using W channel)
  setAllLEDs(0, 0, 0, 31);
  delay(1000);
  digitalWrite(LED_BUILTIN, LOW);   // turn the LED on'
  
  // Warm white (using W channel)
  setAllLEDs(0, 0, 0, 15);
  delay(1000);
  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on

  // Mixed: Red + White
  setAllLEDs(255, 0, 0, 128);
  delay(1000);
  digitalWrite(LED_BUILTIN, LOW);   // turn the LED off by making the voltage LOW
}