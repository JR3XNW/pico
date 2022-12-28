/*
pico FFT Band Scope 2022/12/15 JR3XNW
  Library to add
  arduinoFFT.h
  Wire.h
  U8g2lib.h
*/

#include 
#include 
#include "arduinoFFT.h"

#define I_IN 26  //I-Input pins
#define Q_IN 27  //Q-Input pins 

#define PX1 63  //Positive frequency screen (Q) origin 62
#define PY1 42  //Bottom edge of spectrum screen
#define PY2 56
#define SAMPLES 256  //Must be a power of 2
#define WFrow 12

U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/U8X8_PIN_NONE); 

arduinoFFT FFT = arduinoFFT();

double vReal[SAMPLES];
double vImag[SAMPLES];

byte DSdata[256];
byte WFdata[WFrow][128];

void setup() {
  pinMode(25, OUTPUT);      // pico built-in LED
  Serial.begin(115200);
  analogReadResolution(12);  // Set ADC full scale to 12 bits
  u8g2.begin();
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.setDrawColor(1);
  u8g2.setFontPosTop();  // The upper left corner is used as the character position reference.
  u8g2.clearBuffer();
  u8g2.drawStr(0, 0, "Band Scope v0.1");
  u8g2.sendBuffer();
  delay(1000); 

}
 
void loop() {
    digitalWrite(25, HIGH);  // Built-in LED lights up during sampling 

    /*SAMPLING*/
    for(int i=0;  i< SAMPLES; i++)
    {
        vReal[i] = (analogRead(I_IN) - 2048) * 3.3 / 4096.0;  //
        vImag[i] = (analogRead(Q_IN) - 2048) * 3.3 / 4096.0;  //
    }
    digitalWrite(25, LOW);

    /*FFT*/
    FFT.Windowing(vReal, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
    FFT.Windowing(vImag, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD); 
    FFT.Compute(vReal, vImag, SAMPLES, FFT_REVERSE);
    FFT.ComplexToMagnitude(vReal, vImag, SAMPLES);
    
    u8g2.clearBuffer();   // Screen buffer clear
    showScope();       // Spectrum Display
    showGraphics();         // Scale line and other indications
    u8g2.sendBuffer();    // 

    delay(1);  //Repeat the process every second OR:
    
}

void showScope() {  // Spectrum Display
  int d, d1, d2;
  
  for (int xi = 1; xi < 64; xi++) {  // Positive frequency spectrum display
    d1 = barLength(vReal[xi*2]);
    d2 = barLength(vImag[xi*2+1]);
    d = sqrt(d1 * d1 + d2 * d2);
    u8g2.drawVLine(xi + 64 , PY1 - d, d);  
  }
  for (int xi = 64; xi < 128; xi++) { // Negative frequency spectrum display
    d1 = barLength(vReal[xi*2]); 
    d2 = barLength(vImag[xi*2+1]);
    d = sqrt(d1 * d1 + d2 * d2);
    u8g2.drawVLine(xi - 64 , PY1 - d, d); 
  }
}

int barLength(double d) {  // Calculate the length of the graph
  float fy;
  int y;

  fy = 14.0 * (log10(d) + 3.3);  //
  y = fy;
  y = constrain(y, 0, 42);  // Cut off upper and lower limits

  /*For Test*/
  Serial.print(d, 4);
  Serial.print(", ");
  Serial.print(fy);
  Serial.print(", ");
  Serial.println(y);
  
  return y;
}

void showGraphics() {  // Modifying Graphs
  // area demarcation line
  u8g2.drawHLine(0, PY1, 128);  // lower end of the spectrum
  u8g2.drawHLine(0, 55, 128);  // Lower end line for waterfall

  // Frequency scale (horizontal axis)
  u8g2.drawBox(PX1 - 24, PY2, 2, 2);  // Positive Frequency 10k
  u8g2.drawBox(PX1 - 46, PY2, 2, 2);  // Positive Frequency 20k

  u8g2.drawBox(PX1, PY2, 2, 2);       // Negative frequency 0k
  u8g2.drawBox(PX1 + 22, PY2, 2, 2);  // Negative frequency 10k
  u8g2.drawBox(PX1 + 45, PY2, 2, 2);  // Negative frequency 20k

  u8g2.setFont(u8g2_font_micro_tr);  // Small font(3x5)
  u8g2.drawStr(9, 59, "-20k");       // Negative frequency display
  u8g2.drawStr(32, 58, "-10k");

  u8g2.drawStr(63, 58, "0");  // Positive Frequency
  u8g2.drawStr(81, 58, "10k");
  u8g2.drawStr(105, 58, "20k");
 
  // Dummy display for future use
  u8g2.setFont(u8g_font_unifont); //u8g2_font_t0_16_mr
  char str[20] = {"-Band Scope-"};
  byte x = u8g2.getStrWidth(str);
  u8g2.drawStr(64-(x/2), -1, str);

}