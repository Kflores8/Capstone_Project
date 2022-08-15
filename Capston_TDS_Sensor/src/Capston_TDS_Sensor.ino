/*
 * Project Capston_TDS_Sensor
 * Description: Program read of Total Dissolve Solids within SMART Cooler. TDS indicates how many milligrams of 
 * dissolved solids are dissolved in 1 liter of water. For the Cooler, maximum water capacity is 46 liters. 
 * Author: Kevin Flores
 * Date: 15 Aug 2022
 */

const float VREF = 5.0;      // analog reference voltage(Volt) of the ADC
const int SCOUNT = 30;           // sum of sample point
int TdsSensorPin = A0;
int analogBuffer[SCOUNT];    // store the analog value in the array, read from ADC
int analogBufferTemp[SCOUNT];
int analogBufferIndex = 0, copyIndex = 0;
float averageVoltage = 0, tdsValue = 0, temperature = 25;

void setup()
{
  Serial.begin(115200);
  pinMode(TdsSensorPin, INPUT);
}

void loop()
{
  static unsigned long analogSampleTimepoint = millis();
  if (millis() - analogSampleTimepoint > 40)  //every 40 milliseconds,read the analog value from the ADC
  {
    analogSampleTimepoint = millis();
    analogBuffer[analogBufferIndex] = analogRead(TdsSensorPin);    //read the analog value and store into the buffer
    analogBufferIndex++;
    if (analogBufferIndex == SCOUNT)
      analogBufferIndex = 0;
  }
  static unsigned long printTimepoint = millis();
  if (millis() - printTimepoint > 1000)
  {
    printTimepoint = millis();
    for (copyIndex = 0; copyIndex < SCOUNT; copyIndex++)
      analogBufferTemp[copyIndex] = analogBuffer[copyIndex];
    averageVoltage = getMedianNum(analogBufferTemp, SCOUNT) * (float)VREF / 1024.0; // read the analog value more stable by the median filtering algorithm, and convert to voltage value
    float compensationCoefficient = 1.0 + 0.02 * (temperature - 25.0); //temperature compensation formula: fFinalResult(25^C) = fFinalResult(current)/(1.0+0.02*(fTP-25.0));
    float compensationVolatge = averageVoltage / compensationCoefficient; //temperature compensation
    tdsValue = (133.42 * compensationVolatge * compensationVolatge * compensationVolatge - 255.86 * compensationVolatge * compensationVolatge + 857.39 * compensationVolatge) * 0.5; //convert voltage value to tds value
    Serial.printf("voltage: %0.2f", averageVoltage);
    Serial.printf("V   \n");
    Serial.printf("TDS----Value: %0.1f", tdsValue);
    Serial.printf("ppm\n");
  }
}
int getMedianNum(int bArray[], int iFilterLen)  //tabulates the reading in array and devises by total readings to smooth out variabilities
{
  int bTab[iFilterLen];  //bTab array identifies the tabulated sum of readings stored
  for (byte i = 0; i < iFilterLen; i++) //iFilter len is the number of times being sampled
    bTab[i] = bArray[i]; //bArray is the median of array readings of tds
  int i, j, bTemp;
  for (j = 0; j < iFilterLen - 1; j++)
  {
    for (i = 0; i < iFilterLen - j - 1; i++)
    {
      if (bTab[i] > bTab[i + 1]) 
      {
        bTemp = bTab[i];
        bTab[i] = bTab[i + 1];
        bTab[i + 1] = bTemp;
      }
      //Serial.printf("bTab is %i, bTemp is %i \n", bTab[i], bTemp);
    }
  }
  if ((iFilterLen & 1) > 0)
    bTemp = bTab[(iFilterLen - 1) / 2]; //subtract 1 due to the number of array starting at 0
  else
    bTemp = (bTab[iFilterLen / 2] + bTab[iFilterLen / 2 - 1]) / 2;
  return bTemp;
}