/*
  FlowTEX_I2C.ino
  
  Created: 28/04/2021 18:00:00
  Author: henrique.coser
  
  This example code is in the Public Domain
  This software is distributed on an "AS IS" BASIS, 
  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, 
  either express or implied.
  
  Este código de exemplo é de uso publico,
  Este software é distribuido na condição "COMO ESTÁ",
  e NÃO SÃO APLICÁVEIS QUAISQUER GARANTIAS, implicitas 
  ou explicitas
*/

#include <Wire.h>
#include <stdint.h>

//-------------------------------------------------
// DEFINES
//-------------------------------------------------
#define FLOWTEX_ADDR    0x20

//Defilne for EMA Filter
#define MAX_EMA_ORDER       10
#define EMA_DEFAULT_ORDER   5     //Must be positive and > 0
#define EMA_DEFAULT_ALPHA   0.5f  //Must be positive and >0 and <1

//-------------------------------------------------
// TYPEDEFS
//-------------------------------------------------
//I2C memmory MAP structure table.
typedef struct tI2CTable
{                                   //Address range
    uint8_t flow[3];                //0..2
    uint8_t flowChks;               //3
    uint8_t temp[2];                //4..5
    uint8_t tempChks;               //6
    uint8_t range[3];               //7..9
    uint8_t rangeChks;              //10
    uint8_t serialNumber[10];       //11..20
    uint8_t serialNumberChks;       //21
    uint8_t version[4];             //22..25
    uint8_t versionChks;            //26
    uint8_t fwChks[4];              //27..30
    uint8_t fwChksChks;             //31
}tI2CTable;

//Just a union to merge a memmory buffer with I2C Frame
typedef union tI2CFrame
{
  tI2CTable I2CTable;
  uint8_t   I2CBuffer[sizeof(tI2CTable)];
}tI2CFrame;
//---------------------------------------------

//-------------------------------------------------
// CLASSES
//-------------------------------------------------

/*---------------------------------------------
 * Exponential moving average low pass filter
 * 
 * This filter recursively calculate this 
 * equation:
 * 
 *     Out = (Out*Alpha) + In*(1-Alpha)
 *     
 * This will be calculated the same number    
 * of times as configured in order parameter.
 * 
 *  Higher the Alpha and Order lower is the cut frequency
 *  Lower the Alpha and Order higher is the cut frequency
 *  
 *  Exmple:
 *    Filter with Alpha = 0.5 and Order = 5 has cut frequency higher
 *    than a filter with Alpha = 0.7 and Order 9
 --------------------------------------------*/
class  cEMAFilter
{
  float filterData[MAX_EMA_ORDER];
  float filtValue;
  bool firstSample;
  uint8_t order;
  float alpha;

public:
  
  //Constructor default
  cEMAFilter()
  {
    firstSample = true;
    order = EMA_DEFAULT_ORDER;
    alpha = EMA_DEFAULT_ALPHA;
  }

  //Constructor with parameter
  //  alpha - Filter alpha value must be higher than 0 and lower than 1
  //  order - filter order value must be higher than 0 and equal or lower than MAX_EMA_ORDER
  cEMAFilter(float alpha, uint8_t order)
  {
    firstSample = true;
    order = EMA_DEFAULT_ORDER;
    alpha = EMA_DEFAULT_ALPHA;
    config(alpha, order);    
  }

  //Configure filter parameters
  //  alpha - Filter alpha value must be higher than 0 and lower than 1
  //  order - filter order value must be higher than 0 and equal or lower than MAX_EMA_ORDER
  void config(float alpha, uint8_t order)
  {
    if(((order > 0)&&(order <= MAX_EMA_ORDER))&&
       ((alpha > 0)&&(alpha < 1)))
    {      
      this->order = order;
      this->alpha = alpha;
    }
  }

  //Reset the filter with a presset value.
  //Inputs
  //  Presset value. If supressed 0 will be default
  void reset(float value = 0)
  {
      for(int i = 0; i < order; i++)
      {
        filterData[i] = value;
      }      
  }

  //Filter process
  //Inputs
  //  Value to be filtered
  //Output
  //  Filtered value
  float filter(float value)
  {
    //If is the first sample, force reset the filter
    if(firstSample)
    {
      reset(value);
      firstSample = false;
    }

    //Apply EMA filter 
    //  Equation:     Out = (Out*Alpha) + In*(1-Alpha)
    for(int i = 0; i < order; i++)
    {
      filterData[i] = ((alpha)*filterData[i]) + ((1-alpha)*value);
      value = filterData[i];
    }

    filtValue = value;

    return filtValue;
  }

  //Get the filtered value
  float getValue()
  {
    return filtValue;
  }
};


//-------------------------------------------------
// GLOBAL VARIABLES
//-------------------------------------------------

//Serial buffer
char bufferMessage[50];

//I2C variables
tI2CFrame I2CFrame;
int32_t flow;                //0..2
int16_t temp;                //4..5
int32_t range;               //7..9
uint8_t serialNumber[10];   //11..20
uint8_t version[4];         //22..25
uint32_t fwChks;            //27..30

//Flow an flow filter
float flowValue;
cEMAFilter FlowFilter;

//Software statistics
uint32_t loopCounter;
uint32_t comError = 0;
uint32_t error = 0;
uint32_t success = 0;


//-------------------------------------------------
// PROTOTYPES
//-------------------------------------------------
void RebuildVarFromBuffer(void *ptOutVar, uint8_t *ptBuffer, uint32_t len);
bool masterRead(uint8_t address, uint8_t dataAddress, uint8_t *ptBuffer, uint32_t len );
char *printFloat(char *outputBuffer, float value, float dp, uint32_t maxLen);
void sendMessage(const char *pMessage,...);

//-------------------------------------------------
// IMPLEMENTATION
//-------------------------------------------------
void setup() 
{  
  Wire.begin();        // join i2c bus (address optional for master)
  Serial.begin(9600);  // start serial for output
}

void loop() 
{      
  //Print sensor info every 50 loops
  if(loopCounter > 50)
  {
    char floatOutputBuffer[12];
    sendMessage("\r\n--------------------------------------------------------");
    sendMessage("\r\nSucesso    : %d ",success);
    sendMessage("\r\nErros CHKS : %d ",error);
    sendMessage("\r\nErros I2C  : %d ",comError);
    sendMessage("\r\nFirmware (%d.%d.%d.%d) ",version[0],version[1],version[2],version[3]);                    
    sendMessage("\r\nFundo de escala : %sSccm ", printFloat(floatOutputBuffer, (float)range, 0, 12));
    sendMessage("\r\nVazao (filtrada): %sSccm ", printFloat(floatOutputBuffer, FlowFilter.getValue(), 3, 12));
    sendMessage("\r\nTemperatura     : %s°C", printFloat(floatOutputBuffer,((float)temp)/100.0f, 2, 6));
    sendMessage("\r\n--------------------------------------------------------\r\n");
  }

  //Read flow sensor
  if( masterRead(FLOWTEX_ADDR, 0, (uint8_t *)&I2CFrame.I2CBuffer, sizeof(tI2CTable)) )
  {        
    //Calc frame checksum
    uint8_t chks = 0;
    for(uint8_t i = 0; i < sizeof(tI2CTable); i++)
    {
      chks += I2CFrame.I2CBuffer[i];
    }
    
    if(chks != 0)
    {
      error++;        
    }
    else
    {      
      success++;  
      flow = 0;
      temp = 0;
      range = 0;
      memset(serialNumber,0,sizeof(serialNumber));
      memset(version,0,sizeof(version));
      fwChks = 0;

      //Build variables from I2C buffer
      RebuildVarFromBuffer(&flow, I2CFrame.I2CTable.flow, 3);
      RebuildVarFromBuffer(&temp, I2CFrame.I2CTable.temp, 2);
      RebuildVarFromBuffer(&range, I2CFrame.I2CTable.range, 3);
      RebuildVarFromBuffer(&serialNumber, I2CFrame.I2CTable.serialNumber, 10);
      RebuildVarFromBuffer(&version, I2CFrame.I2CTable.version, 4);
      RebuildVarFromBuffer(&fwChks, I2CFrame.I2CTable.fwChks, 4);

      //Positive and Negatige flow process
      if(flow&0x800000) //Check if the value is negative
      {
        flow = (~flow) + 1; //Swap it to positive
        flow &= 0x7FFFFF; //Clear all remaining bits
        flowValue = -(((float)flow)*((float)range))/((float)0x7FFFFF);   //Calculate negative flow
      }
      else
      {
        flowValue = (((float)flow)*((float)range))/((float)0x7FFFFF);   //Calculate positive flow     
      }
      //------------------------------------

      //Apply filter on the flow
      FlowFilter.filter(flowValue);
    }
  }
  else
  {
    comError++;
  }
  
  loopCounter++;
  delay(10);
}

//Buid a variable from a buffer
//Just will copy a specific part of the buffer into a desired variable
//This will be used to get bytes from a reception buffer and build a int or float variables
void RebuildVarFromBuffer(void *ptOutVar, uint8_t *ptBuffer, uint32_t len)
{
    memcpy(ptOutVar, ptBuffer, len);
}

//Read I2C address
//Inputs: 
//  Device address, 
//  Memmory to read address
//  Buffer that will receive the data
//  Length of data to read
bool masterRead(uint8_t address, uint8_t dataAddress, uint8_t *ptBuffer, uint32_t len )
{
    uint32_t rxLen = 0;
    Wire.requestFrom(address, len);    // request 'len' bytes from slave device  
    while(Wire.available())    // slave may send less than requested
    {
        if(rxLen < len)
        {
            *ptBuffer++ = Wire.read();    // receive a byte as character
        }
        rxLen++;
    }
    return len == rxLen;
}


//This will be a tool to print float.
//Since ARDUINO does not support %f formating, this will do something like it
//Inputs
//  Output char buffer  - Will get the string null terminated
//  Float value to print
//  Number of decimal places
//  Max length - Output buffer always will be built using tge maxLen
char *printFloat(char *outputBuffer, float value, float dp, uint32_t maxLen)
{ 
  bool bNegative = false;
  
  //Check if the value is negative and make it positive
  if(value < 0)
  {
    bNegative = true;
    value = -value;
  }

  //Multiply by the number of decimal places to get the number as a integer
  //But add 0.5 to round it instead of trunc the value
  int32_t intValue = ((value * pow(10,dp))+0.5) ;

  //Ensure that the output buffer will be null terminated
  outputBuffer[maxLen] = 0;
  
  //Will get number by number of the value filling the output buffer
  //To get number by numbe is just need the remaining of a division by 10
  //To transform a number to ASCII just add 0x30  
  for(int i = maxLen-1, j = 0; i >= 0; i--, j++)
  {
      if((j == dp)&&(dp != 0))
      {   outputBuffer[i] = ',';   }
      else
      {
          //Get the remain of division and transform into ASCII adding 0x30
          outputBuffer[i] = (intValue%10)+0x30; 
          intValue /= 10;
      }
  }

  //Suppress the non significant 0 adding a space in place
  for(int i = 0; i < maxLen-1; i++)
  {
    if((outputBuffer[i+1] == ',')||(outputBuffer[i+1] == 0))
    {  break; }
    else
    { 
      if(outputBuffer[i] == '0')
      {
        outputBuffer[i] = ' ';
      }
      else
      { break;  }
    }
  }

  //Find the last space to put the '-' signal when negative
  if(bNegative)
  {
      int idx = 0;
      while(outputBuffer[idx+1] == ' ')
      {
         idx++;
      }
      outputBuffer[idx] = '-';
  }

  return outputBuffer;
};

//Print a debug message to serial port.
//Inputs: 
//  Message 
//  Format parameters like printf function
void sendMessage(const char *pMessage,...)
{
  va_list args;
  va_start(args,pMessage);
  vsnprintf((char *)bufferMessage, sizeof(bufferMessage),pMessage,args);
  va_end(args);
  Serial.write((char *)bufferMessage);
}
