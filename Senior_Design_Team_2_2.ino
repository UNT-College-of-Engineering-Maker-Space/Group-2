/*
  Tyler Adam Martinez
*/
#define USE_ARDUINO_INTERRUPTS true    // Set-up low-level interrupts for most acurate BPM math.
#include "PulseSensorPlayground.h"     // Includes the PulseSensorPlayground Library.   
#define dht_dpin A1 // Use A1 pin as Data pin for DHT11.
#define BAUDRATE 115200
#include <SPI.h>
#include <RH_RF95.h>
#include <Console.h>
#include <Process.h>

//  Variables
const int PulseWire = 0;       // PulseSensor PURPLE WIRE connected to ANALOG PIN 0
const int LED13 = 13;          // The on-board Arduino LED, close to PIN 13.
int Threshold = 550;           // Determine which Signal to "count as a beat" and which to ignore.
                               // Use the "Gettting Started Project" to fine-tune Threshold Value beyond default setting.
                               // Otherwise leave the default "550" value. 
byte bGlobalErr;
char dht_dat[5]; // Store Sensor Data
char node_id[3] = {1,1,1}; //LoRa End Node ID
float frequency = 868.0;
unsigned int count = 1;

String myWriteAPIString = "P07KVY59P5QEY6M6"; //  <-- Remember to change to your person one. 
uint16_t crcdata = 0;
uint16_t recCRCData = 0;
float frequency = 868.0;
String dataString = "";

void uploadData(); // Upload Data to ThingSpeak.
                               
PulseSensorPlayground pulseSensor;  // Creates an instance of the PulseSensorPlayground object called "pulseSensor"
RH_RF95 rf95;

void setup() {   
    InitDHT();
    Serial.begin(9600);   // For Serial Monitor
    Bridge.begin(BAUDRATE);
    if (!rf95.init()) 
    {
    Serial.println("init failed (Serial Monitor)");
    Console.println("init failed (Console Window)");
    } 
    rf95.setFrequency(frequency); //Setup ISM frequency
    rf95.setTxPower(13); //Setup Power, dBm
    //rf95.setSyncWord(0x34);

    Console.println("LoRa Gateway Example  --");
    Console.println("    Upload Single Data to ThinkSpeak");
    
    Serial.println("LoRa End Node Example --"); 
    Serial.println("    DHT11 Temperature and Humidity Sensor\n");
    Serial.print("LoRa End Node ID: ");

    for(int i = 0;i < 3; i++)
    {
        Serial.print(node_id[i],HEX);
    }
    Serial.println();

    // Configure the PulseSensor object, by assigning our variables to it. 
    pulseSensor.analogInput(PulseWire);   
    pulseSensor.blinkOnPulse(LED13);       //auto-magically blink Arduino's LED with heartbeat.
    pulseSensor.setThreshold(Threshold);   

    // Double-check the "pulseSensor" object was created and "began" seeing a signal. 
    if (pulseSensor.begin()) {
    Serial.println("We created a pulseSensor Object !");  //This prints one time at Arduino power-up,  or on Arduino reset.  
  }
}

void InitDHT()
{
    pinMode(dht_dpin,OUTPUT);//Set A1 to output
    digitalWrite(dht_dpin,HIGH);//Pull high A1
}

uint16_t recdata( unsigned char* recbuf, int Length)
{
    crcdata = CRC16(recbuf, Length - 2); //Get CRC code
    recCRCData = recbuf[Length - 1]; //Calculate CRC Data
    recCRCData = recCRCData << 8; //
    recCRCData |= recbuf[Length - 2];
}

//Get Sensor Data
void ReadDHT()
{
    bGlobalErr=0;
    byte dht_in;
    byte i;
        
    //pinMode(dht_dpin,OUTPUT);
    digitalWrite(dht_dpin,LOW);//Pull Low A0 and send signal
    delay(30);//Delay > 18ms so DHT11 can get the start signal
        
    digitalWrite(dht_dpin,HIGH);
    delayMicroseconds(40);//Check the high level time to see if the data is 0 or 1
    pinMode(dht_dpin,INPUT);
    // delayMicroseconds(40);
    dht_in=digitalRead(dht_dpin);//Get A0 Status
    //   Serial.println(dht_in,DEC);
    if(dht_in){
        bGlobalErr=1;
        return;
    }
    delayMicroseconds(80);//DHT11 send response, pull low A0 80us
    dht_in=digitalRead(dht_dpin);
    
    if(!dht_in){
        bGlobalErr=2;
        return;
    }
    delayMicroseconds(80);//DHT11 send response, pull low A0 80us
    for (i=0; i<5; i++)//Get sensor data
    dht_dat[i] = read_dht_dat();
    pinMode(dht_dpin,OUTPUT);
    digitalWrite(dht_dpin,HIGH);//release signal and wait for next signal
    byte dht_check_sum = dht_dat[0]+dht_dat[1]+dht_dat[2]+dht_dat[3];//calculate check sum
    if(dht_dat[4]!= dht_check_sum)//check sum mismatch
        {bGlobalErr=3;}
};

byte read_dht_dat(){
    byte i = 0;
    byte result = 0;
    for(i=0; i< 8; i++)
    {
        while(digitalRead(dht_dpin)==LOW);//wait 50us
        delayMicroseconds(30);//Check the high level time to see if the data is 0 or 1
        if (digitalRead(dht_dpin)==HIGH)
        result |=(1<<(7-i));//
        while (digitalRead(dht_dpin)==HIGH);//Get High, Wait for next data sampleing. 
    }
    return result;
}

uint16_t calcByte(uint16_t crc, uint8_t b)
{
    uint32_t i;
    crc = crc ^ (uint32_t)b << 8;
    
    for ( i = 0; i < 8; i++)
    {
        if ((crc & 0x8000) == 0x8000)
            crc = crc << 1 ^ 0x1021;
        else
            crc = crc << 1;
    }
    return crc & 0xffff;
}

uint16_t CRC16(uint8_t *pBuffer,uint32_t length)
{
    uint16_t wCRC16=0;
    uint32_t i;
    if (( pBuffer==0 )||( length==0 ))
    {
      return 0;
    }
    for ( i = 0; i < length; i++)
    { 
      wCRC16 = calcByte(wCRC16, pBuffer[i]);
    }
    return wCRC16;
}

void HumiditySensor()
{
   Serial.print("###########    ");
    Serial.print("COUNT=");
    Serial.print(count);
    Serial.println("    ###########");
    count++;
    ReadDHT();
    char data[50] = {0} ;
    int dataLength = 7; // Payload Length
    // Use data[0], data[1],data[2] as Node ID
    data[0] = node_id[0] ;
    data[1] = node_id[1] ;
    data[2] = node_id[2] ;
    data[3] = dht_dat[0];//Get Humidity Integer Part
    data[4] = dht_dat[1];//Get Humidity Decimal Part
    data[5] = dht_dat[2];//Get Temperature Integer Part
    data[6] = dht_dat[3];//Get Temperature Decimal Part
    switch (bGlobalErr)
    {
      case 0:
          Serial.print("Current humidity = ");
          Serial.print(data[3], DEC);//Show humidity
          Serial.print(".");
          Serial.print(data[4], DEC);//Show humidity
          Serial.print("%  ");
          Serial.print("temperature = ");
          Serial.print(data[5], DEC);//Show temperature
          Serial.print(".");
          Serial.print(data[6], DEC);//Show temperature
          Serial.println("C  ");
          break;
       case 1:
          Serial.println("Error 1: DHT start condition 1 not met.");
          break;
       case 2:
          Serial.println("Error 2: DHT start condition 2 not met.");
          break;
       case 3:
          Serial.println("Error 3: DHT checksum error.");
          break;
       default:
          Serial.println("Error: Unrecognized code encountered.");
          break;
    }
    
    uint16_t crcData = CRC16((unsigned char*)data,dataLength);//get CRC DATA
    //Serial.println(crcData,HEX);
    
    Serial.print("Data to be sent(without CRC): ");
    
    int i;
    for(i = 0;i < dataLength; i++)
    {
        Serial.print(data[i],HEX);
        Serial.print(" ");
    }
    Serial.println();
        
    unsigned char sendBuf[50]={0};

    for(i = 0;i < dataLength;i++)
    {
        sendBuf[i] = data[i] ;
    }
    
    sendBuf[dataLength] = (unsigned char)crcData; // Add CRC to LoRa Data
    sendBuf[dataLength+1] = (unsigned char)(crcData>>8); // Add CRC to LoRa Data

    Serial.print("Data to be sent(with CRC):    ");
    for(i = 0;i < (dataLength +2); i++)
    {
        Serial.print(sendBuf[i],HEX);
        Serial.print(" ");
    }
    Serial.println();

    rf95.send(sendBuf, dataLength+2);//Send LoRa Data
     
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];//Reply data array
    uint8_t len = sizeof(buf);//reply data length

    if (rf95.waitAvailableTimeout(3000))// Check If there is reply in 3 seconds.
    {
        // Should be a reply message for us now   
        if (rf95.recv(buf, &len))//check if reply message is correct
       {
            if(buf[0] == node_id[0] && buf[1] == node_id[2] && buf[2] == node_id[2] ) // Check if reply message has the our node ID
           {
               pinMode(4, OUTPUT);
               digitalWrite(4, HIGH);
               Serial.print("Got Reply from Gateway: ");//print reply
               Serial.println((char*)buf);
              
               delay(400);
               digitalWrite(4, LOW); 
               //Serial.print("RSSI: ");  // print RSSI
               //Serial.println(rf95.lastRssi(), DEC);        
           }    
        }
        else
        {
           Serial.println("recv failed");//
           rf95.send(sendBuf, strlen((char*)sendBuf));//resend if no reply
        }
    }
    else
    {
        Serial.println("No reply, is LoRa gateway running?");//No signal reply
        rf95.send(sendBuf, strlen((char*)sendBuf));//resend data
    }
    delay(30000); // Send sensor data every 30 seconds
    Serial.println("");
}


void loop() {

HumiditySensor();
TempertureSensor(); 
int myBPM = pulseSensor.getBeatsPerMinute();  // Calls function on our pulseSensor object that returns BPM as an "int".
                                               // "myBPM" hold this BPM value now. 

if (pulseSensor.sawStartOfBeat()) {            // Constantly test to see if "a beat happened". 
  Serial.println("♥  A HeartBeat Happened ! "); // If test is "true", print a message "a heartbeat happened".
  Serial.print("BPM: ");                        // Print phrase "BPM: " 
  Serial.println(myBPM);                        // Print the value inside of myBPM. 
}

  delay(20);                    // considered best practice in a simple sketch.

}

void uploadData() {//Upload Data to ThingSpeak
  // form the string for the API header parameter:


  // form the string for the URL parameter, be careful about the required "
  String upload_url = "https://api.thingspeak.com/update?api_key=";
  upload_url += myWriteAPIString;
  upload_url += "&";
  upload_url += dataString;

  Console.println("Call Linux Command to Send Data");
  Process p;    // Create a process and call it "p", this process will execute a Linux curl command
  p.begin("curl");
  p.addParameter("-k");
  p.addParameter(upload_url);
  p.run();    // Run the process and wait for its termination

  Console.print("Feedback from Linux: ");
  // If there's output from Linux,
  // send it out the Console:
  while (p.available()>0) 
  {
    char c = p.read();
    Console.write(c);
  }
  Console.println("");
  Console.println("Call Finished");
  Console.println("####################################");
  Console.println("");
}


void TempertureSensor()
    if (rf95.waitAvailableTimeout(2000))// Listen Data from LoRa Node
    {
        uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];//receive data buffer
        uint8_t len = sizeof(buf);//data buffer length
        if (rf95.recv(buf, &len))//Check if there is incoming data
        {
            recdata( buf, len);
            Console.print("Get LoRa Packet: ");
            for (int i = 0; i < len; i++)
            {
                Console.print(buf[i],HEX);
                Console.print(" ");
            }
            Console.println();
            if(crcdata == recCRCData) //Check if CRC is correct
            { 
                if(buf[0] == 1 && buf[1] == 1 && buf[2] ==1) //Check if the ID match the LoRa Node ID
                {
                    uint8_t data[] = "   Server ACK";//Reply 
                    data[0] = buf[0];
                    data[1] = buf[1];
                    data[2] = buf[2];
                    rf95.send(data, sizeof(data));// Send Reply to LoRa Node
                    rf95.waitPacketSent();
                    int newData[4] = {0, 0, 0, 0}; //Store Sensor Data here
                    for (int i = 0; i < 4; i++)
                    {
                        newData[i] = buf[i + 3];
                    }
                    int hh = newData[0];
                    int hl = newData[1];
                    int th = newData[2];
                    int tl = newData[3];
                    Console.print("Get Temperature:");
                    Console.print(th);
                    Console.print(".");
                    Console.println(tl);
                    Console.print("Get Humidity:");
                    Console.print(hh);
                    Console.print(".");
                    Console.println(hl);
                                       
                    dataString ="field1=";
                    dataString += th;
                    dataString +=".";
                    dataString += tl;
                    dataString +="&field2=";
                    dataString += hh;
                    dataString +=".";
                    dataString += hl;
                                       
                    uploadData(); // 
                    dataString="";
                }
            } 
            else 
              Console.println(" CRC Fail");     
         }
         else
         {
              //Console.println("recv failed");
              ;
          }
     }
 
}
