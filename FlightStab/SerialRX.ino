/* FlightStab **************************************************************************************************/

/***************************************************************************************************************
*
* SERIAL RX
* see http://paparazzi.github.io/docs/latest/stm32_2subsystems_2radio__control_2spektrum__arch_8c_source.html
*
***************************************************************************************************************/

#if defined(SERIALRX_SPEKTRUM)
int8_t rshift;
#endif

#if defined(SERIALRX_SBUS)
// The following value is added to the received pulse count
// to make the center pulse width = 1500 when the TX output is 1500
// TODO(noobee): i guess we would need to make this configurable..
#define SBUS_OFFSET 1009 // 1009 for Futaba X8R, 1003 for Taranis FRSKY X8R, 984 for Orange R800x
#endif
      
#if (defined(SERIALRX_SPEKTRUM) || defined(SERIALRX_SBUS) || defined(SERIALRX_SUMD))
  #if defined(NANOWII)
    HardwareSerial *pSerial = &Serial1; // TODO: hardcoded for NanoWii for now
  #else
    HardwareSerial *pSerial = &Serial; 
  #endif // NANOWI
  
  //jrb add for debug  
  #if defined(SERIAL_DEBUG) && 0
    volatile int8_t RXcount;
    int8_t PrintIndex;
    uint16_t work;
  #endif
  
#if defined(SERIALRX_SUMD)
  #define SUMD_BAUD		115200L

  #define SUMD_START_CHAR1	0xa8
  #define SUMD_START_CHAR2	0x01

  #define SUMD_WAIT_SYNC1	0x00
  #define SUMD_WAIT_SYNC2	0x01
  #define SUMD_WAIT_NB_CHANNEL	0x02
  #define SUMD_WAIT_DATA	0x03

  #define POLYNOM               0x11021
#endif

#if defined(SERIALRX_SUMD) //Vars for Graupner SUMD RX Configs
    typedef struct {
      uint8_t state;
      uint8_t nb_channel;
      uint8_t dataCount;
      uint8_t rawBuf[23] ;
    } sumdStruct_t;
    static sumdStruct_t sumd_data;
#endif


  void serialrx_init()
  {
    #if defined (SERIALRX_SPEKTRUM)
      pSerial->begin(115200L);
      rshift = cfg.serialrx_spektrum_levels == SERIALRX_SPEKTRUM_LEVELS_1024 ? 10 : 11; // 1024->10, 2048->11
    #endif
    #if defined (SERIALRX_SBUS)
      pSerial->begin(100000L, SERIAL_8E2);
    #endif
    //Init Port for SUMD
    #if defined(SERIALRX_SUMD)
      pSerial->begin(SUMD_BAUD);
      sumd_data.state = SUMD_WAIT_SYNC1;
    #endif
  }

  bool serialrx_update()
  {
    #if !defined(SERIALRX_SUMD)         //SUMD doesn't use these
      static uint8_t buf[25];
      static int8_t index = 0;
    #endif // SERIALRX_SUMD
    #if defined (SERIALRX_SBUS)
      bool sbus_return = false;
    #endif // SERIALRX_SBUS  
    
  #if defined (SERIALRX_SPEKTRUM) 	// Used only for Spektrum    
    static uint32_t last_rx_time;
    uint32_t t;
  #endif
  
  #if defined (SERIALRX_SUMD)
    bool sumd_return = false;
  #endif // SERIALRX_SBUS

  //jrb add for debug
  #if defined(SERIAL_DEBUG) && 0
    Serial.println("Serial update");
  #endif
  
  while (pSerial->available()) {
    uint8_t ch = pSerial->read();

  #if defined (SERIALRX_SPEKTRUM)    
    #warning SERIALRX_SPEKTRUM defined // emit device name 
  //jrb add for debug    
      #if defined(SERIAL_DEBUG) && 0
        Serial.println("Serial Spektrum");
      #endif

      t = micros1();
      // we assume loop() calls to serialrx_update() in << 7ms intervals
      if ((int32_t)(t - last_rx_time) > 7000) {
        index = 0; // found pause before new frame, resync
      }
      last_rx_time = t;
      buf[index++] = ch;
    
      if (index >= 15) {

/*jrb        
//  Satellites alone never return frame size information in buf[1] the best I can tell
//  Data always seems to be 2048 bits, even with 9XR transmitter.  The following
//  should be removed and 10 or 11 bit format saved as configuration data
//          if ((buf[2] & 0x80) == 0x00) { 
//          // single frame type or 1st frame of two, contains "transmitter type"
//            rshift = (buf[1] & 0x10) ? 11 : 10; // 11 or 10 bit data
//          }
jrb*/ 
 
        if (rshift > 0) {
          // 10 bit == f  0 c3 c2 c1  c0 d9 d8 d7 d6 d5 d4 d3 d2 d1 d0
          // 11 bit == f c3 c2 c1 c0 d10 d9 d8 d7 d6 d5 d4 d3 d2 d1 d0        
          for (int8_t i=2; i<2+2*7; i+=2) {
            uint16_t w = ((uint16_t)buf[i] << 8) | (uint16_t)buf[i+1];
            int8_t chan = (w >> rshift) & 0xf;
            if (chan < rx_chan_size) {
              *rx_chan[chan] = ((w << (11 - rshift) & 0x7ff) - 1024 + 1500); // scale to 11 bits 1024 +/- 684;
            }          
          }
        }  
        index = 0;

  //jrb add for debug       
  #if defined(SERIAL_DEBUG) && 0   
      if (RXcount > 100)
      {
        Serial.print("rshift = ");
        Serial.print(rshift);
        if ((buf[2] & 0x80) == 0x00) { 
          Serial.println(" Low Channels");
        }
        else  {
          Serial.println(" High Channels");  
        }
        
        for (PrintIndex = 0; PrintIndex < 8; PrintIndex++)
        {
          Serial.print(*rx_chan[cfg.serialrx_order-2][PrintIndex]); Serial.print(' ');
        }
        Serial.println(' '); 
              
        for (PrintIndex = 0; PrintIndex < 16; PrintIndex += 2)
        { 
          work = (buf[PrintIndex]<<8)+ buf[PrintIndex + 1];
          Serial.print(work,HEX); Serial.print(' ');
        }
        Serial.println(' '); 
       RXcount = 0; 
      }
      RXcount++; 
  #endif
      return (true);
    }
    else
      return (false);
  }  
  #endif // SERIALRX_SPEKTRUM



  #if defined (SERIALRX_SBUS)
    #warning SERIALRX_SBUS defined // emit device name 
    //jrb add for debug    
      #if defined(SERIAL_DEBUG) && 0
        Serial.println("Serial S.BUS");
      #endif
      
      if (index == 0 && ch != 0x0f) { // SBUS_SYNCBYTE
        break;
      }
      buf[index++] = ch;
      if (index >= 25) {
        volatile int16_t **p = rx_chan;
        uint8_t adj_index;
        
        // Only process first 8 channels
        *p[0] = (((((uint16_t)buf[1]  >> 0) | ((uint16_t)buf[2]  << 8)) & 0x7ff) >> 1) + SBUS_OFFSET;
        *p[1] = (((((uint16_t)buf[2]  >> 3) | ((uint16_t)buf[3]  << 5)) & 0x7ff) >> 1) + SBUS_OFFSET; 
        *p[2] = (((((uint16_t)buf[3]  >> 6) | ((uint16_t)buf[4]  << 2) | ((uint16_t)buf[5] << 10)) & 0x7ff) >> 1) + SBUS_OFFSET; 
        *p[3] = (((((uint16_t)buf[5]  >> 1) | ((uint16_t)buf[6]  << 7)) & 0x7ff) >> 1) + SBUS_OFFSET; 
        *p[4] = (((((uint16_t)buf[6]  >> 4) | ((uint16_t)buf[7]  << 4)) & 0x7ff) >> 1) + SBUS_OFFSET; 
        *p[5] = (((((uint16_t)buf[7]  >> 7) | ((uint16_t)buf[8]  << 1) | ((uint16_t)buf[9] << 9)) & 0x7ff) >> 1) + SBUS_OFFSET;
        *p[6] = (((((uint16_t)buf[9]  >> 2) | ((uint16_t)buf[10] << 6)) & 0x7ff) >> 1) + SBUS_OFFSET; 
        *p[7] = (((((uint16_t)buf[10] >> 5) | ((uint16_t)buf[11] << 3)) & 0x7ff) >> 1) + SBUS_OFFSET;
       
       // For some reason the SBUS data provides only about 75% of the actual RX output pulse width
       // Adjust the actual value by +/-25%.  Sign determined by pulse width above or below center of 1520us 
       for(adj_index=0; adj_index<rx_chan_size; adj_index++)
       {
       	if (*p[adj_index] < 1520)
       	  *p[adj_index] -= (1520 - *p[adj_index]) >> 2;		
       	else	
       	  *p[adj_index] += (*p[adj_index] - 1520) >> 2;
       }	 
        index = 0;
        sbus_return = true;
      }

  //jrb add for debug  
    #if defined(SERIAL_DEBUG) && 0   
      if (RXcount > 100)
      {
        for (index = 0; index < 8; index++)
        {
          Serial.print(*rx_chan[cfg.serialrx_order-2][index]); Serial.print(' ');
        }
        Serial.println(' '); 
       RXcount = 0; 
      }
      RXcount++; 
    #endif
  }
  return (sbus_return); 
  #endif // SERIALRX_SBUS
  
  
  #if defined (SERIALRX_SUMD)
    #warning SERIALRX_SUMD defined // emit device name 
        
    #if defined(SERIAL_DEBUG) && 0
      Serial.println("Serial Graupner SUMD 8 channels MAX!");
    #endif
   
  switch (sumd_data.state) {
    case SUMD_WAIT_SYNC1:
      if (ch == SUMD_START_CHAR1) {
	sumd_data.state = SUMD_WAIT_SYNC2;
	sumd_data.dataCount = 0;
	sumd_data.nb_channel = 0 ;
      }
      break;
    case SUMD_WAIT_SYNC2:
      if (ch == SUMD_START_CHAR2) {
        sumd_data.state = SUMD_WAIT_NB_CHANNEL;
      }
      break;
    case SUMD_WAIT_NB_CHANNEL:
      sumd_data.nb_channel = ch * 2 + 2 ;
      sumd_data.state = SUMD_WAIT_DATA;
      sumd_data.dataCount = 0;
      break;
    case SUMD_WAIT_DATA:
      sumd_data.rawBuf[sumd_data.dataCount++] = ch;
      if (sumd_data.dataCount == sumd_data.nb_channel ) {
        sumd_data.state = SUMD_WAIT_SYNC1;
        sumd_return = sumdDecode();
      }
      break;
    default:
      sumd_data.state = SUMD_WAIT_SYNC1;
      break;
   }
 }
 return(sumd_return);
 #endif // SERIALRX_SUMD
}


#if defined (SERIALRX_SUMD)
boolean sumdDecode(void) {
  // calculate CRC16 XMODEM
  int crc = 0xA604 ;                                            /* CRC of three first bytes = A8 01 08 (header: two byte, nbr channels: one byte) */
  for (int num=0; num < sumd_data.nb_channel - 2  ; num++) {    /* Step through bytes in memory */
    crc = crc ^ (sumd_data.rawBuf[num] << 8);                   /* Fetch byte from memory, XOR into CRC top byte*/
    for (int i=0; i<8; i++) {                                   /* Prepare to rotate 8 bits */
       if (crc & 0x8000)                                        /* b15 is set... */
         crc = (crc << 1) ^ POLYNOM;                            /* rotate and XOR with XMODEM polynomic */
       else                                                     /* b15 is clear... */
         crc = crc << 1;              			        /* just rotate */
     }                                                          /* Loop for 8 bits */
  }
  int sumd_crc = ( sumd_data.rawBuf[16] << 8 ) + sumd_data.rawBuf[17] ;
  /*  @100% Min 1100ms->8800, Mid 1500ms->12000, Max 1900ms->15200  */
  /*  @50%  Min 1300, Mid 1500, Max 1700  */
  /*  @150% Min  900, Mid 1500, Max 2100  */
  //serialrx_order: R E T A 1 a 2 f
  //                0 1 2 3 4 5 6 7
  if (crc == sumd_crc) {     
    volatile int16_t **p = rx_chan;
    *p[2] = ( int16_t ) ( ( ( sumd_data.rawBuf[0] << 8 ) + sumd_data.rawBuf[1] ) >> 3 );    //Throttle S1
    *p[3] = ( int16_t ) ( ( ( sumd_data.rawBuf[2] << 8 ) + sumd_data.rawBuf[3] ) >> 3 );    //Aileron  S2
    *p[1] = ( int16_t ) ( ( ( sumd_data.rawBuf[4] << 8 ) + sumd_data.rawBuf[5] ) >> 3 );    //Elevator S3
    *p[0] = ( int16_t ) ( ( ( sumd_data.rawBuf[6] << 8 ) + sumd_data.rawBuf[7] ) >> 3 );    //Rudder   S4
    *p[5] = ( int16_t ) ( ( ( sumd_data.rawBuf[8] << 8 ) + sumd_data.rawBuf[9] ) >> 3 );    //Aileron  S5
    *p[7] = ( int16_t ) ( ( ( sumd_data.rawBuf[10] << 8 ) + sumd_data.rawBuf[11] ) >> 3 );  //Flaps    S6
    *p[4] = ( int16_t ) ( ( ( sumd_data.rawBuf[12] << 8 ) + sumd_data.rawBuf[13] ) >> 3 );  //Extra1   S7
    *p[6] = ( int16_t ) ( ( ( sumd_data.rawBuf[14] << 8 ) + sumd_data.rawBuf[15] ) >> 3 );  //Extra2   S8
    return true;
  }
  else {
    /* CRC Error */
    return false;
  }
}
#endif // SERIALRX_SUMD

#endif // SERIALRX_SPEKTRUM || SERIALRX_SBUS || SERIALRX_SUMD 
