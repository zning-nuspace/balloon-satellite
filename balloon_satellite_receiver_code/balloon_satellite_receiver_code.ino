/***************************************************************************

  _   _           ____                                
 | \ | |  _   _  / ___|   _ __     __ _    ___    ___ 
 |  \| | | | | | \___ \  | '_ \   / _` |  / __|  / _ \
 | |\  | | |_| |  ___) | | |_) | | (_| | | (__  |  __/
 |_| \_|  \__,_| |____/  | .__/   \__,_|  \___|  \___|
                         |_|                          
  (C)2022 NuSpace Pte Ltd
Description:
    Balloon Satellite Receiver Code. 
Author: Hubert Khoo Hui Bo
Date Written: 24th November 2022

***************************************************************************/


#include <SoftwareSerial.h>

SoftwareSerial HC12(10, 11); // HC-12 TX Pin, HC-12 RX Pin

union temperature {
    uint8_t     bytes[sizeof( float )];
    float       temp;
};

temperature uni;

void setup() {
  Serial.begin(9600);             // Serial port to computer
  HC12.begin(9600);               // Serial port to HC12

}

void loop( void )
{
    byte
        ch,
        idx;
    bool
        done;
        
    if( HC12.available() > 0 )
    {
        if( HC12.read() == '>' )
        {
            done = false;
            idx = 0;
            while( !done )
            {
                if( HC12.available() > 0 )
                {
                    ch = HC12.read();
                    if( ch == '<' )
                        done = true;
                    else
                    {
                        if( idx < sizeof( float ) )
                            uni.bytes[idx++] = ch;
                        
                    }//else
                
                }//if
                   
            }//while

            Serial.print( "Float value received: " ); 
            Serial.println( uni.temp, 4 );
            
        }//if
        
    }//if
    
}//loop


/* If you just wanted simple typing communication between the two, it's here. 
Comment out the loop above and flash this code onto two separate arduinos.

void loop() {
  while (HC12.available()) {        // If HC-12 has data
    Serial.write(HC12.read());      // Send the data to Serial monitor
  }
  while (Serial.available()) {      // If Serial monitor has data
    HC12.write(Serial.read());      // Send that data to HC-12
  }
}
*/