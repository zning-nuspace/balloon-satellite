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