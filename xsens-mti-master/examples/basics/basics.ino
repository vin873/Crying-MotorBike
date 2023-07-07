// This basic example uses 'Serial' which maps to hardware serial on an Uno.
// For other hardware targets, modify the Serial usage as needed.

// Use MTManager to configure outputs: Quaternion, Acceleration, pressure, temperature.
// Modify the baud-rate in this sketch to match the configured rate in MTManager.

#include "xsens_mti.h"      // Main library
#include "xsens_utility.h"  // Needed for quaternion conversion function

// Cache a copy of IMU data
float    temperature     = 0;       // in degress celcius
uint32_t pressure        = 0;       // in pascals
float    euler_pry[3]    = { 0 };   // -180 to +180 degress
float    acceleration[3] = { 0 };   // in m/s^2
int flag=0,caliTimes=200;
float currTotalPitch=0,currTotalRoll=0,currTotalYaw=0,offsetPitch=0,offsetRoll=0,offsetYaw=0;
// Callback function used by the library
void imu_callback( XsensEventFlag_t event, XsensEventData_t *mtdata );
// The library holds state and pointers to callbacks in this structure
//   - macro used to simplfiy instantiation, read write_config example for more 
xsens_interface_t imu_interface = XSENS_INTERFACE_RX( &imu_callback );

// Provide fallback LED pin if the selected board doesn't have one
#ifndef LED_BUILTIN
    #define LED_BUILTIN 13
#endif

// Normal arduino setup/loop functions
void setup( void )
{
    Serial.begin( 115200 );
    Serial1.begin( 115200 );
    pinMode( LED_BUILTIN, OUTPUT );
    UART_Send_GyroBiasEst();
    for(int i=0;i<100;i++){
      UART_Send_ResetHeading();
      }
}

void loop( void )
{
    // Inbound serial data needs to be parsed
    // When a valid packet is decoded, imu_callback() will fire
    while( Serial1.available() > 0 )  
    {  
        xsens_mti_parse( &imu_interface, Serial1.read() );
    }

    // Light goes high if the IMU pitch exceeds 10 degrees
    digitalWrite( LED_BUILTIN, (euler_pry[0] > 10.0f) );
}

float crossProduct(float first[],float second[]){
  float result[3],angle; 
  result[0]=first[1]*second[2]-first[2]*second[1];
  result[1]=first[2]*second[0]-first[0]*second[2];
  result[2]=first[0]*second[1]-first[1]*second[0];
  angle=acos(result[2]/(pow((pow(result[0],2)+pow(result[1],2)+pow(result[2],2)),0.5)));
  angle*= (180.0 / PI);
  return angle;
  }
// Called when the library decoded an inbound packet
//   - If the packet was an MData2 frame (which contains packed motion data)
//   - the callback is called once for each sub-field
void imu_callback( XsensEventFlag_t event, XsensEventData_t *mtdata )
{
    // The library provides a pointer to the a union containing decoded data
    // Use XsensEventFlag_t to determine what kind of packet arrived, 
    // then copy data from the union as needed.

    // union
    // {
    //     uint8_t u1;
    //     uint16_t u2;
    //     uint32_t u4;
    //     float    f4;
    //     float    f4x2[2];
    //     float    f4x3[3];
    //     float    f4x4[4];
    //     float    f4x9[9];
    // } data;

    switch( event )
    {
        case XSENS_EVT_QUATERNION:
            // We use the type field of XsensEventData_t as a doublecheck before
            // blindly copying bytes out of the union
            if( mtdata->type == XSENS_EVT_TYPE_FLOAT4 )
            {
                // Convert the quaternion to euler angles
                xsens_quaternion_to_euler( mtdata->data.f4x4, euler_pry );
                float first[3]={cos(euler_pry[0]),0,sin(euler_pry[0])},second[3]={0,cos(euler_pry[1]),sin(euler_pry[1])};
                float inclination=(crossProduct(first,second));
                // Convert from radians to degrees
                euler_pry[0] *= (180.0 / PI);
                euler_pry[1] *= (180.0 / PI);
                euler_pry[2] *= (180.0 / PI);
                Serial.print("Inclination : ");
                Serial.print(inclination,5);
                Serial.print("  p : ");
                Serial.print(euler_pry[0],5);
                Serial.print("  r : ");
                Serial.print(euler_pry[1],5);
                Serial.print("  y : ");
                Serial.println(euler_pry[2],5);
            }
            break;

        case XSENS_EVT_ACCELERATION:
            if( mtdata->type == XSENS_EVT_TYPE_FLOAT3 )
            {
                acceleration[0] = mtdata->data.f4x3[0];
                acceleration[1] = mtdata->data.f4x3[1];
                acceleration[2] = mtdata->data.f4x3[2];
                Serial.print("Acc_x : ");
                Serial.print(acceleration[0]);
                Serial.print("  Acc_y : ");
                Serial.print(acceleration[1]);
                Serial.print("  Acc_z : ");
                Serial.println(acceleration[2]);
            }
            break;

        case XSENS_EVT_PRESSURE:
            if( mtdata->type == XSENS_EVT_TYPE_U32 )
            {
                pressure = mtdata->data.u4;
            }
            break;

        case XSENS_EVT_TEMPERATURE:
            if( mtdata->type == XSENS_EVT_TYPE_FLOAT )
            {
                temperature = mtdata->data.f4;
            }
            break;
    }
}

void UART_Send(uint8_t u8_data) {
  Serial1.write(u8_data);
}

void UART_Send_GyroBiasEst(){
  UART_Send(0xFA); 
  UART_Send(0xFF); 
  UART_Send(0x22); 
  UART_Send(0x02); 
  UART_Send(0x00); 
  UART_Send(0x04); 
  UART_Send(0xD9); 
}

void UART_Send_ResetHeading(){
  UART_Send(0xFA); 
  UART_Send(0xFF); 
  UART_Send(0xA4); 
  UART_Send(0x02); 
  UART_Send(0x00); 
  UART_Send(0x01); 
  UART_Send(0x5A); 
  
  UART_Send(0xFA); 
  UART_Send(0xFF); 
  UART_Send(0xA4); 
  UART_Send(0x02); 
  UART_Send(0x00); 
  UART_Send(0x04); 
  UART_Send(0x57); 

  UART_Send(0xFA); 
  UART_Send(0xFF); 
  UART_Send(0xA4); 
  UART_Send(0x02); 
  UART_Send(0x00); 
  UART_Send(0x00); 
  UART_Send(0x5B);   
}
