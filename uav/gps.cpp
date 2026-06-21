
// #include "gps.h"

// TinyGPSPlus gps;
// HardwareSerial SerialGPS(1);

// double current_lat = 0;
// double current_lon = 0;

// double home_lat = 0;
// double home_lon = 0;

// float gps_x = 0;
// float gps_y = 0;

// float gps_vx = 0;
// float gps_vy = 0;

// bool gps_fix = false;
// bool home_set = false;

// void gpsSetup()
// {
//     SerialGPS.begin(115200, SERIAL_8N1,
//                     GPS_RX_PIN,
//                     GPS_TX_PIN);
// }

// void gpsUpdate()
// {
//     while (SerialGPS.available())
//     {
//         gps.encode(SerialGPS.read());
//     }

//     if (gps.location.isUpdated())
//     {
//         gps_fix = true;

//         current_lat = gps.location.lat();
//         current_lon = gps.location.lng();

//         if (!home_set)
//         {
//             home_lat = current_lat;
//             home_lon = current_lon;

//             home_set = true;
//         }

//         //--------------------------------
//         // local coordinate (meter)
//         //--------------------------------

//         gps_x =
//             (current_lon - home_lon)
//             *111320.0f
//             *cos(home_lat * DEG_TO_RAD);

//         gps_y =
//             (current_lat - home_lat)
//             *111320.0f;

//         //--------------------------------
//         // velocity
//         //--------------------------------

//         float speed = gps.speed.mps();

//         float course =
//             gps.course.deg()
//             * DEG_TO_RAD;

//         gps_vx = speed * sin(course);
//         gps_vy = speed * cos(course);
//     }
// }
