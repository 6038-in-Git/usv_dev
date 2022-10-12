void setup() {
  // put your setup code here, to run once:

}

void loop() {
  if (ss.available() > 0)
        if (gps.encode(ss.read()))
          {
          temp = gps.location.lat();
          usv_lat_udp[0] = gps.location.lat()*1E-1;   temp = temp - usv_lat_udp[0]*1E1;     usv_lat_udp[0] = usv_lat_udp[0] + 48;
          usv_lat_udp[1] = gps.location.lat()*1E0;    temp = temp - usv_lat_udp[1]*1E0;     usv_lat_udp[1] = usv_lat_udp[0] + 48;
          usv_lat_udp[2] = 46; 
          usv_lat_udp[3] = gps.location.lat()*1E1;    temp = temp - usv_lat_udp[3]*1E-1;    usv_lat_udp[3] = usv_lat_udp[3] + 48;
          usv_lat_udp[4] = gps.location.lat()*1E2;    temp = temp - usv_lat_udp[4]*1E-2;    usv_lat_udp[4] = usv_lat_udp[4] + 48;
          usv_lat_udp[5] = gps.location.lat()*1E3;    temp = temp - usv_lat_udp[5]*1E-3;    usv_lat_udp[5] = usv_lat_udp[5] + 48;
          usv_lat_udp[6] = gps.location.lat()*1E4;    temp = temp - usv_lat_udp[6]*1E-4;    usv_lat_udp[6] = usv_lat_udp[6] + 48;
          usv_lat_udp[7] = gps.location.lat()*1E5;    temp = temp - usv_lat_udp[7]*1E-5;    usv_lat_udp[7] = usv_lat_udp[7] + 48;
          usv_lat_udp[8] = gps.location.lat()*1E6;    temp = temp - usv_lat_udp[8]*1E-6;    usv_lat_udp[8] = usv_lat_udp[8] + 48;
          usv_lat_udp[9] = gps.location.lat()*1E7;    temp = temp - usv_lat_udp[9]*1E-7;    usv_lat_udp[9] = usv_lat_udp[9] + 48;
          usv_lat_udp[10] = gps.location.lat()*1E8;   temp = temp - usv_lat_udp[10]*1E-8;   usv_lat_udp[10] = usv_lat_udp[10] + 48;
          usv_lat_udp[11] = gps.location.lat()*1E9;   temp = temp - usv_lat_udp[11]*1E-9;   usv_lat_udp[11] = usv_lat_udp[11] + 48;
    
          temp = gps.location.lng();
          usv_lng_udp[0] = gps.location.lng()*1E-2;   temp = temp - usv_lng_udp[0]*1E2;     usv_lng_udp[0] = usv_lng_udp[0] + 48;
          usv_lng_udp[1] = gps.location.lng()*1E-1;   temp = temp - usv_lng_udp[1]*1E1;     usv_lng_udp[1] = usv_lng_udp[1] + 48;
          usv_lng_udp[2] = gps.location.lng()*1E0;    temp = temp - usv_lng_udp[2]*1E0;     usv_lng_udp[2] = usv_lng_udp[2] + 48;
          usv_lng_udp[3] = 46; 
          usv_lng_udp[4] = gps.location.lng()*1E1;    temp = temp - usv_lng_udp[4]*1E-1;    usv_lng_udp[4] = usv_lng_udp[4] + 48;
          usv_lng_udp[5] = gps.location.lng()*1E2;    temp = temp - usv_lng_udp[5]*1E-2;    usv_lng_udp[5] = usv_lng_udp[5] + 48;
          usv_lng_udp[6] = gps.location.lng()*1E3;    temp = temp - usv_lng_udp[6]*1E-3;    usv_lng_udp[6] = usv_lng_udp[6] + 48;
          usv_lng_udp[7] = gps.location.lng()*1E4;    temp = temp - usv_lng_udp[7]*1E-4;    usv_lng_udp[7] = usv_lng_udp[7] + 48;
          usv_lng_udp[8] = gps.location.lng()*1E5;    temp = temp - usv_lng_udp[8]*1E-5;    usv_lng_udp[8] = usv_lng_udp[8] + 48;
          usv_lng_udp[9] = gps.location.lng()*1E6;    temp = temp - usv_lng_udp[9]*1E-6;    usv_lng_udp[9] = usv_lng_udp[9] + 48;
          usv_lng_udp[10] = gps.location.lng()*1E7;   temp = temp - usv_lng_udp[10]*1E-7;   usv_lng_udp[10] = usv_lng_udp[10] + 48;
          usv_lng_udp[11] = gps.location.lng()*1E8;   temp = temp - usv_lng_udp[11]*1E-8;   usv_lng_udp[11] = usv_lng_udp[11] + 48;
          usv_lng_udp[12] = gps.location.lng()*1E9;   temp = temp - usv_lng_udp[12]*1E-9;   usv_lng_udp[12] = usv_lng_udp[12] + 48;
          }
        else
        {
          usv_lat_udp[0] = 46;
          usv_lat_udp[1] = 46;
          usv_lat_udp[2] = 46; 
          usv_lat_udp[3] = 46;
          usv_lat_udp[4] = 46;
          usv_lat_udp[5] = 46;
          usv_lat_udp[6] = 46;
          usv_lat_udp[7] = 46;
          usv_lat_udp[8] = 46;
          usv_lat_udp[9] = 46;
          usv_lat_udp[10] = 46;
          usv_lat_udp[11] = 46;

          usv_lng_udp[0] = 46;
          usv_lng_udp[1] = 46;
          usv_lng_udp[2] = 46; 
          usv_lng_udp[3] = 46;
          usv_lng_udp[4] = 46;
          usv_lng_udp[5] = 46;
          usv_lng_udp[6] = 46;
          usv_lng_udp[7] = 46;
          usv_lng_udp[8] = 46;
          usv_lng_udp[9] = 46;
          usv_lng_udp[10] = 46;
          usv_lng_udp[11] = 46;
          usv_lng_udp[12] = 46;
   
        }
}




            temp = gps.location.lat();
            usv_lat_udp[0] = gps.location.lat()*1E-1;   temp = temp - usv_lat_udp[0]*1E1;     usv_lat_udp[0] = usv_lat_udp[0] + 48;
            usv_lat_udp[1] = gps.location.lat()*1E0;    temp = temp - usv_lat_udp[1]*1E0;     usv_lat_udp[1] = usv_lat_udp[0] + 48;
            usv_lat_udp[2] = 46; 
            usv_lat_udp[3] = gps.location.lat()*1E1;    temp = temp - usv_lat_udp[3]*1E-1;    usv_lat_udp[3] = usv_lat_udp[3] + 48;
            usv_lat_udp[4] = gps.location.lat()*1E2;    temp = temp - usv_lat_udp[4]*1E-2;    usv_lat_udp[4] = usv_lat_udp[4] + 48;
            usv_lat_udp[5] = gps.location.lat()*1E3;    temp = temp - usv_lat_udp[5]*1E-3;    usv_lat_udp[5] = usv_lat_udp[5] + 48;
            usv_lat_udp[6] = gps.location.lat()*1E4;    temp = temp - usv_lat_udp[6]*1E-4;    usv_lat_udp[6] = usv_lat_udp[6] + 48;
            usv_lat_udp[7] = gps.location.lat()*1E5;    temp = temp - usv_lat_udp[7]*1E-5;    usv_lat_udp[7] = usv_lat_udp[7] + 48;
            usv_lat_udp[8] = gps.location.lat()*1E6;    temp = temp - usv_lat_udp[8]*1E-6;    usv_lat_udp[8] = usv_lat_udp[8] + 48;
            usv_lat_udp[9] = gps.location.lat()*1E7;    temp = temp - usv_lat_udp[9]*1E-7;    usv_lat_udp[9] = usv_lat_udp[9] + 48;
            usv_lat_udp[10] = gps.location.lat()*1E8;   temp = temp - usv_lat_udp[10]*1E-8;   usv_lat_udp[10] = usv_lat_udp[10] + 48;
            usv_lat_udp[11] = gps.location.lat()*1E9;   temp = temp - usv_lat_udp[11]*1E-9;   usv_lat_udp[11] = usv_lat_udp[11] + 48;



            temp = gps.location.lng();
            usv_lng_udp[0] = gps.location.lng()*1E-2;   temp = temp - usv_lng_udp[0]*1E2;     usv_lng_udp[0] = usv_lng_udp[0] + 48;
            usv_lng_udp[1] = gps.location.lng()*1E-1;   temp = temp - usv_lng_udp[1]*1E1;     usv_lng_udp[1] = usv_lng_udp[1] + 48;
            usv_lng_udp[2] = gps.location.lng()*1E0;    temp = temp - usv_lng_udp[2]*1E0;     usv_lng_udp[2] = usv_lng_udp[2] + 48;
            usv_lng_udp[3] = 46; 
            usv_lng_udp[4] = gps.location.lng()*1E1;    temp = temp - usv_lng_udp[4]*1E-1;    usv_lng_udp[4] = usv_lng_udp[4] + 48;
            usv_lng_udp[5] = gps.location.lng()*1E2;    temp = temp - usv_lng_udp[5]*1E-2;    usv_lng_udp[5] = usv_lng_udp[5] + 48;
            usv_lng_udp[6] = gps.location.lng()*1E3;    temp = temp - usv_lng_udp[6]*1E-3;    usv_lng_udp[6] = usv_lng_udp[6] + 48;
            usv_lng_udp[7] = gps.location.lng()*1E4;    temp = temp - usv_lng_udp[7]*1E-4;    usv_lng_udp[7] = usv_lng_udp[7] + 48;
            usv_lng_udp[8] = gps.location.lng()*1E5;    temp = temp - usv_lng_udp[8]*1E-5;    usv_lng_udp[8] = usv_lng_udp[8] + 48;
            usv_lng_udp[9] = gps.location.lng()*1E6;    temp = temp - usv_lng_udp[9]*1E-6;    usv_lng_udp[9] = usv_lng_udp[9] + 48;
            usv_lng_udp[10] = gps.location.lng()*1E7;   temp = temp - usv_lng_udp[10]*1E-7;   usv_lng_udp[10] = usv_lng_udp[10] + 48;
            usv_lng_udp[11] = gps.location.lng()*1E8;   temp = temp - usv_lng_udp[11]*1E-8;   usv_lng_udp[11] = usv_lng_udp[11] + 48;
            usv_lng_udp[12] = gps.location.lng()*1E9;   temp = temp - usv_lng_udp[12]*1E-9;   usv_lng_udp[12] = usv_lng_udp[12] + 48;
