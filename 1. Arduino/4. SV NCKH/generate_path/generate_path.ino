void setup() {
    // put your setup code here, to run once:
    
    Serial.begin(9600);
    const double k_fix_lat = 113894;
    const double k_fix_lng = 104225;
    
    double  target_lat_1 =  21.010678;
    double  target_lat_2 =  21.010157;
    double  target_lat_3 =  21.010080;
    double  target_lat_4 =  21.010681;
    double  target_lng_1 =  105.844636;
    double  target_lng_2 =  105.84469;
    double  target_lng_3 =  105.844103;
    double  target_lng_4 =  105.844094;
    
    double distance_12 = pow(pow(k_fix_lat*(target_lat_1 - target_lat_2) , 2)+pow(k_fix_lng*(target_lng_1 - target_lng_2) , 2) ,0.5);
    //double distance_34 = pow(pow(k_fix_lat*(target_lat_3 - target_lat_4) , 2)+pow(k_fix_lng*(target_lng_3 - target_lng_4) , 2) ,0.5);
    uint8_t dis_step = 5;   //khoảng cách giữa 2 đường (m)
    uint8_t num_step_int = distance_12/dis_step;
    double num_step_double = distance_12/(double) dis_step;
    
    double delta_lat_step_12 = ((target_lat_2 - target_lat_1)* ((double)num_step_int) / num_step_double)/((double)num_step_int);
    double delta_lng_step_12 = ((target_lng_2 - target_lng_1)* ((double)num_step_int) / num_step_double)/((double)num_step_int);
    
    double delta_lat_step_34 = ((target_lat_4 - target_lat_3)* ((double)num_step_int) / num_step_double)/((double)num_step_int);
    double delta_lng_step_34 = ((target_lng_4 - target_lng_3)* ((double)num_step_int) / num_step_double)/((double)num_step_int);
    
    double target_lat_list_12[100];
    for(int i = 0; i < num_step_int + 1; i++)
    {
    target_lat_list_12[i] = target_lat_1 + delta_lat_step_12*(double)i;
    }
    target_lat_list_12[num_step_int + 1] = target_lat_2;
    
    
    double target_lng_list_12[100];
    for(int i = 0; i < num_step_int + 1; i++)
    {
    target_lng_list_12[i] = target_lng_1 + delta_lng_step_12*(double)i;
    }
    target_lng_list_12[num_step_int + 1] = target_lng_2;
    
    double target_lat_list_34[100];
    for(int i = 0; i < num_step_int + 1; i++)
    {
    target_lat_list_34[i] = target_lat_3 + delta_lat_step_34*(double)i;
    }
    target_lat_list_34[num_step_int + 1] = target_lat_4;
    
    double target_lng_list_34[100];
    for(int i = 0; i < num_step_int + 1; i++){
    target_lng_list_34[i] = target_lng_3 + delta_lng_step_34*(double)i;
    }
    target_lng_list_34[num_step_int + 1] = target_lng_4;
    
    double target_lat_list[200];
    double target_lng_list[200];
    
    for (int i = 0; i < (num_step_int + 2)/2+1; i ++) 
    {
    target_lat_list[4*i] = target_lat_list_12[2*i];
    target_lat_list[4*i+1] = target_lat_list_12[2*i+1];
    target_lat_list[4*i+2] = target_lat_list_34[2*i];
    target_lat_list[4*i+3] = target_lat_list_34[2*i+1];
    }
    
    for(int i = 0; i < 2*(num_step_int + 2) + 1; i++) 
    {
    Serial.println(target_lat_list[i], 6);
    }
    
    for (int i = 0; i < (num_step_int + 2)/2+1; i ++) 
    {
    target_lng_list[4*i] = target_lng_list_12[2*i];
    target_lng_list[4*i+1] = target_lng_list_12[2*i+1];
    target_lng_list[4*i+2] = target_lng_list_34[2*i];
    target_lng_list[4*i+3] = target_lng_list_34[2*i+1];
    }
    
    for(int i = 0; i < 2*(num_step_int + 2) + 1; i++) 
    {
    Serial.println(target_lng_list[i], 6);
    }
    
    /*//Serial.print(" target_lat_list_12: ");
    //Serial.print(target_lat_list_12[2],6);
    Serial.print(" num_step: ");
    Serial.print(num_step_int);
    Serial.print(" distance_12: ");
    Serial.print(distance_12);
    Serial.print(" target_lat_1: ");
    Serial.print(target_lat_1, 6);
    // Serial.print(" distance_34: ");
    //Serial.print(distance_34);*/

}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println('...');
}
