  double  target_lat_1 = 0;
  double  target_lat_2 = 0;
  double  target_lat_3 = 0;
  double  target_lat_4 = 0;
  double  target_lng_1 = 0;
  double  target_lng_2 = 0;
  double  target_lng_3 = 0;
  double  target_lng_4 = 0;

  const double k_fix_lat = 113894;
  const double k_fix_lng = 104225;

  double target_lat = 0;
  double target_lng = 0;

  float distance_12 = 0;
  const float step_12 = 5;
  int step_no = 0;

void setup() {

  target_lat_1 = 21.010678;
  target_lat_2 = 21.010157;
  target_lat_3 = 21.010080;
  target_lat_4 = 21.010681;
  target_lng_1 = 105.844636;
  target_lng_2 = 105.84469;
  target_lng_3 = 105.844103;
  target_lng_4 = 105.844094;

  distance_12 = get_distance (target_lat_1, target_lat_2, target_lng_1, target_lng_2);
  step_no = get_step_no(distance_12, step_12);
  Serial.begin(9600);
  Serial.print("step_no: ");
  Serial.print(step_no);
  Serial.println("; ");

}

void loop() {

  for (int i = 1; i<=(step_no+1)*2; i++)
  {
    target_lat = get_target_list(i, step_no, target_lat_1, target_lat_2, target_lat_3, target_lat_4);
    target_lng = get_target_list(i, step_no, target_lng_1, target_lng_2, target_lng_3, target_lng_4);
    Serial.print("target_lat: ");
    Serial.print(target_lat,6);
    Serial.print("; ");
    Serial.print("target_lng: ");
    Serial.print(target_lng,6);
    Serial.println("; ");
  }
  Serial.println("~~~~~~~~~~~~~~~");
}

float get_distance (double _target_lat_1, double _target_lat_2, double _target_lng_1, double _target_lng_2)
{
  float _distance;
  _distance = pow(pow(k_fix_lat*(_target_lat_2 - _target_lat_1) , 2) + pow(k_fix_lng*(target_lng_1 - target_lng_2) , 2) ,0.5);
  return _distance;
}

int get_step_no (float _distance, float _step_size)
{
  int _step_no;
  _step_no = _distance/_step_size;
  return _step_no;
}

double get_target (int i, int _step_no, double _target_1, double _target_2)
{
  double _target;
  double _step;

  _step = (_target_2 - _target_1)/_step_no;
  _target = _target_1 + _step*i;

  return _target;
}

double get_target_list (int i, int _step_no, double _target_1, double _target_2, double _target_3, double _target_4)
{
  double _target_list = 0;
  int k_int = 0;
  int mod_4 = 0;
  k_int = i/4;
  mod_4 = i%4;

  if (mod_4 == 0)
    _target_list = get_target((2*k_int), _step_no, _target_1, _target_2);

  if (mod_4 == 1)
    _target_list = get_target((2*k_int+1), _step_no, _target_1, _target_2);

  if (mod_4 == 2)
    _target_list = get_target((2*k_int), _step_no, _target_4, _target_3);

  if (mod_4 == 3)
    _target_list = get_target((2*k_int+1), _step_no, _target_4, _target_3);
  
  return _target_list;
}
