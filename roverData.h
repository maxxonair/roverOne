
struct WheelStatus
{
  boolean isForward = false;
  boolean isStop    = true;
  int pace          = 0;
};

struct JoyData 
{
  int counter = 0 ;
  int X1      = 0 ;
  int Y1      = 0 ;
  int X2      = 0 ;
  int Y2      = 0 ;
};

struct Telemetry 
{
  long clockTime      = 0 ;
  int drivingMode     = 0 ;
  int frontDistance   = 0 ;
  double ax           = 0 ;
  double ay           = 0 ;
  double az           = 0 ;
  double gx           = 0 ;
  double gy           = 0 ; 
  double gz           = 0 ;
  double temperature  = 0 ;
  double busVoltage   = 0 ;
  WheelStatus statusWheel_A;
  WheelStatus statusWheel_B;
  WheelStatus statusWheel_C;
  WheelStatus statusWheel_D;
};
