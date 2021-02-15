#include <stdio.h>
#include <errno.h>
#include <string.h>

#include <wiringPi.h>
#include <softServo.h>

#define FRU 0 //FRU  =  Front Right Upper leg
#define FRL 1 //FRL  =  Front Right Lower leg
#define FLU 2 //FLU  =  Front Left  Upper leg
#define FLL 3 //FLL  =  Front Left  Lower leg
#define BRU 4 //BRU  =  Back  Right Upper leg
#define BRL 5 //BRL  =  Back  Right Lower leg
#define BLU 6 //BLU  =  Back  Left  Upper leg
#define BLL 7 //BLL  =  Back  Left  Lower leg

//*********NEUTRAL*STAND*CALIBRATION****************
#define NEUTRALFRU = 60;   // You need to calibrate this to your Greyhound
#define NEUTRALFRL = 6;    // All the legs need to be in the normal standing position
#define NEUTRALFLU = 75;   //
#define NEUTRALFLL = 178;  //
#define NEUTRALBRU = 28;   //
#define NEUTRALBRL = 132;  //
#define NEUTRALBLU = 125;  //
#define NEUTRALBLL = 45;   //

int degreeToPulse(int degree){
    return degree * 2000 / 180;
}

int main ()
{
	if (wiringPiSetup () == -1) {
        fprintf (stdout, "oops: %s\n", strerror (errno)) ;
        return 1 ;
	}

    int Speed = 500; // time of a loop in ms

	softServoSetup (FRU, FRL, FLU, FLL, BRU, BRL, BLU, BLL) ;

	softServoWrite (FRU,  degreeToPulse(NEUTRALFRU)) ;
    softServoWrite (FRL,  degreeToPulse(NEUTRALFRL)) ;
    softServoWrite (FLU,  degreeToPulse(NEUTRALFLU)) ;
    softServoWrite (FLL,  degreeToPulse(NEUTRALFLL)) ;
    softServoWrite (BRU,  degreeToPulse(NEUTRALBRU)) ;
    softServoWrite (BRL,  degreeToPulse(NEUTRALBRL)) ;
    softServoWrite (BLU,  degreeToPulse(NEUTRALBLU)) ;
    softServoWrite (BLL,  degreeToPulse(NEUTRALBLL)) ;

	delay (10) ;
}