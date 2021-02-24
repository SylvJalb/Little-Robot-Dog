#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

#include <wiringPi.h>
#include <pca9685.h>

#define PIN_BASE 300
#define PWM_RANGE_MIN 101 // tick when servo is at 0°
#define PWM_RANGE_MAX 599 // tick when servo is at 180°
#define HERTZ 50

//The 4 legs
#define FR 0 //FR  =  Front Right leg
#define FL 1 //FL  =  Front Left  leg
#define BR 2 //BR  =  Back  Right leg
#define BL 3 //BL  =  Back  Left  leg

//The 8 servo motors
#define FRU 0 //FRU  =  Front Right Upper leg
#define FRL 1 //FRL  =  Front Right Lower leg
#define FLU 2 //FLU  =  Front Left  Upper leg
#define FLL 3 //FLL  =  Front Left  Lower leg
#define BRU 4 //BRU  =  Back  Right Upper leg
#define BRL 5 //BRL  =  Back  Right Lower leg
#define BLU 6 //BLU  =  Back  Left  Upper leg
#define BLL 7 //BLL  =  Back  Left  Lower leg

#define LENGTHFU 58.0  // Length of Front Upper legs
#define LENGTHFL 133.0 // Length of Front Lower legs
#define LENGTHBU 78.0  // Length of Back  Upper legs
#define LENGTHBL 92.0  // Length of Back  Lower legs

// Function that move servo motor to given degree
void moveToDegree(unsigned int servo, float degree){
    int tick = (int)(((degree / 180.0) * (PWM_RANGE_MAX - PWM_RANGE_MIN)) + PWM_RANGE_MIN);
	pwmWrite(PIN_BASE + servo, tick);
    printf("Serv %d to %d \n", servo, tick);
}

// Function that retrieves the degrees for the servos from x and y coordinates
int getDegrees(unsigned int leg, float xB, float yB, float* topDegree, float* botDegree){
    // Definition of necessary variables :
    // Length of Upper legs (Radius of the first circle)
    float rA = (leg == FR || leg == FL ? LENGTHFU : LENGTHBU);
    // Length of Lower legs (Radius of the second circle)
    float rB = (leg == FR || leg == FL ? LENGTHFL : LENGTHBL);
    // The shoulder position
    float xA = 0.0;
    float yA = 0.0;

    ////////////////////////////////////////////////////////
    // Calculate the x intersections of the two circles : // (the x Knee postition)
    float xKnee = -1.0 ; //first result
    float x2 = -1.0 ;    //second result

    float a;
    float b;
    float c;

    if(yA != yB){
        float div = (xA-xB)/(yA-yB);
        float N = ((rB*rB)-(rA*rA)-(xB*xB)+(xA*xA)-(yB*yB)+(yA*yA)) / (2*(yA-yB));
        a = div * div + 1;
        b = (2*yA*div) - (2*N*div) - (2*xA);
        c = (xB*xB) + (yB*yB) + (N*N) - (rA*rA) - (2*yA*N);
    } else {
        // don't div by 0
        // temporary exit with error
        return 0;
    }

    float delta = sqrt( (double)((b*b) - (4*a*c)) ); // Δ = √(b²-4ac)

    //solutions
    if(delta > 0){
        xKnee = (0 - b - sqrt((double)delta)) / (2*a);
        x2 = (0 - b + sqrt((double)delta)) / (2*a);
    } else {
        if(delta == 0){
            xKnee = (0-b) / (2*a);
        } else {
            // there is no solution, the two circles do not intersect, impossible movement
            return 0; //return false
        }
    }
    // if we have 2 solutions, Knee position is intersection with the largest x :
    if(x2 != -1.0)
        xKnee = ( xKnee > x2 ? xKnee : x2 ); // get the largest x


    ////////////////////////////////////////////////////////
    // Calculate the y intersections of the two circles : // (the x Knee postition)
    float yKnee = yA + sqrt( (double)((rA*rA) - ((xKnee-xA)*(xKnee-xA))) ) ; //first result
    if(yKnee != yB + sqrt( (double)((rB*rB) - ((xKnee-xB)*(xKnee-xB))) ) )
        yKnee = yA - sqrt( (double)((rA*rA) - ((xKnee-xA)*(xKnee-xA))) ); //second result


    ////////////////////////////////////////////
    // Calculate the angle of 2 servomotors : //
    float topA = sqrt( (double)((xKnee+(rA+rB))*(xKnee+(rA+rB)) + yKnee*yKnee) );
    float topB = rA + rB;
    float botA = sqrt( (double)(xKnee*xKnee + yKnee*yKnee) );
    *topDegree = acosf((topB*topB + rA*rA - topA*topA) / (2*topB*rA));
    *botDegree = acosf((rB*rB + rA*rA - botA*botA) / (2*rB*rA));

    //all went well
    return 1; //return true
}

int main () {
	wiringPiSetup();

    // Setup with pinbase 300 and i2c location 0x40
	int fd = pca9685Setup(PIN_BASE, 0x40, HERTZ);
	if (fd < 0) {
		printf("Error in setup\n");
		return fd;
	}

	// Reset all output
	pca9685PWMReset(fd);

    //int Speed = 500; // time of a loop in ms

    /*
    pwmWrite(FRU,0);
    pwmWrite(FRL,0);
    pwmWrite(FLU,0);
    pwmWrite(FLL,0);
    pwmWrite(BRU,0);
    pwmWrite(BRL,0);
    pwmWrite(BLU,0);
    pwmWrite(BLL,0);
    */

    moveToDegree(16, 0);
	delay(1800000); //wait 30 minutes

    //float topDegree;
    //float botDegree;

    // GO TO INITIAL POSITION
    /*
    for(unsigned int i = FRU ; i <= BLL ; i += 2){
        float xPos = 0 - ( i <= FLL ? LENGTHFU + LENGTHFL : LENGTHBU + LENGTHBL);
        if(getDegrees(i, xPos, 0.0, &topDegree, &botDegree) != 0){
            moveToDegrees(i, topDegree, botDegree);
        }
    }
    */

    printf("ok Final\n");

    return 0;
}