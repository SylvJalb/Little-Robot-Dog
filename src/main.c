#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

#include <wiringPi.h>
#include <pca9685.h>

#define PI 3.14159265

#define PIN_BASE 300
#define PWM_RANGE_MIN 101 // tick when servo is at 0°
#define PWM_RANGE_MAX 499 // tick when servo is at 180°
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

//The legs Gap
#define FURGAP 40 // Front Upper Right leg degree Gap
#define FULGAP 40 // Front Upper Left  leg degree Gap
#define BURGAP 17 // Back  Upper Right leg degree Gap
#define BULGAP 17 // Back  Upper Left  leg degree Gap

#define LENGTHFU 58.0  // Length of Front Upper legs
#define LENGTHFL 133.0 // Length of Front Lower legs
#define LENGTHBU 78.0  // Length of Back  Upper legs
#define LENGTHBL 92.0  // Length of Back  Lower legs

// Function that move servo motor to given degree
void moveToDegree(unsigned int servo, float degree){
    int tick = (int)(((degree / 180.0) * (PWM_RANGE_MAX - PWM_RANGE_MIN)) + PWM_RANGE_MIN);
	pwmWrite(PIN_BASE + servo, tick);
    //printf("Servo n°%d to the position : %f degree -> %d tick\n", servo, degree, tick);
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

    float distA = rA + rB;
    float distB;
    float distC = sqrt( (double)(xB*xB + yB*yB) );

    //printf("\txB => %f\n\tyB => %f\n\trA => %f\n\trB => %f\n", xB, yB, rA, rB);

    if(distC > distA){
        // there is no solution, the two circles do not intersect, impossible movement
        printf("Impossible position ! (%f , %f)\n", xB, yB);
        return -1;
    }

    ////////////////////////////////////////////////////////
    // Calculate the x intersections of the two circles : // (the x Knee postition)
    if(yA == yB){
        // don't div by 0
        printf("Div by 0 ! => yA == yB !\n");
        return -1;
    }
    float div = (xA-xB)/(yA-yB);
    float N = ((rB*rB)-(rA*rA)-(xB*xB)+(xA*xA)-(yB*yB)+(yA*yA)) / (2*(yA-yB));
    float a = (div * div) + 1;
    float b = (2*yA*div) - (2*N*div) - (2*xA);
    float c = (xA*xA) + (yA*yA) + (N*N) - (rA*rA) - (2*yA*N);

    float rdelta = sqrt( (double) ((b*b) - (4*a*c)) ); // √Δ = √(b²-4ac)
    //printf("\trdelta => %f\n", rdelta);
    //solutions
    float xKnee = (0 - b - rdelta) / (2*a);
    float x2 = (0 - b + rdelta) / (2*a);
    //printf("\txKnee => %f\n", xKnee);

    // if we have 2 solutions, Knee position is intersection with the smallest x :
    if(xKnee > x2){
        xKnee = x2; // get the smallest x
        //printf("\txKnee => %f (CHANGE xKnee)\n", xKnee);
    }


    ////////////////////////////////////////////////////////
    // Calculate the y intersections of the two circles : // (the x Knee postition)
    float yKnee = N - (xKnee * div);
    //printf("\tyKnee => %f\n", yKnee);


    ////////////////////////////////////////////
    // Calculate the angle of 2 servomotors : //
    double convertToDegree = 180.0 / PI;
    distB = sqrt( (double)((xKnee+distA)*(xKnee+distA) + yKnee*yKnee) );
    *topDegree = acosf((distA*distA + rA*rA - distB*distB) / (2*distA*rA)) * convertToDegree;
    *botDegree = acosf((rB*rB + rA*rA - distC*distC) / (2*rB*rA)) * convertToDegree;
    //printf("\tdistA => %f\n\tdistB => %f\n\tdistC => %f\n\ttopDegree => %f\n\tbotDegree => %f\n", distA, distB, distC, *topDegree, *botDegree);

    // if yKnee is > 0, topDegree is negative
    if(yKnee > 0){
        *topDegree = 0 - *topDegree;
        //printf("\ttopDegree => %f (REVERSE <-- yKnee > 0)\n", *topDegree);
    }
    // ajust to the real degrees
    switch(leg){
        case FR :
            *topDegree += FURGAP;
            *botDegree = 180 - *botDegree;
            break;
        case FL :
            *topDegree = 180 - *topDegree - FULGAP;
            break;
        case BR :
            *topDegree += BURGAP;
            *botDegree = 180 - *botDegree;
            break;
        case BL :
            *topDegree = 180 - *topDegree - BULGAP;
            break;
        default:
            //param leg is not a leg !
            printf("param leg = %d is not a leg !\n", (int)leg);
            return -2;
    }
    //If impossible degrees
    if(*topDegree > 180 || *topDegree < 0 || *botDegree > 180 || *botDegree < 0){
        printf("Impossible degrees ! (upper : %f , lower : %f)\n", *topDegree, *botDegree);
        return -3;
    }
    //printf("Final return : topDegree => %f ||| botDegree => %f\n", *topDegree, *botDegree);

    //all went well
    return 0;
}

int main () {
    // Setup wiringPi
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

    float topDegree;
    float botDegree;

    // DO PUSH UP
    while(1){
        for(float j = -100 ; j > -160 ; j = j - 3.0){
            for(unsigned int i = FR ; i <= BL ; i += 1){
                if(getDegrees(i, 0.0, j, &topDegree, &botDegree) != 0){
                    pca9685PWMReset(fd);
                    return 1;
                }
                moveToDegree(i*2     , topDegree);
                moveToDegree(i*2 + 1 , botDegree);
            }
            delay(40);
        }
        for(float j = -160 ; j < -100 ; j = j + 3.0){
            for(unsigned int i = FR ; i <= BL ; i += 1){
                if(getDegrees(i, 0.0, j, &topDegree, &botDegree) != 0){
                    pca9685PWMReset(fd);
                    return 1;
                }
                moveToDegree(i*2     , topDegree);
                moveToDegree(i*2 + 1 , botDegree);
            }
            delay(20);
        }
    }

    printf("FIN\n");

    return 0;
}