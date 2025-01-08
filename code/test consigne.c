#include "stdio.h"
#include "math.h"       // Contains sqrt definition

#define CLK_TO_CM(a) (a*DIAM*PI)/1440
#define CM_TO_ANG(a) (a*360)/(EMPA*PI)
#define SQUARE(a) a*a
#define MIN(a, b) a<b ? a:b
#define ABS(a) a>0 ? a:-a
#define SIGN(a) a>0 ? 1:-1

#define PI 3.1415926535
#define KP_L 0.0369
#define KP_R CM_TO_ANG(KP_L)
#define ACCEL 0.005
#define DIAM 8.2
#define EMPA 11 // CORRIGER
#define ANG_ACCEL CM_TO_ANG(ACCEL)

void move(int distance, int angle);

int main(void)
{
    move(10, 0);
    move(100, 0);
    move(0, 90);
    move(0, 270);
    move(-10, 0);
    move(-100, 0);
    move(0, -90);
    move(0, -270);

}

void move(int distance, int angle)
{

    // ----------------------- INITIALISATION -----------------------
    int t;
    float l[215]; // contiendra la consigne de position (tous les centi�mes)
    float a[215]; // contiendra la consigne d'angle (tous les centi�mes)
    char straight = (angle == 0);
    int mid_time;

    // ------------------- GENERATION DE CONSIGNE -------------------

    if (straight)
    { // s'il ne faut pas tourner
        mid_time = MIN(100, (int) sqrt((double) 4.0 / (ACCEL) * (ABS(distance)))/2.0);
                // temps avant de parcourir la moiti� de la distance � faire
        for (t=0; t < mid_time; t++)
        {
            l[t] = (SIGN(distance)) * ACCEL/2.0 * SQUARE(t);
            l[2*mid_time - (t+1)] = (float) distance - l[t];
        }
        for (t=2*mid_time; t<2*mid_time + 15; t++)
        { // rajoute la position finale pour 15 centi�mes de secondes
            l[t] = (float) distance;
        }
        for (t=0; t<2*mid_time + 15; t++)
        { // consigne d'angle nulle
            a[t] = 0;
        }
    }
    else
    { // s'il faut tourner
        mid_time = MIN(100, (int) sqrt((double) 4.0 / (ANG_ACCEL) * (ABS(angle)))/2.0);
                // temps avant de parcourir la moiti� de l'angle � faire
        for (t=0; t < mid_time; t++)
        {
            a[t] = (SIGN(angle)) * ANG_ACCEL/2.0 * SQUARE(t);
            a[2*mid_time - (t+1)] = (float) angle - a[t];
        }
        for (t=2*mid_time; t<2*mid_time + 15; t++)
        { // rajoute l'angle final pour 15 centi�mes de secondes
            a[t] = (float) angle;
        }
        for (t=0; t<2*mid_time + 15; t++)
        { // consigne d'angle nulle
            l[t] = 0;
        }
    }

    printf("Distance = %d\n", distance);
    for (t=0; t<(2*mid_time + 15); t++)
    {
        printf("%f ", l[t]);
    }
    printf("\nAngle = %d\n", angle);
    for (t=0; t<(2*mid_time + 15); t++)
    {
        printf("%f ", a[t]);
    }
    printf("\n\n");
}
