#include <xc.h>         // Contains SFR variables definition

#define FCY 40000000     // cycle frequency. Needed for __delay_ms
#include "libpic30.h"   // Contains __delay_ms definition
#include "math.h"       // Contains sqrt definition
#include "FskDetector.h"
#include <math.h>
#include "adc.h"

#define SQUARE(a) a*a
#define MIN(a, b) a<b ? a:b
#define ABS(a) a>0 ? a:-a
#define SIGN(a) a>0 ? 1:-1

#define SCALE_FACTOR (1<<9)
#define PI 3.1415926535
#define KP_L 0.0369
#define KP_R 0.017971
#define ACCEL 0.005
#define DIAM 8.2
#define EMPA 14.15 
#define CLK_TO_CM(a) (a*DIAM*PI)/1440
#define CM_TO_ANG(a) (a*180)/(EMPA*PI)
#define ANG_ACCEL CM_TO_ANG(ACCEL)

void setup();
void set_speed_R(float pourcentage);
void set_speed_L(float pourcentage);
void move(int val, int angle);
void listen();
void verify(int msg);

int main(void) 
{
    frcPll40MHzConfig();
    setup();
    LATAbits.LATA4=0;
    listen();
    //verify(0b0110110010011);
    //__delay_ms(1000);
    while(1) {}
    return 0;
}

int threshold(int32_t value, int32_t threshold) {
    return (value > threshold) ? 1 : 0;
}


int32_t to_fixed_point(double value) {
    return round(value * SCALE_FACTOR);
}
double from_fixed_point(int32_t value) {
    return (double)value / SCALE_FACTOR;
}

int32_t process_filter_stage(int32_t input_sample_fixed, int32_t sos[5], int32_t gain, int32_t stage, int32_t x_buffer[2][4], int32_t y_buffer[2][4]) {

    int32_t b[3] = {sos[0], sos[1], sos[2]}; //pourquoi ne pas prendre directement fixed point en dehors ??? Genre calculer � l'avance ?
    int32_t a[2] = {sos[3], sos[4]};

    // Calcul des produits des coefficients b et des valeurs du signal/input buffer
    int32_t product_b0_x = (b[0] * input_sample_fixed) / SCALE_FACTOR;
    int32_t product_b1_xbuffer0 = (b[1] * x_buffer[0][stage]) / SCALE_FACTOR;
    int32_t product_b2_xbuffer1 = (b[2] * x_buffer[1][stage]) / SCALE_FACTOR;
    int32_t product_a1_ybuffer0 = (a[0] * y_buffer[0][stage]) / SCALE_FACTOR;
    int32_t product_a2_ybuffer1 = (a[1] * y_buffer[1][stage]) / SCALE_FACTOR;

    // Calcul final de y_n combinant tous les produits ci-dessus
    int32_t y_n = product_b0_x + product_b1_xbuffer0 + product_b2_xbuffer1 - product_a1_ybuffer0 - product_a2_ybuffer1;

    // Mise � jour des buffers pour la r�cursivit�
    x_buffer[1][stage] = x_buffer[0][stage];
    x_buffer[0][stage] = input_sample_fixed;
    y_buffer[1][stage] = y_buffer[0][stage];
    y_buffer[0][stage] = y_n;

    int32_t res= (y_n * gain) / (SCALE_FACTOR);
    return res;
}

void set_speed_R(float pourcentage) 
{
    // defini la vitesse de rotation du moteur droit
    if (pourcentage < 0){
        // inverse le sens de rotation du moteur
        pourcentage = - pourcentage;
        LATBbits.LATB5 = 0;
    }
    else {
        LATBbits.LATB5 = 1;
    }
    if ((ABS(pourcentage)) > 1){
        pourcentage = 1.0;
    }
    OC1RS = 1999 * pourcentage;
}

void set_speed_L(float pourcentage) 
{
    // defini la vitesse de rotation du moteur gauche
    if (pourcentage < 0){
        // inverse le sens de rotation du moteur
        pourcentage = - pourcentage;
        LATBbits.LATB4 = 0;
    }
    else {
        LATBbits.LATB4 = 1;
    }
    if ((ABS(pourcentage)) > 1){
        pourcentage = SIGN(pourcentage);
    }
    OC2RS = 1999 * pourcentage;
}

void move(int distance, int angle)
{
    
    // ----------------------- INITIALISATION -----------------------
    LATAbits.LATA4=1;
    POS1CNT = 0x8000; // initialise la position du moteur 1 au milieu pour eviter l'overflow
    POS2CNT = 0x8000; // initialise la position du moteur 2 au milieu pour eviter l'overflow
    int t;
    float l[215]; // contiendra la consigne de position (tous les centiemes)
    float a[215]; // contiendra la consigne d'angle (tous les centiemes)
    char straight = (angle == 0);
    int mid_time;
    
    // ------------------- GENERATION DE CONSIGNE -------------------
    
    if (straight)
    { // s'il ne faut pas tourner
        mid_time = MIN(100, (int) sqrt((double) 4.0 / (ACCEL) * (ABS(distance)))/2.0);
                // temps avant de parcourir la moitie de la distance a faire
        for (t=0; t < mid_time; t++)
        {
            l[t] = (SIGN(distance)) * ACCEL/2.0 * SQUARE(t);
                // phase d'acc�l�ration
            l[2*mid_time - (t+1)] = (float) distance - l[t];
                // phase de d�c�l�ration
        }
        for (t=2*mid_time; t<2*mid_time + 15; t++)
        { // rajoute la position finale pour 15 centiemes de secondes
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
                // temps avant de parcourir la moitie de l'angle a faire
        for (t=0; t < mid_time; t++)
        {
            a[t] = (SIGN(angle)) * ANG_ACCEL/2.0 * SQUARE(t);
                // phase d'acc�l�ration
            a[2*mid_time - (t+1)] = (float) angle - a[t];
                // phase de d�c�l�ration
        }
        for (t=2*mid_time; t<2*mid_time + 15; t++)
        { // rajoute l'angle final pour 15 centiemes de secondes
            a[t] = (float) angle;
        }
        for (t=0; t<2*mid_time + 15; t++)
        { // consigne de position nulle
            l[t] = 0;
        }
    }
    
    // -------------------- SUIVI DE CONSIGNE --------------------POSxCNT
    
    t = 0; // on commence au debut de la liste
    float pos_l; // distance parcourue par la roue gauche
    float pos_r; // distance parcourue par la roue droite
    float cmd_l; // ancienne commande de translation
    float cmd_r; // ancienne commande re rotation
    char timer1_count = 0;
    T1CONbits.TON = 1; // on allume le timer 1 (100Hz)
    while(t < 2*mid_time + 15){
        // tant qu'on n'a pas fait toute la consigne
        if (IFS0bits.T1IF) { 
            IFS0bits.T1IF = 0; // on eteint le timer 1
            // si le timer 1 sonne (7 fois par centieme de seconde)
            timer1_count++;
            if (timer1_count == 6) {
                // tous les centiemes de seconde
                timer1_count = 0;
                pos_r = CLK_TO_CM((int)(POS1CNT-0x8000)); // centre, transforme en cm
                pos_l = CLK_TO_CM((int)(POS2CNT-0x8000)); // centre, transforme en cm
                if (t != mid_time)
                { // si on est en MRUA
                    cmd_l = KP_L*(l[t]-(pos_l+pos_r)/2.0);
                    cmd_r = KP_R*(a[t]-(CM_TO_ANG((float)(pos_l-pos_r))));
                    set_speed_R(cmd_l - cmd_r);
                    set_speed_L(cmd_l + cmd_r);
                    t++;
                }
                else 
                { // si on est en MRU
                    if (straight)
                    {
                        set_speed_R(cmd_l - KP_R*(0.0-(CM_TO_ANG((float)(pos_l-pos_r)))));
                        set_speed_L(cmd_l + KP_R*(0.0-(CM_TO_ANG((float)(pos_l-pos_r)))));
                        //if ((ABS(l[mid_time] - (pos_r+pos_l)/2.0)) <= 35.0*(ABS((pos_l+pos_r-last_pos_l-last_pos_r)/2.0)))
                        if ((ABS(KP_L*(l[t]-(pos_l+pos_r)/2.0))) <= (ABS(cmd_l)))
                        { // si il faut commencer a ralentir
                            t++;
                        }
                    }
                    else
                    {
                        set_speed_R(KP_L*(0.0-(pos_l+pos_r)/2.0) - cmd_r);
                        set_speed_L(KP_L*(0.0-(pos_l+pos_r)/2.0) + cmd_r);
                        //if ((ABS(a[mid_time] - CM_TO_ANG((float)(pos_l-pos_r)))) <= 10.0*(ABS(CM_TO_ANG((float)(pos_l-pos_r)) - CM_TO_ANG((float)(last_pos_l-last_pos_r)))))
                        if ((ABS(KP_R*(a[t]-(CM_TO_ANG((float)(pos_l-pos_r)))))) <= (ABS(cmd_r)))
                        { // si il faut commencer a ralentir
                            t++;
                        }
                    }
                }
            }
        }
    }
    
    // ------------------------ RESET ------------------------
    set_speed_L(0); // eteint le moteur gauche
    set_speed_R(0); // eteint le moteur droite
    T1CONbits.TON = 0; // eteint le timer 1
    TMR1 = 0; // remet la valeur du timer 1 a 0 (optionnel)
}

void listen() {
    //unsigned int sample;
    int32_t input_sample;
    //int data[100];

    
//------------------------------------------------------------------------------
// On initialise toutes les donn�es ici

    int32_t x_buffer_900[2][4]={{0}} ;
    int32_t y_buffer_900[2][4]={{0}} ;
    int32_t x_buffer_1100[2][4]={{0}} ;
    int32_t y_buffer_1100[2][4]={{0}} ;

    //int32_t  input_sample_900Hz, input_sample_1100Hz;
    int32_t  output_sample_900Hz, output_sample_1100Hz;

    // Coefficients et gains pour les filtres de 900Hz et 1100Hz
    const double coeffs_900Hz[4][5] = {
            {1, 2, 1, -1.86370006, 0.98824962},
            {1, 2, 1, -1.86719114, 0.98840267},
            {1, -2, 1, -1.86766577, 0.99507105},
            {1, -2, 1, -1.87591938, 0.99522511}
    };
    const double  gains_900Hz[5] = {0.001771, 0.001726, 0.023, 0.02283};

    const double  coeffs_1100Hz[4][5] = {
            {1, 2, 1, -1.80592184, 0.98584165},
            {1, 2, 1, -1.80081358, 0.98565917},
            {1, -2, 1, -1.8047745, 0.99398116},
            {1, -2, 1, -1.8169234, 0.99416513}
    };
    const double  gains_1100Hz[5] = {0.00259, 0.002661, 0.02277, 0.02273};

    int32_t coeffs_900Hz_fixed[4][5]={};
    int32_t coeffs_1100Hz_fixed[4][5]={};
    int32_t gains_900Hz_fixed[5]={};
    int32_t gains_1100Hz_fixed[5]={};
    int32_t threshold900 = 500; // Exemple de seuil pour 900Hz
    int32_t threshold1100 = 150; // Exemple de seuil pour 1100Hz

    for (int j=0;j<4;j++){
        for (int z=0;z<5;z++){
            coeffs_900Hz_fixed[j][z]=to_fixed_point(coeffs_900Hz[j][z]);
            coeffs_1100Hz_fixed[j][z]=to_fixed_point(coeffs_1100Hz[j][z]);
        }
    }

    for (int k=0;k<5;k++){
        gains_900Hz_fixed[k]=to_fixed_point(gains_900Hz[k]);
        gains_1100Hz_fixed[k]=to_fixed_point(gains_1100Hz[k]);
    }

    unsigned int count=0;
    
    int32_t maxValue900;
    int32_t maxValue1100;
    int message=0;
    TMR3 = 0;
    T3CONbits.TON = 1;      // on d?marre le timer3
//------------------------------------------------------------------------------
    while(1) {
        
        while (!adcConversionDone()) {}  // en "pollant" l'ADC
           
            input_sample = adcRead();  // Exemple d'�chantillon d'entr�e

                    // Traitement des filtres pour chaque �chantillon
                    

            for (int i = 0; i < 4; i++) {
                if (i==0){
                    output_sample_900Hz = process_filter_stage(input_sample, coeffs_900Hz_fixed[i], gains_900Hz_fixed[i], i, x_buffer_900, y_buffer_900);
                    output_sample_1100Hz = process_filter_stage(input_sample, coeffs_1100Hz_fixed[i], gains_1100Hz_fixed[i], i, x_buffer_1100, y_buffer_1100);
                }
                else {
                    output_sample_900Hz = process_filter_stage(output_sample_900Hz, coeffs_900Hz_fixed[i], gains_900Hz_fixed[i], i, x_buffer_900, y_buffer_900);
                    output_sample_1100Hz = process_filter_stage(output_sample_1100Hz, coeffs_1100Hz_fixed[i], gains_1100Hz_fixed[i], i, x_buffer_1100, y_buffer_1100);
                }
            }

            if (count==0) {
                maxValue900=output_sample_900Hz;
                maxValue1100=output_sample_1100Hz;
            }
            else {
                if (output_sample_900Hz > maxValue900){
                    maxValue900=output_sample_900Hz;
                }
                else if (output_sample_1100Hz > maxValue1100){
                    maxValue1100=output_sample_1100Hz;
                }
            }
                    
            count++;
            if (count==16){
                       
                int result900 = threshold(maxValue900, threshold900);
                int result1100 = threshold(maxValue1100, threshold1100);
                message= fskDetector(result900,result1100);
                
                count=0;
                       
            }
                //}
//            message = 0x32;
            if (message){
                verify(message);
                message=0;
            }

    }
}

void verify(int command) { // vérification du bit de début et du bit de fin
    char order = command >> 8;
    int value = (command & 0b0011111111);
    value = value * (1 - 2*(order & 0b1)); // inverse le parametre pour reculer/tourner � gauche
    order = order >> 1; // enl�ve le deuxi�me bit devenu inutile
    move((order^0b1)*value, (order&0b1)*value);
}

void setup() 
{
    
    // ------------------------- MOTEUR DROIT -------------------------
    
    // Module OC1 (output compare)
        OC1R = 0;
        OC1CONbits.OCM = 6;  // Module en PWM (sans protection)
        OC1CONbits.OCTSEL = 0; // Utilise le timer2 comme source
        OC1RS = 0; // On initialise la tension a 0
        //PR = 40 000 000/20 000 - 1 = 1999  (20kHz inaudible -> confort)
        // Config timer2
        PR2 = 1999;
        _RP2R = 0b10010; // RB2 est la sortie
        
    // PWM1 sens
        TRISBbits.TRISB5 = 0; // RB5 est une sortie
        LATBbits.LATB5 = 1; // tourne dans le sens positif
        
    // Module QEI1 (quadrature encodeur interface)
        _QEA1R = 8; // phase A en RB8
        _QEB1R = 9; // phase B en RB9
        QEI1CONbits.QEIM = 0b111; // active en x4 avec reset automatique
        QEI1CONbits.UPDN = 1; // sens de rotation positif
        QEI1CONbits.SWPAB = 1; // capteurs A et B pas echanges
        
    // ------------------------- MOTEUR GAUCHE -------------------------
        
    // Module OC2 (output compare)
        OC2R = 0;
        OC2CONbits.OCM = 6;  // Module en PWM (sans protection)
        OC2CONbits.OCTSEL = 0; // Utilise le timer2 comme source
        OC2RS = 0; // On initialise la tension a 0
        _RP3R = 0b10011; // RB3 est la sortie
        T2CONbits.TON = 1;

    // PWM1 sens
        TRISBbits.TRISB4 = 0; // RB4 est une sortie
        LATBbits.LATB4 = 1; // tourne dans le sens positif
        
    // Module QEI2 (quadrature encodeur interface)
        _QEA2R = 10; // phase A en RB10
        _QEB2R = 11; // phase B en RB11
        QEI2CONbits.QEIM = 0b111; // active en x4 avec reset automatique
        QEI2CONbits.UPDN = 1; // sens de rotation positif
        QEI2CONbits.SWPAB = 1; // capteurs A et B pas echanges
        
    // ---------------------------- UART ----------------------------
    /*    
        _U1RXR = 7; // r�ception en RP7
        _RP6R = 3; // �mission en RP6
        U1MODEbits.PDSEL = 0; // 8 bits sans parite
        U1MODEbits.STSEL = 0; // 1 stop bit
        U1MODEbits.BRGH = 0;
        U1BRG = 3; // f = 57.6 kHz
*/
    // --------------------- ECHANTILLONAGE QEI ---------------------
        
    // Timer 1
        // PR = 40 000 000/100 - 1 = 399 999 (100 Hz))
        // Mais overflow : PR sur 16 bits -> max = 65 536
        // donc on utilise 57 143 et toutes les 7 incr�mentations, on augmente
        PR1 = 57143;

    // -------------------------- ADC --------------------------

        TRISAbits.TRISA4=0;
        // Configuration de L'ADC pour utilisation en polling sur AN0
        adcInit(ADC_TIMER3_SAMPLING);
        PR3 = 2508;   // T=62.7434?s=(PR1+1)/40MHz => PR1+1=2509.736
        
    // ------------------------- DEMARRAGE -------------------------
 //       U1MODEbits.UARTEN = 1;
 //       U1STAbits.UTXEN = 1;
}
