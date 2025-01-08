void set_speed_R(float pourcentage) 
{
    // definit la vitesse de rotation du moteur droit
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
    // definit la vitesse de rotation du moteur gauche
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
    LATAbits.LATA4 = 1;
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
        for (t = 0; t < mid_time; t++)
        {
            l[t] = (SIGN(distance)) * ACCEL/2.0 * SQUARE(t);
                // phase d'acceleration
            l[2*mid_time - (t + 1)] = (float) distance - l[t];
                // phase de deceleration
        }
        for (t = 2*mid_time; t < 2*mid_time + 15; t++)
        { // rajoute la position finale pour 15 centiemes de secondes
            l[t] = (float) distance;
        }
        for (t = 0; t < 2*mid_time + 15; t++)
        { // consigne d'angle nulle
            a[t] = 0;
        }
    }
    else
    { // s'il faut tourner
        mid_time = MIN(100, (int) sqrt((double) 4.0 / (ANG_ACCEL) * (ABS(angle)))/2.0);
                // temps avant de parcourir la moitie de l'angle a faire
        for (t = 0; t < mid_time; t++)
        {
            a[t] = (SIGN(angle)) * ANG_ACCEL/2.0 * SQUARE(t);
                // phase d'acceleration
            a[2*mid_time - (t + 1)] = (float) angle - a[t];
                // phase de deceleration
        }
        for (t = 2*mid_time; t < 2*mid_time + 15; t++)
        { // rajoute l'angle final pour 15 centiemes de secondes
            a[t] = (float) angle;
        }
        for (t = 0; t < 2*mid_time + 15; t++)
        { // consigne de position nulle
            l[t] = 0;
        }
    }
    
    // -------------------- SUIVI DE CONSIGNE --------------------POSxCNT
    
    t = 0; // on commence au debut de la liste
    float pos_l; // distance parcourue par la roue gauche
    float pos_r; // distance parcourue par la roue droite
    float cmd_l; // ancienne commande de translation
    float cmd_r; // ancienne commande de rotation
    char timer1_count = 0;
    T1CONbits.TON = 1; // on allume le timer 1 (100Hz)
    while (t < 2*mid_time + 15){
        // tant qu'on n'a pas fait toute la consigne
        if (IFS0bits.T1IF) { 
            IFS0bits.T1IF = 0; // on eteint le timer 1
            // si le timer 1 sonne (7 fois par centieme de seconde)
            timer1_count++;
            if (timer1_count == 6) {
                // tous les centiemes de seconde
                timer1_count = 0;
                pos_r = CLK_TO_CM((int)(POS1CNT - 0x8000)); // centre, transforme en cm
                pos_l = CLK_TO_CM((int)(POS2CNT - 0x8000)); // centre, transforme en cm
                if (t != mid_time)
                { // si on est en MRUA
                    cmd_l = KP_L * (l[t] - (pos_l + pos_r)/2.0);
                    cmd_r = KP_R * (a[t] - (CM_TO_ANG((float)(pos_l - pos_r))));
                    set_speed_R(cmd_l - cmd_r);
                    set_speed_L(cmd_l + cmd_r);
                    t++;
                }
                else 
                { // si on est en MRU
                    if (straight)
                    {
                        set_speed_R(cmd_l - KP_R * (0.0 - (CM_TO_ANG((float)(pos_l - pos_r)))));
                        set_speed_L(cmd_l + KP_R * (0.0 - (CM_TO_ANG((float)(pos_l - pos_r)))));
                        //if ((ABS(l[mid_time] - (pos_r + pos_l)/2.0)) <= 35.0 * (ABS((pos_l + pos_r - last_pos_l - last_pos_r)/2.0)))
                        if ((ABS(KP_L * (l[t] - (pos_l + pos_r)/2.0))) <= (ABS(cmd_l)))
                        { // si il faut commencer a ralentir
                            t++;
                        }
                    }
                    else
                    {
                        set_speed_R(KP_L * (0.0 - (pos_l + pos_r)/2.0) - cmd_r);
                        set_speed_L(KP_L * (0.0 - (pos_l + pos_r)/2.0) + cmd_r);
                        //if ((ABS(a[mid_time] - CM_TO_ANG((float)(pos_l - pos_r)))) <= 10.0 * (ABS(CM_TO_ANG((float)(pos_l - pos_r)) - CM_TO_ANG((float)(last_pos_l - last_pos_r)))))
                        if ((ABS(KP_R * (a[t] - (CM_TO_ANG((float)(pos_l - pos_r)))))) <= (ABS(cmd_r)))
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
    set_speed_R(0); // eteint le moteur droit
    T1CONbits.TON = 0; // eteint le timer 1
    TMR1 = 0; // remet la valeur du timer 1 a 0 (optionnel)
}