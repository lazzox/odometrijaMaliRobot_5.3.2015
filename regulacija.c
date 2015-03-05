#include "regulacija.h"
#include "sinus.h"
#include "uart.h"
#include "pwm.h"
#include <p33FJ128MC802.h>
#include <libpic30.h>
#include <math.h>

//POMOCNE PROMENLJIVE:
static volatile long positionR, positionL, posKor;              //trenutne pozicije na enkoderima
static volatile int vR, vL; //trenutne brzine na motorima
static volatile int zaglavL, zaglavR;
static volatile unsigned char brzinaL = 0x80;
static volatile long L = 0, orientation = 0, teta = 0;
static volatile long long int Xlong = 0, Ylong = 0;
static volatile float vmax, accel;
static volatile float omega, alfa;
static volatile long X = 0, Y = 0;
//volatile long brojac, i;
static volatile unsigned long sys_time = 0;
//PROMENLJIVE POTREBNE ZA REGULACIJU
static volatile long long PWML, PWMD;
static volatile long t_ref = 0, d_ref = 0;
static volatile float v_ref;

static volatile long d, t;
static volatile long sint, cost;
static volatile long x, y;
static volatile long greska,greskaL,greskaR;
static volatile signed long brzina, commande_distance,commande_distanceL,commande_distanceR, commande_rotation;

static enum States currentStatus = STATUS_IDLE;

static inline void LeviPWM(unsigned int PWM)
{
    P2DC1 = PWM;
}

static inline void DesniPWM(unsigned int PWM)
{
    P1DC1 = PWM;
}

// **********************************************************************
// ODOMETRIJA (ide na 1ms)
//pidovanje zavisi od vremena trajanja timerskog interapta
// **********************************************************************
void __attribute__((__interrupt__, auto_psv)) _T1Interrupt(void)
{
    sys_time++;

    //CITANJE PODATAKA O POZICIJAMA OBA MOTORA:
    vR = -(int)POS2CNT;	//ocitavamo broj inkremenata u zadnjoj periodi
    POS2CNT = 0;	//resetujemo brojac inkremenata
    positionR += vR;	//azuriramo trenutnu poziciju

    vL = +(int)POS1CNT;
    POS1CNT = 0;	//resetujemo brojac inkremenata
    positionL += vL;	//azuriramo trenutnu poziciju

    // sranje i mora bez toga
    //posKor = positionL * 1.001; //ako bezi udesno, smanjuj; ako bezi ulevo, povecavaj
                                //daska je mama








    L = (positionR + positionL) / 2;//predjena ukupna distanca
    orientation = (positionR - positionL) % K1;//stvarna ukupna orijentacija robota

    if(orientation > K1/2)
        orientation -= K1;
    if(orientation < -K1/2)
        orientation += K1;

    //ovde je izbegnuta prva long long operacija
    /*PRE*/// teta = (long long) orientation * 32768 / K1;
    teta = (orientation * 16384) / (K1 / 2);//trans iz ink u ugao

    // 32768-> pun krug, podeljeno sa K1 daje neke kvazi jedinice u zavisnosti
    // od parametara sistema, rezolucije enkodera, tockova, rastojanja...
    if(teta < 0)
        teta += 32768;

    // tabela sinusa ima 8192 elemenata -> prvi kvadrant
    d = vR + vL;	//ovo se kasnije deli sa 2,predjeni put u zadnjih 1ms
    // uvek se svodi na prvi kvadrant!
    if(teta < 8192)
    {
        t = teta;
        sint = sinus[t];
        cost = sinus[8191 - t];
    }
    else
        if(teta < 16384) // DRUGI KVADRANT
        {
            t = teta - 8192;
            sint = sinus[8191 - t];
            cost = -sinus[t];
        }
        else
            if(teta < 24576) // treci kvadrant
            {
                t = teta - 16384;
                sint = -sinus[t];
                cost = -sinus[8191 - t];
            }
            else    // 4. kvadrant
            {
                t = teta - 24576;
                sint = -sinus[8191 - t];
                cost = sinus[t];
            }

		// x, y -> predjeno delta po x i y koordinati u zadnjoj periodi,tj 1ms (deltax, deltay)
		x = d * cost;
		y = d * sint;

		// x, y je min 32767,stvarne pozicije na terenu,u inkrementima
		Xlong += x;
		Ylong += y;

		//izacunavanje trenutne pozicije [mm]
		// pre:

		//X = (((long long)Xlong / 2) / 32768) / K2;
		//Y = (((long long)Ylong / 2) / 32768) / K2;

		// pretvaranje u milimetre,ova X i Y su ono sto dobijamo kad serijskom trazimo poziciju
		X = (((long long)Xlong / 2) / 32768);
		// pretvaranje u milimetre
		X /= K2;
		Y = (((long long)Ylong / 2) / 32768);
		Y /= K2;


                //od sad upravljanje ,dosad u interaptu bilo odometrija
		//regulator rastojanja
		//brzina = (vL + vR) / 2; // srednja vrednost predjenih inkremenata u zadnjoj periodi -> v = s/t, brzina
		
                greskaL = d_ref - positionL;
                greskaR = d_ref - positionR;
		zaglavL = (greska >= 0 ? greska : -greska);//detekcija blokade robota


                commande_distanceL = greskaL * Gp_D - Gd_D * vL;//PD blok
		commande_distanceR = greskaR * Gp_D - Gd_D * vR;//PD blok


		//regulator orjentacije
		brzina = vL - vR;//ugau u zanjih 1ms u inkre
		greska = (orientation - t_ref) % K1;

		if(greska > K1/2)
			greska -= K1;
		if(greska < -K1/2)
			greska += K1;

		zaglavR = (greska >= 0 ? greska : -greska);//detekcija blokade robota
		commande_rotation = greska * Gp_T - brzina * Gd_T;//PD blok

		// ukupan PWM dobija se superpozicijom uzimajuci u obzir oba regulatora
		// commande_distance = regulator rastojanja
		// commande_rotation = regulator orijentacije

            PWML = -800;//commande_distanceL - commande_rotation;
            PWMD = +800;//commande_distanceR + commande_rotation;
//podesavanja parametra distance i rotacije odvojeno   (//)commande_distance(//) - commande_rotation
            // 52 str.
            //devoir de vacance

            // saturacija,ogranicava max brzinu kretanja
            if(PWMD <= -3200)
                PWMD = -3200;
            else if(PWMD >= 3200)
                PWMD = 3200;

            if(PWML <= -3200)
                PWML = -3200;
            else if(PWML >= 3200)
                PWML = 3200;
            //izbor smera,i kontrola hardvera:
            if(PWML >= 0)
            {
                LATBbits.LATB9 = 0;
                LeviPWM(PWML);
            }
            else
            {
                LATBbits.LATB9 = 1;
                LeviPWM(3200 + PWML);
            }

            if (PWMD >= 0)
            {
                LATBbits.LATB15 = 0;
                DesniPWM(PWMD);
            }
            else
            {
                LATBbits.LATB15 = 1;
                DesniPWM(3200 + PWMD);
            }

             // }//KRAJ REGULACIJE

            IFS0bits.T1IF = 0;    /* Clear Timer interrupt flag */
}

void resetDriver(void)
{
    //inicijalizacija parametara:
    positionR = positionL = 0;
    L = orientation = 0;
    vR = vL = 0;

    setSpeed(0x80);
    setSpeedAccel(K2);	//K2 je za 1m/s /bilo je 2

    setPosition(0, 0, 0);
    currentStatus = STATUS_IDLE;
}

//zadavanje X koordinate
static void setX(int tmp)
{
    unsigned long t;

    Xlong = (long long)tmp * 65536 * K2;
    d_ref = L;
    t_ref = orientation;

    t = sys_time;
    while(sys_time == t);
}

//zadavanje Y koordinate
static void setY(int tmp)
{
    unsigned long t;

    Ylong = (long long)tmp * 65536 * K2;
    d_ref = L;
    t_ref = orientation;

    t = sys_time;
    while(sys_time == t);
}

//zadavanje orjentacije
static void setO(int tmp)
{
    unsigned long t;

    positionL = -(tmp * K1 / 360) / 2;
    positionR = (tmp * K1 / 360) / 2;

    L = (positionR + positionL) / 2;
    orientation = (positionR - positionL) % K1;

    d_ref = L;
    t_ref = orientation;

    t = sys_time;
    while(sys_time == t);
}

void setPosition(int X, int Y, int orientation)
{
    setX(X);
    setY(Y);
    setO(orientation);

    currentStatus = STATUS_IDLE;
}

void sendStatusAndPosition(void)
{
    int tmpO;

    // mora seljacki nacin zato sto je govno govnasto
    if(currentStatus == STATUS_ERROR)
        putch('E');
    else if(currentStatus == STATUS_MOVING)
        putch('M');
    else if(currentStatus == STATUS_ROTATING)
        putch('R');
    else if(currentStatus == STATUS_IDLE)
        putch('I');
    else if(currentStatus == STATUS_STUCK)
        putch('S');

    putch(X >> 8);
    putch(X);

    putch(Y >> 8);
    putch(Y);

    tmpO = orientation * 360 / K1 + 0.5;
    putch(tmpO >> 8);
    putch(tmpO);
    putch(positionR >> 16);
    putch(positionR >> 8);
    putch(positionR);

    putch(positionL >> 16);
    putch(positionL >> 8);
    putch(positionL);

}

void setSpeedAccel(float v)
{
    vmax = v;	//K2 je za 1m/s
    omega = 2 * vmax;
    accel = vmax / 500;	//ako nam faza ubrzanja traje 1000ms
    alfa = 2 * accel;
}


enum States getStatus(void)
{
    return currentStatus;
}

void forceStatus(enum States newStatus)
{
    currentStatus = newStatus;
}

//citanje serijskog porta tokom kretanja
static char getCommand(void)
{
    char command;

    //U1STAbits.OERR = 0;       //skrnavac usrani mora rucno da se resetuje
    if(UART_CheckRX())    //proverava jel stigao karakter preko serijskog porta
    {
        command = getch();

        switch(command)
        {           
            case 'P':
                sendStatusAndPosition();
                break;

            case 'S':
                //ukopaj se u mestu
                d_ref = L;
                t_ref = orientation;
                v_ref = 0;

                currentStatus = STATUS_IDLE;
                __delay_ms(10);

                return 0;

            case 's':
                //stani i ugasi PWM
                d_ref = L;
                t_ref = orientation;
                v_ref = 0;

                CloseMCPWM();
                currentStatus = STATUS_IDLE;
                __delay_ms(10);

                return 0;

            default:
                //case 'G' : case 'D' : case 'T' : case 'A' : case 'Q':
                // primljena komanda za kretanje u toku kretanja

                // stop, status ostaje MOVING

                d_ref = L;
                t_ref = orientation;
                v_ref = 0;

                __delay_ms(100);

                return 0;

        }// end of switch(command)
    }

    return 1;
}

static char checkStuckCondition(void)
{
    if ((zaglavL / 25 > brzinaL) || (zaglavR / 25 > brzinaL)) //16,32
    {
        //ukopaj se u mestu
        d_ref = L;
        t_ref = orientation;
        v_ref = 0;

        currentStatus = STATUS_STUCK;
        __delay_ms(50);
        return 0;
    }

    return 1;
}

//gadjaj tacku (Xd, Yd)
void gotoXY(int Xd, int Yd, unsigned char krajnja_brzina, char smer)
{
    long t, t0, t1, t2, t3;
    long T1, T2, T3;
    long L_dist, L0, L1, L2, L3;
    long D0, D1, D2;
    float v_vrh, v_end, v0;
    int duzina, ugao;
    long long int Xdlong, Ydlong;

    Xdlong = (long long)Xd * K2 * 65536;
    Ydlong = (long long)Yd * K2 * 65536;

    v0 = v_ref;
    smer = (smer >= 0 ? 1 : -1);

    //okreni se prema krajnjoj tacki,nadji ugao gde treba srenuti
    ugao = atan2(Ydlong-Ylong, Xdlong-Xlong) * (180 / PI) - orientation * 360 / K1;//svugde dodam *0.5
    if(smer < 0)
        ugao += 180;
    while(ugao > 180)
        ugao -= 360;
    while(ugao < -180)
        ugao += 360;

    if(okret(ugao))
        return;

    duzina = sqrt((X - Xd) * (X - Xd) + (Y - Yd) * (Y - Yd));//put koji treba preci

    //if(duzina < 500)
	//setSpeedAccel(K2/3);

    //kretanje_pravo(duzina, krajnja_brzina);
    v_end = vmax * krajnja_brzina / 256;
    L_dist = (long)duzina * K2;//pretvaranje u inkrem

    T1 = (vmax - v_ref) / accel;// racunanje koliklo ce trajati svaka faza
    L0 = L;
    L1 = v_ref * T1 + accel * T1 * T1 / 2;

    T3 = (vmax - v_end) / accel;
    L3 = vmax * T3 - accel * T3 * T3 / 2;

    if( (L1 + L3) < L_dist)
    {
        //moze da dostigne vmax
        L2 = L_dist - L1 - L3;
        T2 = L2 / vmax;
    }
    else
    {
        //ne moze da dostigne vmax
        T2 = 0;
        v_vrh = sqrt(accel * L_dist + (v_ref * v_ref + v_end * v_end) / 2);
        if( (v_vrh < v_ref) || (v_vrh < v_end) )
        {
            currentStatus = STATUS_ERROR;
            return; //mission impossible
        }

        T1 = (v_vrh - v_ref) / accel;
        T3 = (v_vrh - v_end) / accel;
    }

    t = t0 = sys_time;//dodaje se trenutno vreme
    t1 = t0 + T1;
    t2 = t1 + T2;
    t3 = t2 + T3;
    D0 = d_ref;

    currentStatus = STATUS_MOVING;
    while(t < t3)
        if(t != sys_time)
        {
            t = sys_time;
            if(!getCommand())
                return;

            if (!checkStuckCondition())
                return;

            if(t <= t2)//obnavlja referencu ugla,tj proverava da li je dodatno skrenuo
            {
                if(smer > 0)
                    t_ref = atan2(Ydlong-Ylong, Xdlong-Xlong) / (2 * PI) * K1;
                else
                    t_ref = atan2(Ylong-Ydlong, Xlong-Xdlong) / (2 * PI) * K1;
            }

            if(t <= t1)
            {
                v_ref = v0 + accel * (t-t0);//trenutna brzuna,t-to je integral
                D1 = D2 = d_ref = D0 + smer * (v0 * (t-t0) + accel * (t-t0)*(t-t0)/2);//(v0 * (t-t0) integralimo pocetnu brzinu,ako je ima
            }//accel * (t-t0)*(t-t0)/2) integralimo dvaput akceleraciju
            else if(t <= t2)
            {
                v_ref = vmax;
                D2 = d_ref = D1 + smer * vmax * (t-t1);
            }
            else if(t <= t3)
            {
                v_ref = vmax - accel * (t-t2);
                d_ref = D2 + smer * (vmax * (t-t2) - accel * (t-t2) * (t-t2) / 2);
            }
        }

    currentStatus = STATUS_IDLE;
}


//funkcija za kretanje pravo s trapezoidnim profilom brzine
void kretanje_pravo(int duzina, unsigned char krajnja_brzina)
{
    long t, t0, t1, t2, t3;
    long T1, T2, T3;
    long L_dist, L0, L1, L2, L3;
    long D0, D1, D2;
    float v_vrh, v_end, v0;
    char predznak;

//Ne moze na manjim rastojanjima od 50cm da ide brzimom koju zelis, ograniceno je na /3 brzine
    //if((duzina < 500) && (duzina > -500))
        //setSpeedAccel(K2 / 3);

    v0 = v_ref;
    v_end = vmax * krajnja_brzina / 256;
    predznak = (duzina >= 0) ? 1 : -1;
    L_dist = (long)duzina * K2; // konverzija u inkremente

    T1 = (vmax - v_ref) / accel;
    L0 = L;
    L1 = v_ref * T1 + accel * T1 * (T1 / 2);

    T3 = (vmax - v_end) / accel;
    L3 = vmax * T3 - accel * T3 * (T3 / 2);

    if((L1 + L3) < (long)predznak * L_dist)
    {
        //moze da dostigne vmax
        L2 = predznak * L_dist - L1 - L3;
        T2 = L2 / vmax;
    }
    else
    {
        //ne moze da dostigne vmax
        T2 = 0;
        v_vrh = sqrt(accel * predznak * L_dist + (v_ref * v_ref + v_end * v_end) / 2);
        if((v_vrh < v_ref) || (v_vrh < v_end))
        {
            currentStatus = STATUS_ERROR;
            return; //mission impossible
        }

        T1 = (v_vrh - v_ref) / accel;
        T3 = (v_vrh - v_end) / accel;
    }

    t = t0 = sys_time;
    t1 = t0 + T1;
    t2 = t1 + T2;
    t3 = t2 + T3;
    D0 = d_ref;

    currentStatus = STATUS_MOVING;
    while(t < t3)
        if(t != sys_time)
        {
            t = sys_time;
            if(!getCommand())
                return;

            if (!checkStuckCondition())
                return;

            if(t <= t1)
            {
                v_ref = v0 + accel * (t-t0);
                D1 = D2 = d_ref = D0 + predznak * (v0 * (t-t0) + accel * (t-t0)*(t-t0)/2);
            }
            else if(t <= t2)
            {
                v_ref = vmax;
                D2 = d_ref = D1 + predznak * vmax * (t-t1);
            }
            else if(t <= t3)
            {
                v_ref = vmax - accel * (t-t2);
                d_ref = D2 + predznak * (vmax * (t-t2) - accel * (t-t2) * (t-t2) / 2);//strana 22
            }
        }

    currentStatus = STATUS_IDLE;
}

//funkcija za dovodjenje robota u zeljenu apsolutnu orjentaciju
void apsolutni_ugao(int ugao)
{
    int tmp = ugao - orientation * 360 / K1;

    if(tmp > 180)
        tmp -= 360;
    if(tmp <-180)
        tmp += 360;

    okret(tmp);
}

//funkcija za okretanje oko svoje ose s trapezoidnim profilom brzine
char okret(int ugao)
{
    long t, t0, t1, t2, t3;
    long T1, T2, T3;
    long Fi_total, Fi1;
    float ugao_ref, w_ref = 0;
    char predznak;

    predznak = (ugao >= 0 ? 1 : -1);
    Fi_total = (long)ugao * K1 / 360;

    T1 = T3 = omega / alfa;
    Fi1 = alfa * T1 * T1 / 2;
    if(Fi1 > (predznak * Fi_total / 2))
    {
        //trougaoni profil
        Fi1 = predznak  * Fi_total / 2;
        T1 = T3 = sqrt(2 * Fi1 / alfa);
        T2 = 0;
    }
    else
    {
        //trapezni profil
        T2 = (predznak * Fi_total - 2 * Fi1) / omega;
    }

    ugao_ref = t_ref;
    t = t0 = sys_time;
    t1 = t0 + T1;
    t2 = t1 + T2;
    t3 = t2 + T3;

    currentStatus = STATUS_ROTATING;
    while(t < t3)
        if(t != sys_time)
        {
            t = sys_time;
            if(!getCommand())
                return 1;
            //if (t % 16 == 0) putch ((char)(zaglavR / 16));
            if(!checkStuckCondition())
                return 1;

            if(t <= t1)
            {
                w_ref += alfa;
                ugao_ref += predznak * (w_ref - alfa / 2);
                t_ref = ugao_ref;
            }
            else if(t <= t2)
            {
                w_ref = omega;
                ugao_ref += predznak * omega;
                t_ref = ugao_ref;
            }
            else if(t <= t3)
            {
                w_ref -= alfa;
                ugao_ref += predznak * (w_ref + alfa / 2);
                t_ref = ugao_ref;
            }
        }

    currentStatus = STATUS_IDLE;

    return 0;
}

void kurva(long Xc, long Yc, int Fi, char smer)
{
    float R, Fi_pocetno, delta, luk;
    long t, t0, t1, t2, t3;
    long T1, T2, T3;
    long Fi_total, Fi1;
    float v_poc, dist_ref, ugao_ref, w_ref = 0, v_ref = 0;
    char predznak;
    int ugao;

    predznak = (Fi >= 0 ? 1 : -1);
    R = sqrt(((X-Xc) * (X-Xc) + (Y-Yc) * (Y-Yc)));
    Fi_pocetno = atan2(((int)Y-(int)Yc), ((int)X-(int)Xc));
    ugao = Fi_pocetno * 180 / PI;
    smer = (smer >= 0 ? 1 : -1);

    ugao = (ugao + smer * predznak * 90) % 360;
    if(ugao > 180)
        ugao -= 360;
    if(ugao < -180)
        ugao += 360;

    ugao -= orientation * 360 / K1;
    ugao %= 360;

    if(ugao > 180)
        ugao -= 360;
    if(ugao < -180)
        ugao += 360;

    if(okret(ugao))
        return;

    v_poc = vmax;
    if(vmax > K2/32)
        setSpeedAccel(K2 / 32);

    Fi_total = (long)Fi * K1 / 360;

    T1 = T3 = omega / alfa;
    Fi1 = alfa * T1 * T1 / 2;
    if(Fi1 > (predznak * Fi_total / 2))
    {
        //trougaoni profil
        Fi1 = predznak  * Fi_total / 2;
        T1 = T3 = sqrt(2 * Fi1 / alfa);
        T2 = 0;
    }
    else
    {
        //trapezni profil
        T2 = (predznak * Fi_total - 2 * Fi1) / omega;
    }

    t = t0 = sys_time;
    t1 = t0 + T1;
    t2 = t1 + T2;
    t3 = t2 + T3;
    ugao_ref = t_ref;
    dist_ref = d_ref;

    currentStatus = STATUS_MOVING;
    while(t < t3)
        if(t != sys_time)
        {
            t = sys_time;
            if(!getCommand())
            {
                setSpeedAccel(v_poc);
                return;
            }

            if(!checkStuckCondition())
            {
                setSpeedAccel(v_poc);
                return;
            }

            if(t <= t1)
            {
                w_ref += alfa;
                v_ref += accel;
                delta = predznak * (w_ref - alfa / 2);
                luk = predznak * R * delta / D_tocka;
                ugao_ref += delta;
                dist_ref += smer * luk;
                t_ref = ugao_ref;
                d_ref = dist_ref;
            }
            else if(t <= t2)
            {
                w_ref = omega;
                v_ref = vmax;
                delta = predznak * omega;
                luk = predznak * R * delta / D_tocka;
                ugao_ref += delta;
                dist_ref += smer * luk;
                t_ref = ugao_ref;
                d_ref = dist_ref;
            }
            else if(t <= t3)
            {
                w_ref -= alfa;
                v_ref -= accel;
                delta = predznak * (w_ref + alfa / 2);
                luk = predznak * R * delta / D_tocka;
                ugao_ref += delta;
                dist_ref += smer * luk;
                t_ref = ugao_ref;
                d_ref = dist_ref;
            }
        }

    setSpeedAccel(v_poc);
    currentStatus = STATUS_IDLE;
}

void stop(void)
{
    d_ref = L;
    t_ref = orientation;

    currentStatus = STATUS_IDLE;
}

void setSpeed(unsigned char tmp)
{
    if(tmp>160)
        tmp=160;
    
    brzinaL = tmp;
    setSpeedAccel(K2  * (unsigned char)brzinaL / 256);
}
