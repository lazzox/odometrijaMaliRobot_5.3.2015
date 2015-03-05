#ifndef REGULACIJA_H
#define	REGULACIJA_H

#ifndef FCY
#define FCY 29491200ULL
#endif

#define PI	3.1415926535897932384626433832795
#define d_tocka	62  //Precnik odometrijskog tocka
#define D_tocka	150//rastojanje izmedju odometrijskih tockova

//enkoder daje 5000 inkremenata po krugu,pua  4 zbog qei
#define K1	(long)(4*2*23*1024.0f * D_tocka / d_tocka)  //broj ikremenata po krugu
#define K2	(long)(4*23*1024.0f / (d_tocka * PI))  //broj inkremenata po 1mm
#define Gp_D	2.5//1.2//1.4//1.2//distanca
#define Gd_D	65//37//48//18
#define Gp_T	2.5//0.7//2.5//1.2 //0.8//rotacija
#define Gd_T	50//20//65//35//12

/*pROVERI ENKODER: Part no:MA5D1N4FBK1SA0; Type no.: sca24-5000-N-04-09-64-01-S-00*/
//#define STATUS_IDLE     'I'
//#define STATUS_MOVING   'M'
//#define STATUS_STUCK    'S'
enum States
{
    STATUS_IDLE,
    STATUS_MOVING,
    STATUS_ROTATING,
    STATUS_STUCK,
    STATUS_ERROR
};

void resetDriver(void);
// zadavanje pozicije
void setPosition(int X, int Y, int orientation);

void sendStatusAndPosition(void);
//zadavanje brzine i ubrzanja
void setSpeedAccel(float v);

//funkcija za stizanje u tacku (Xd, Yd)
void gotoXY(int Xd, int Yd, unsigned char krajnja_brzina, char smer);

//funkcija za kretanje pravo s trapezoidnim profilom brzine
void kretanje_pravo(int duzina, unsigned char krajnja_brzina);

//funkcija za dovodjenje robota u zeljenu apsolutnu orjentaciju
void apsolutni_ugao(int ugao);

//funkcija za okretanje oko svoje ose s trapezoidnim profilom brzine
char okret(int ugao);//ne vraca indikaciju
void testing();
//funkcija za kretanje po kruznoj putanji
void kurva(long Xc, long Yc, int Fi, char smer);

void stop(void);
void setSpeed(unsigned char tmp);

enum States getStatus(void);

void forceStatus(enum States);

#endif	/* REGULACIJA_H */

