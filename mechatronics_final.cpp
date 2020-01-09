//
//2018/01/11
//
#include "mbed.h"
#include "rtos.h"
//thread
const int DELTA_T = 1;     //thread time [ms]
const int SIG_TH1 = 0x1;   //signal number
//Robot state structure
typedef struct{
    short push_on;         //switch
    short ref_sensor;　    //sensor imformation
    short black;            //sensor gray
    short former;          //former sensor imformation
    short back;            //back mode
    short jump;            //jump mode
    float right;           //right duty
    float left;            //left duty
} Newsmap;
//pin setting
DigitalOut led1(D2);       //LED blue
DigitalOut ledR(D12);      //LED green (Blinker Right)
DigitalOut ledL(D13);      //LED green (Blinker Left)
PwmOut     pwmR(D1);       //Right motor
DigitalOut digitalR (D0);  //Right motor
PwmOut     pwmL(D5);       //Left motor
DigitalOut digitalL (D6);  //Left motor
DigitalIn  pushswitch(D11);//pushswitch
AnalogIn   AD_counter(D3); //AD counter
AnalogIn   refRight(A0);   //Right sensor (red)
AnalogIn   refCenter(A1);  //Center sensor (yellow)
AnalogIn   refLeft(A2);    //Left sensor (blue)
InterruptIn interRight(D4);//Right encoder
InterruptIn interLeft(D9); //Left encoder
//encoder
int counterR=0;
int counterL=0;
void handlerR(void){
    counterR++;
}
void handlerL(void){
    counterL++;
}
//motor
void motor(int digital,float duty_R,float duty_L){
    digitalR = digital;    //1,0 (advance=1,goback=0)
    pwmR.write(duty_R);    //0.0~1.0
    digitalL = digital;    //1,0
    pwmL.write(duty_L);    //0,0~1.0
}
//LED
void flash(int R,int L){
    ledR=R;     　　　　　　　//1,0 (on=1,off=0)
    ledL=L;     　　　　　　　//1,0
}
//sensor_input
void sensor_input(Newsmap* map){
    //初期化
    map->ref_sensor= 0;
    map->black = 0;
    //スイッチが反応したら１
    if(pushswitch.read()>0.0) map->push_on = 1;　　　　　//Switch is on.
    //２進数による白黒判定(白０、黒１)
    if(refRight.read()<=0.35f) map->ref_sensor += 1;　　//001,Right
    if(refCenter.read()<=0.028f) map->ref_sensor += 2; //010,Center
    if(refLeft.read()<=0.20f) map->ref_sensor += 4; 　　//100,Left
    //黒を２段階で判定
    if(refRight.read()<=0.20f) map->black = 1;　　　　　　//Right
    if(refCenter.read()<=0.022f) map->black = 2;   　　　//Center
    if(refLeft.read()<=0.10f) map->black = 3; 　　　　　　//Left
}
//linetrace
/*
センサーの白黒判定を元にduty差がだんだん大きくなる比例制御になっている。
今回は値を直接設定している。またセンサーの黒を2段階にすることで10通りの
制御を行なっている。センサーが全て白に出たらさらにduty差をつけているので
さらに２通りの制御を行なっている。
*/
void linetrace(Newsmap* map){
    //初期化
    float right = 0.0;　　　//duty比　Right
    float left = 0.0;      //duty比　Left
    //距離による自己位置推定で飛び越え、s字で低速運転を行う
    if(counterR>51000&&counterR<53200)map->jump=1;　　　//飛び越え開始
    if(counterR>53200&&counterR<53500)map->jump=2;     //方向転換
    if(counterR>53500&&counterR<60000)map->jump=3;     //線まで直進（低速）
    if(counterR>60000)map->jump=0;                     //飛び越え終了
    //duty比の設定
    float z = 0.33; 　　　　　//直進　010
    float a = 0.20; 　　　　　//回転（低）
    float b = 0.30; 　　　　　//回転（速） 011,110
    float c = 0.33; 　　　　　//回転（速） 001,100
    float g = 0.0;　　　　　　//110,011用
    float y = 0.0;          //100,001用
    float x = 0.01;         //000用
    float m = 0.02;         //motorのduty差用
    float KD_T = 0.20;      //KD*DELTA_T
    //最初の直線（高速）
    if(counterR<32000){
        z = 0.45;
        a = 0.40; 
        b = 0.42;
        c = 0.45;
        m = 0.05;
        KD_T = 0.55;
    }
    //飛び越え（低速でトレース）
    if(map->jump==3){
        z = 0.22;
        a = 0.22;
        b = 0.28;
        c = 0.30;
        m = 0.0;
        KD_T = 0.20;
    }
    //グレーでduty差を大きく
    if(map->black==1||map->black==3){
        g=0.01;
        y=0.02;
    }
    if(map->black==2){
        g=0.02;
    }
    //sensorの判定でトレースする
    int ref = map->ref_sensor;
    switch(ref){
        case 6: //110
            flash(0,1);
            right = b+g;
            left = a;
            break;
        case 4: //100
            flash(0,1);
            right = c+y;
            left = a;
            break;
        case 3: //011
            flash(1,0);
            right = a;
            left = b+g;
            break;
        case 2: //010
            flash(1,1);
            right = z;
            left = z;
            break;
        case 1: //001
            flash(1,0);
            right = a;
            left = c+y;
            break;
        default:　//前回の値を元に走行（duty差はより大きく）000,101,111
            if(map->former==6||map->former==4){
                flash(0,1);
                right = c+x;
                left = a;
                ref=4;
            }
            if(map->former==3||map->former==1){
                flash(1,0);
                right = a;
                left = c+x;
                ref=1;
            }
            if(map->former==2){
                flash(1,1);
                right = z;
                left = z;
                ref=2;
            }
        }
        //D制御を行う
        if(right!=z){
            right = right + KD_T*(map->right-right);
            left = left + KD_T*(map->left-left);
        }
        motor(0,right,left+m); //モーター特性により左右差 通常0.03
        //データを保存
        map->right=right;
        map->left=left;
        //飛び越え
        if(map->jump==1){  //方向変換
            flash(0,0);
            motor(0,0.33f,0.29f);
            map->jump=3;
        }
        if(map->jump==2){  //低速直線
            flash(1,1);
            motor(0,0.30f,0.30f);
            ref=6;         //黒を見つけるまで直線
        }
        //push back       
        if(map->push_on==1){
            map->back++;
            if(map->back<55)motor(1,0.26,0.70);  //方向転換
            if(map->back>55&&map->back<70){    
                flash(1,1);
                motor(0,0.6,0.6);　　//直進
            }
            if(map->back>70){
                map->push_on=0;
                map->back=0;
                ref=6;　　　　　　　　//黒を見つけるまで左回転
            }
        }
        //データを保存
        map->former=ref;
}
//main control
void robot_control(void const* args){  
    Newsmap map;
    //motor(10kHz)
    pwmR.period(1.0/10000);
    pwmL.period(1.0/10000);
    //初期化
    map.push_on=0;
    map.back=0;
    map.jump=0;
    map.right=0;
    map.left=0;
    while(true){
        Thread::signal_wait(SIG_TH1);
        sensor_input(&map);
        linetrace(&map);
        led1 = 0;
    }
}
int main() {
    Thread thread (robot_control);
    interRight.rise(&handlerR);
    interRight.fall(&handlerR);
    interLeft.rise(&handlerL);
    interLeft.fall(&handlerL);
    while(true){
        Thread::wait(DELTA_T);
        led1 = 1;
        thread.signal_set(SIG_TH1);
    }
}
