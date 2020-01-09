//
//���J�g���j�N�X���K
//�ŏI�R����
//2018/01/11
//A-6�ǁ@���{�E�c���E���]
//
#include "mbed.h"
#include "rtos.h"
//thread
const int DELTA_T = 1;     //thread time [ms]
const int SIG_TH1 = 0x1;   //signal number
//Robot state structure
typedef struct{
    short push_on;         //switch
    short ref_sensor;�@    //sensor imformation
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
    ledR=R;     �@�@�@�@�@�@�@//1,0 (on=1,off=0)
    ledL=L;     �@�@�@�@�@�@�@//1,0
}
//sensor_input
void sensor_input(Newsmap* map){
    //������
    map->ref_sensor= 0;
    map->black = 0;
    //�X�C�b�`������������P
    if(pushswitch.read()>0.0) map->push_on = 1;�@�@�@�@�@//Switch is on.
    //�Q�i���ɂ�锒������(���O�A���P)
    if(refRight.read()<=0.35f) map->ref_sensor += 1;�@�@//001,Right
    if(refCenter.read()<=0.028f) map->ref_sensor += 2; //010,Center
    if(refLeft.read()<=0.20f) map->ref_sensor += 4; �@�@//100,Left
    //�����Q�i�K�Ŕ���
    if(refRight.read()<=0.20f) map->black = 1;�@�@�@�@�@�@//Right
    if(refCenter.read()<=0.022f) map->black = 2;   �@�@�@//Center
    if(refLeft.read()<=0.10f) map->black = 3; �@�@�@�@�@�@//Left
}
//linetrace
/*
�Z���T�[�̔������������duty�������񂾂�傫���Ȃ��ᐧ��ɂȂ��Ă���B
����͒l�𒼐ڐݒ肵�Ă���B�܂��Z���T�[�̍���2�i�K�ɂ��邱�Ƃ�10�ʂ��
������s�Ȃ��Ă���B�Z���T�[���S�Ĕ��ɏo���炳���duty�������Ă���̂�
����ɂQ�ʂ�̐�����s�Ȃ��Ă���B
*/
void linetrace(Newsmap* map){
    //������
    float right = 0.0;�@�@�@//duty��@Right
    float left = 0.0;      //duty��@Left
    //�����ɂ�鎩�Ȉʒu����Ŕ�щz���As���Œᑬ�^�]���s��
    if(counterR>51000&&counterR<53200)map->jump=1;�@�@�@//��щz���J�n
    if(counterR>53200&&counterR<53500)map->jump=2;     //�����]��
    if(counterR>53500&&counterR<60000)map->jump=3;     //���܂Œ��i�i�ᑬ�j
    if(counterR>60000)map->jump=0;                     //��щz���I��
    //duty��̐ݒ�
    float z = 0.33; �@�@�@�@�@//���i�@010
    float a = 0.20; �@�@�@�@�@//��]�i��j
    float b = 0.30; �@�@�@�@�@//��]�i���j 011,110
    float c = 0.33; �@�@�@�@�@//��]�i���j 001,100
    float g = 0.0;�@�@�@�@�@�@//110,011�p
    float y = 0.0;          //100,001�p
    float x = 0.01;         //000�p
    float m = 0.02;         //motor��duty���p
    float KD_T = 0.20;      //KD*DELTA_T
    //�ŏ��̒����i�����j
    if(counterR<32000){
        z = 0.45;
        a = 0.40; 
        b = 0.42;
        c = 0.45;
        m = 0.05;
        KD_T = 0.55;
    }
    //��щz���i�ᑬ�Ńg���[�X�j
    if(map->jump==3){
        z = 0.22;
        a = 0.22;
        b = 0.28;
        c = 0.30;
        m = 0.0;
        KD_T = 0.20;
    }
    //�O���[��duty����傫��
    if(map->black==1||map->black==3){
        g=0.01;
        y=0.02;
    }
    if(map->black==2){
        g=0.02;
    }
    //sensor�̔���Ńg���[�X����
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
        default:�@//�O��̒l�����ɑ��s�iduty���͂��傫���j000,101,111
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
        //D������s��
        if(right!=z){
            right = right + KD_T*(map->right-right);
            left = left + KD_T*(map->left-left);
        }
        motor(0,right,left+m); //���[�^�[�����ɂ�荶�E�� �ʏ�0.03
        //�f�[�^��ۑ�
        map->right=right;
        map->left=left;
        //��щz��
        if(map->jump==1){  //�����ϊ�
            flash(0,0);
            motor(0,0.33f,0.29f);
            map->jump=3;
        }
        if(map->jump==2){  //�ᑬ����
            flash(1,1);
            motor(0,0.30f,0.30f);
            ref=6;         //����������܂Œ���
        }
        //push back       
        if(map->push_on==1){
            map->back++;
            if(map->back<55)motor(1,0.26,0.70);  //�����]��
            if(map->back>55&&map->back<70){    
                flash(1,1);
                motor(0,0.6,0.6);�@�@//���i
            }
            if(map->back>70){
                map->push_on=0;
                map->back=0;
                ref=6;�@�@�@�@�@�@�@�@//����������܂ō���]
            }
        }
        //�f�[�^��ۑ�
        map->former=ref;
}
//main control
void robot_control(void const* args){  
    Newsmap map;
    //motor(10kHz)
    pwmR.period(1.0/10000);
    pwmL.period(1.0/10000);
    //������
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