#include <Ps3Controller.h>
#include <CAN.h>

//timer
float runTime, prevT = 0, timeDif, stateT;
float tempo_ciclo, prev_T = 0, time_Dif, state_T;
int timeInterval = 1000, totalTempTime;
int callerFixedFreq = 16;
int tempoInvio = 800000;

int player = 0;
int battery = 0;
int X=70;
int Y=160;
int xj,yj;
String buff;
char buffX[] = " ";
char buffY[] = " ";
int prec_xj = 70;
int prec_yj = 160;
int n_pos,limit=0;
bool ciclo=false;

static int posX[50];
static int posY[50];

void timeManagement();
void timeInvio();
void Invio_CAN(int a,int b);

void forever()
{
       timeManagement(); 
       xj=(int)(Ps3.data.analog.stick.lx);
       yj=(int)(Ps3.data.analog.stick.ly);
     
     if(stateT >= 1000000/callerFixedFreq){
        stateT = 0;
    
       if (xj>-1){ 
        X=X+1;
        if (X>300) X=300; 
       }

       if (xj<-1){ 
        X=X-1; 
         if (X<=-300) X=-300;
       }

       //cordinata Y
       if (yj>-1){ 
        Y=Y+1;
        if (Y>420) Y=420;
       }
 
       if (yj<-1){ 
        Y=Y-1;
        if ((Y<=0)||(Y<100)) Y=100; 
       }
       if ((prec_xj!=X)||(prec_yj!=Y)){
  
       prec_xj=X;
       prec_yj=Y;
       
       Invio_CAN(X,Y);

       Serial.print(" x="); Serial.print(X, DEC);Serial.print(" y=");Serial.print(Y, DEC); 
       Serial.println();
       }
       
      }
     
     
     
     
     
     if( Ps3.event.button_down.l1 ){
       Serial.println("reset offset");
       CAN.beginPacket(0x41);
       CAN.write('O');
       CAN.endPacket();
     }
        
     if( Ps3.event.button_down.r1 ){
       Serial.println("pos reset");
       X=70;
       Y=160;
     }
     if( Ps3.event.button_down.cross ){
       Serial.println("Salva posizione");
       posX[n_pos]=X;
       posY[n_pos]=Y;
       n_pos++;
       limit=n_pos;
     }
     if( Ps3.event.button_down.circle ){
       Serial.println("Avvia ciclo");
       ciclo=true;
     }
     if( Ps3.event.button_down.triangle ){
       Serial.println("Stop ciclo");
       ciclo=false;
     }
     if( Ps3.event.button_down.square ){
       Serial.println("azzera posizioni salvate");
       n_pos=0;
     }
     
     if (ciclo){
      timeInvio();
      if(n_pos == limit){
        n_pos = 0 ;
      }
        if(state_T >= tempoInvio ){
         
             state_T = 0;
             Invio_CAN(posX[n_pos],posY[n_pos]);
             Serial.print(" x="); Serial.print(posX[n_pos], DEC);Serial.print(" y=");Serial.print(posY[n_pos], DEC);Serial.print("pos");Serial.print(n_pos, DEC); 
             Serial.println();
             n_pos += 1;
        }
     }
        
        
        
        
        
     
    

}

void onConnect(){
    Serial.println("Connected.");
}



void setup()
{
    Serial.begin(115200);

    Ps3.attach(forever);
    Ps3.attachOnConnect(onConnect);
    Ps3.begin("00:1a:7d:da:71:0c");
    
    if (!CAN.begin(500E3)) {
    Serial.println("Starting CAN failed!");
    while (1);
    }
    Serial.println("Ready.");
  
}

void loop()
{
  
    if(!Ps3.isConnected())
        return;

    //-------------------- Player LEDs -------------------
   // Serial.print("Setting LEDs to player "); Serial.println(player, DEC);
   // Ps3.setPlayer(player);

    //player += 1;
    //if(player > 10) player = 0;
    
/*
    //------ Digital cross/square/triangle/circle buttons ------
    if( Ps3.data.button.cross && Ps3.data.button.down )
        Serial.println("Pressing both the down and cross buttons");
    if( Ps3.data.button.square && Ps3.data.button.left )
        Serial.println("Pressing both the square and left buttons");
    if( Ps3.data.button.triangle && Ps3.data.button.up )
        Serial.println("Pressing both the triangle and up buttons");
    if( Ps3.data.button.circle && Ps3.data.button.right )
        Serial.println("Pressing both the circle and right buttons");

    if( Ps3.data.button.l1 && Ps3.data.button.r1 )
        Serial.println("Pressing both the left and right bumper buttons");
    if( Ps3.data.button.l2 && Ps3.data.button.r2 )
        Serial.println("Pressing both the left and right trigger buttons");
    if( Ps3.data.button.l3 && Ps3.data.button.r3 )
        Serial.println("Pressing both the left and right stick buttons");
    if( Ps3.data.button.select && Ps3.data.button.start )
        Serial.println("Pressing both the select and start buttons");*/
   
    delay(2000);
}
void timeInvio(){
  tempo_ciclo = micros();
  time_Dif = tempo_ciclo - prev_T;
  prev_T = tempo_ciclo;
  state_T += time_Dif;
}

void timeManagement(){
  runTime = micros();
  timeDif = runTime - prevT;
  prevT = runTime;
  stateT += timeDif;
}

void Invio_CAN(int a,int b){
  
  buff=String(a);
       
  CAN.beginPacket(0x20);
       
       for (int i=0;i<=buff.length();i++){
        CAN.write(buff[i]);
       }
       
  CAN.endPacket();
       
  buff=String(b);
  CAN.beginPacket(0x40);
       
       for (int i=0;i<=buff.length();i++){
        CAN.write(buff[i]);
       }
       
  CAN.endPacket();
}
