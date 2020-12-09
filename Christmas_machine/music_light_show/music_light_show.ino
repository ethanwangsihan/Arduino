//定义宏用来求出数组长度
#define GET_ARRAY_LEN(array,len){len = (sizeof(array) / sizeof(array[0]));}

//预定义音符常量到频率数值
#define DO_L  262
#define RE_L  294
#define MI_L  330
#define FA_L  349
#define SOL_L 392
#define LA_L  440
#define SI_L  494
#define DO_M  523
#define RE_M  587
#define MI_M  659
#define FA_M  698
#define SOL_M 784
#define LA_M  880
#define SI_M  988
#define DO_H  1047
#define RE_H  1175
#define MI_H  1319
#define FA_H  1397
#define SOL_H 1568
#define LA_H  1760
#define SI_H  1976

//整型数组保存歌曲we wish you a merry christmas的一组音符
int melody[] = {
   SOL_M ,  DO_H , DO_H, RE_H, DO_H, SI_M, LA_M, LA_M, LA_M,

  RE_H , RE_H , MI_H , RE_H , DO_H , SI_M ,  SOL_M,

   SOL_M, MI_H, MI_H, FA_H, MI_H, RE_H, DO_H, LA_M,

   SOL_M,  SOL_M, LA_M, RE_H, SI_M, DO_H,

   SOL_M, DO_H, DO_H, DO_H, SI_M, SI_M, DO_H, SI_M, LA_M,  SOL_M,

  RE_H, MI_H, RE_H, DO_H, SOL_H,  SOL_M,  SOL_M,  SOL_M, LA_M, RE_H, SI_M, DO_H

};
int melody1[] = {
  MI_M,MI_M,MI_M,MI_M,MI_M,MI_M,MI_M,SOL_M,DO_M,RE_M,MI_M,
  FA_M,FA_M,FA_M,FA_M,FA_M,MI_M,MI_M,MI_M,MI_M,MI_M,RE_M,RE_M,DO_M,RE_M,SOL_M,
  MI_M,MI_M,MI_M,MI_M,MI_M,MI_M,MI_M,SOL_M,DO_M,RE_M,MI_M,
  FA_M,FA_M,FA_M,FA_M,FA_M,MI_M,MI_M,MI_M,MI_M,SOL_M,SOL_M,FA_M,RE_M,DO_M
};

// note durations: 4 = quarter note, 8 = eighth note, etc.:
//整型数组保存歌曲we wish you a merry christmas所有音符的节奏
int noteDurations[] = {
  2, 2, 4, 4, 4, 4, 2, 2, 2,
  2, 4, 4, 4, 4, 2, 2,
  2, 2, 4, 4, 4, 4, 2, 2,
  4, 4, 2, 2, 2, 1,
  2, 2, 2, 2, 1, 2, 2, 2, 2, 2,
  2, 2, 2, 2, 2, 2, 4, 4, 2, 2, 2, 1
};

int noteDurations1[] = {
  4,4,2,4,4,2,4,4,4,4,1,
  4,4,4,8,4,4,4,8,8,4,4,4,4,2,2,
  4,4,2,4,4,2,4,4,4,4,1,
  4,4,4,8,4,4,4,8,8,4,4,4,4,2
  };

int lightPin[] = {
  2,3,12
};


void setup() {
  pinMode(9,OUTPUT);
  pinMode(2,OUTPUT);
  pinMode(3,OUTPUT);
  pinMode(12,OUTPUT);

}

void loop() {
  //闪烁部分
  digitalWrite(2,HIGH);
  delay(700);
  digitalWrite(3,HIGH);
  delay(700);
  digitalWrite(12,HIGH);
  delay(100);
  digitalWrite(2,LOW);
  delay(700);
  digitalWrite(3,LOW);
  delay(700);
  digitalWrite(12,LOW);
  delay(1000);
  digitalWrite(2,HIGH);
  delay(700);
  digitalWrite(2,LOW);
  delay(700);
  digitalWrite(3,HIGH);
  delay(700);
  digitalWrite(3,LOW);
  delay(700);
  digitalWrite(12,HIGH);
  delay(700);
  digitalWrite(12,LOW);
  delay(700);
  digitalWrite(2,HIGH);
  digitalWrite(3,HIGH);
  digitalWrite(12,HIGH);
  delay(500);
  digitalWrite(2,LOW);
  digitalWrite(3,LOW);
  digitalWrite(12,LOW);
  delay(500);
  digitalWrite(2,HIGH);
  digitalWrite(3,HIGH);
  digitalWrite(12,HIGH);
  delay(500);
  digitalWrite(2,LOW);
  digitalWrite(3,LOW);
  digitalWrite(12,LOW);
  delay(500);
  digitalWrite(2,HIGH);
  digitalWrite(3,HIGH);
  digitalWrite(12,HIGH);
  delay(500);
  digitalWrite(2,LOW);
  digitalWrite(3,LOW);
  digitalWrite(12,LOW);
  delay(500);
  digitalWrite(2,HIGH);
  digitalWrite(3,HIGH);
  digitalWrite(12,HIGH);
  delay(500);
  digitalWrite(2,LOW);
  digitalWrite(3,LOW);
  digitalWrite(12,LOW);
  delay(500);
  digitalWrite(2,HIGH);
  digitalWrite(3,HIGH);
  digitalWrite(12,HIGH);
  delay(500);
  digitalWrite(2,LOW);
  digitalWrite(3,LOW);
  digitalWrite(12,LOW);
  delay(500);
  digitalWrite;(2,HIGH);
  digitalWrite(3,HIGH);
  digitalWrite(12,HIGH);
  delay(500);
  digitalWrite(2,LOW);
  digitalWrite(3,LOW);
  digitalWrite(12,LOW);
  delay(500);
  digitalWrite(2,HIGH);
  digitalWrite(3,HIGH);
  digitalWrite(12,HIGH);
  delay(500);
  digitalWrite(2,LOW);
  digitalWrite(3,LOW);
  digitalWrite(12,LOW);
  delay(500);
  //音乐部分
  // 通过for循环安顺序和节奏播放we wish you a merry christmas， 每个循环播放一个音符
  // thisNote的数值表示当前循环次数，从零开始

  int lightIdx=0;

  //计算melody数组长度，并保存到整型变量melodyCount中
  int melodyCount;
  GET_ARRAY_LEN(melody,melodyCount);
  
  for (int thisNote = 0; thisNote < melodyCount; thisNote++) {

    //根据节奏数值来计算音符的持续时间, 1000除以节奏代码，比如4分音符就是1000/4=250毫秒
    int noteDuration = 1000 / noteDurations[thisNote]; 

    
    //在9通道输出音符melody[thisNote]，并持续noteDuration毫秒
    tone(9, melody[thisNote], noteDuration);

    
    //每个音符和下一个音符之间间隔30毫秒，可能是为了等蜂鸣器的震动完全停止，这样不会干扰下一个音符的播放
    delay(noteDuration + 30);
    
  }

  //两次播放之间间隔10秒钟（10000毫秒）
  delay(3000);

  //计算melody1数组长度，并保存到整型变量melodyCount中
  GET_ARRAY_LEN(melody1,melodyCount);
  for (int thisNote = 0; thisNote < melodyCount; thisNote++) {

    //根据节奏数值来计算音符的持续时间, 1000除以节奏代码，比如4分音符就是1000/4=250毫秒
    int noteDuration = 1000 / noteDurations1[thisNote]; 

    //在9通道输出音符melody[thisNote]，并持续noteDuration毫秒
    tone(9, melody1[thisNote], noteDuration);
    
    //每个音符和下一个音符之间间隔30毫秒，可能是为了等蜂鸣器的震动完全停止，这样不会干扰下一个音符的播放
    delay(noteDuration + 30);
  }

  //两次播放之间间隔10秒钟（10000毫秒）
  delay(2000);
  
}
