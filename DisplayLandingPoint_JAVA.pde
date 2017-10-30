import processing.serial.*;

PImage foot_img, landingPoint_img, backButton, landingPointWhite;
PImage whiteBoardGood_imgLeft, whiteBoardBad_imgLeft, whiteBoardGood_imgRight, whiteBoardBad_imgRight;
Serial myPort1, myPort2;
int analog1_0_high, analog1_0_low, analog1_1_high, analog1_1_low, analog1_2_high, analog1_2_low, analog1_3_high, analog1_3_low, analog1_4_high, analog1_4_low; // Arduinoから送られてきた分割された値
int analog2_0_high, analog2_0_low, analog2_1_high, analog2_1_low, analog2_2_high, analog2_2_low, analog2_3_high, analog2_3_low, analog2_4_high, analog2_4_low;
int sensorValueLeft0, sensorValueLeft1, sensorValueLeft2, sensorValueLeft3, sensorValueLeft4, sensorValueRight0, sensorValueRight1, sensorValueRight2, sensorValueRight3, sensorValueRight4; // 分割された値から算出したセンサ値
int winWidth = 800, winHeight = 800;
PrintWriter output, outputPressOrder; // ファイル変数
boolean isLandingPoint1_0 = false, isLandingPoint1_1 = false, isLandingPoint1_2 = false, isLandingPoint1_3 = false, isLandingPoint1_4 = false;// 圧力センサの接地判定
boolean isLandingPoint2_0 = false, isLandingPoint2_1 = false, isLandingPoint2_2 = false, isLandingPoint2_3 = false, isLandingPoint2_4 = false;
double time1 = 0, time2 = 0; // Arduinoで取得した時間
double lapTime1 = 0, lapTime2 = 0; // 時間を加算するための一時保存用
double groundTimeLeft = 0, groundTimeRight = 0; // 地面に初めて接地した時間 
double landingTimeRight = 0, landingTimeLeft = 0; // 地面から離れた時間
double diffTimeLeft = 0, diffTimeRight = 0; // 片足が接地してからもう片足が接地するまでの時間
double timeIntervalLeft0_1 = 0, timeIntervalLeft1_2 = 0, timeIntervalLeft2_3 = 0, timeIntervalLeft3_4 = 0, timeIntervalRight0_1 = 0, timeIntervalRight1_2 = 0, timeIntervalRight2_3 = 0, timeIntervalRight3_4 = 0; // 各センサ間の設置時間間隔
double sensorReactedTimeLeft[] = {0, 0, 0, 0, 0}, sensorReactedTimeRight[] = {0, 0, 0, 0, 0}; // 各センサが地面に設置した時間
double evacuateLeft0 = 0, evacuateLeft1 = 0, evacuateLeft2 = 0, evacuateLeft3 = 0, evacuateLeft4 = 0, evacuateRight0 = 0, evacuateRight1 = 0, evacuateRight2 = 0, evacuateRight3 = 0, evacuateRight4 = 0; //センサが反応した時間を一時保存
int runningSpeed = 0; 
int peak1_0 = 2000, peak1_1 = 2000, peak1_2 = 2000, peak1_3 = 2000, peak1_4 = 2000, peak2_0 = 2000, peak2_1 = 2000, peak2_2 = 2000, peak2_3 = 2000, peak2_4 = 2000; // 各圧力センサ値のピーク
double peakTime1_0 = 0, peakTime1_1 = 0, peakTime1_2 = 0, peakTime1_3 = 0, peakTime1_4 = 0, peakTime2_0 = 0, peakTime2_1 = 0, peakTime2_2 = 0, peakTime2_3 = 0, peakTime2_4 = 0;
int pressOrderLeft[] = {0, 0, 0, 0, 0}, pressOrderRight[] = {0, 0, 0, 0, 0}; // 着地点の順番を格納する
int orderLeft = 1, orderRight = 1; // 着地点の順番
int crrectPressOrderRight = 0, crrectPressOrderLeft = 0;
int crrectPeak1_0 = 0, crrectPeak1_1 = 0, crrectPeak1_2 = 0, crrectPeak1_3 = 0, crrectPeak1_4 = 0, crrectPeak2_0 = 0, crrectPeak2_1 = 0, crrectPeak2_2 = 0, crrectPeak2_3 = 0, crrectPeak2_4 = 0;
double timeTest = 0;
void setup() {
  size(800, 800);
  output = createWriter("Output"+year()+"-"+month()+"-"+day()+"-"+hour()+"-"+minute()+"-"+second()+".csv");
  outputPressOrder = createWriter("OutputPressOrder"+year()+"-"+month()+"-"+day()+"-"+hour()+"-"+minute()+"-"+second()+".csv");
  output.println("time1,inByte1_0,inByte1_1,inByte1_2,inByte1_3,inByte1_4,time2,inByte2_0,inByte2_1,inByte2_2,inByte2_3,inByte2_4");
  outputPressOrder.println("StrideLeft,ContactTimeLeft,LeftOrder1,LeftOrder2,LeftOrder3,LeftOrder4,LeftOrder5,LPeak1,LPeak2,LPeak3,LPeak4,LPeak5,TIL0_1,TIL1_2,TIL2_3,TIL3_4,,StrideRight,ContactTimeRight,RightOrder1,RightOrder2,RightOrder3,RightOrder4,RightOrder5,RPeak1,RPeak2,RPeak3,RPeak4,RPeak5,TIR0_1,TIR1_2,TIR2_3,TIR3_4");
  // システム1号機
  myPort1 = new Serial(this, "/dev/tty.HC-06-DevB", 9600);
  myPort2 = new Serial(this, "/dev/tty.HC-06-DevB-2", 9600);
  // システム2号機
  //myPort1 = new Serial(this, "/dev/tty.HC-06-DevB-1", 9600);
  //myPort2 = new Serial(this, "/dev/tty.HC-06-DevB-3", 9600);
  foot_img = loadImage("foot_sole900.jpeg");
  landingPoint_img = loadImage("Landing_Point.png");
  landingPointWhite = loadImage("white.png");
  runningSpeed = 3; // トレッドミルの時速を指定
  image(foot_img, 0, 0, 400, 400);
  PFont font = createFont("MS Gothic",48,true);
  fill(0);
  textFont(font);
  textSize(55);
  textAlign(CENTER);
  text("歩幅: 0.0cm", 600, 160);
  text("接地時間: 0.0秒", 600, 400);
  //timeTest = System.nanoTime();
  //println("ナノ秒"+System.nanoTime());
  //println("秒"+System.nanoTime()/1000000000);
  //println(millis());
}

void draw() {
  timeTest = System.nanoTime();
  //println("1"+timeTest);
  if (mousePressed) {
    if (mouseX>=0 && mouseX<800 && mouseY>=0 && mouseY<=800) {
      output.flush(); // データ書き込み
      output.close(); // ファイル閉じる
      outputPressOrder.flush();
      outputPressOrder.close();
      println("File Close");
    }
  }
  
  //Left
  if(myPort1.available()>0){
    if (sensorValueLeft0 <= 1000) {
      isLandingPoint1_0 = true;
      // センサーが反応した時間取得
      sensorReactedTimeLeft[0] = System.nanoTime();
      // 着地した順が格納されてなかったら着地順を入れる
      if (pressOrderLeft[0] == 0) {
        pressOrderLeft[0] = orderLeft;
        orderLeft++;
      }
      // 取得したセンサ値が前の値より小さければピークを更新
      if (sensorValueLeft0 <= peak1_0) {
        peak1_0 = sensorValueLeft0;
        peakTime1_0 = System.nanoTime() - sensorReactedTimeLeft[0];
      } else if (sensorValueLeft0 > peak1_0) {
      }
      // 着地時に他のセンサが着地判定していたら
      if (isLandingPoint1_1 || isLandingPoint1_2 || isLandingPoint1_3 || isLandingPoint1_4) { 
      } else {
        groundTimeLeft = System.nanoTime(); // 着地時間記録
        diffTimeLeft = landingTimeRight - groundTimeLeft; // 左足を離してから右足が着くまでの時間
      }
      //image(landingPoint_img, 190, 0, 180, 180);
    } else {
      isLandingPoint1_0 = false;
      //image(landingPointWhite, winWidth*28/100, 65, 75, 75);
    }
    
    if (sensorValueLeft1 <= 1000) {
      isLandingPoint1_1 = true;
      sensorReactedTimeLeft[1] = System.nanoTime();
      if (pressOrderLeft[1] == 0) {
        pressOrderLeft[1] = orderLeft;
        orderLeft++;
      }
      if (sensorValueLeft1 <= peak1_1) {
        peak1_1 = sensorValueLeft1;
        peakTime1_1 = System.nanoTime() - sensorReactedTimeLeft[1];
      } else if (sensorValueLeft1 > peak1_1) {
      }
      if (isLandingPoint1_0 || isLandingPoint1_2 || isLandingPoint1_3 || isLandingPoint1_4) {
      } else {
        groundTimeLeft = System.nanoTime();
        diffTimeLeft = landingTimeRight - groundTimeLeft;
      }
      //image(landingPoint_img, 9, winHeight*4/25, 100, 100);
    } else {
      isLandingPoint1_1 = false;
      //image(landingPointWhite, 28, 166, 40, 40);
    }
    
    if (sensorValueLeft2 <= 1000) {
      isLandingPoint1_2 = true;
      sensorReactedTimeLeft[2] = System.nanoTime();
      if (pressOrderLeft[2] == 0) {
        pressOrderLeft[2] = orderLeft;
        orderLeft++;
      }
      if (sensorValueLeft2 <= peak1_2) {
        peak1_2 = sensorValueLeft2;
        peakTime1_2 = System.nanoTime() - sensorReactedTimeLeft[2];
      } else if (sensorValueLeft2 > peak1_2) {
      }
      if (isLandingPoint1_0 || isLandingPoint1_1 || isLandingPoint1_3 || isLandingPoint1_4) {
      } else {
        groundTimeLeft = System.nanoTime();
        diffTimeLeft = landingTimeRight - groundTimeLeft;
      }
      //image(landingPoint_img, 170, 120, 210, 210);
    } else {
      isLandingPoint1_2 = false;
      //image(landingPointWhite, 200, winHeight*6/25, 100, 100);
    }
    
    if (sensorValueLeft3 <= 1000) {
      isLandingPoint1_3 = true;
      sensorReactedTimeLeft[3] = System.nanoTime();
      if (pressOrderLeft[3] == 0) {
        pressOrderLeft[3] = orderLeft;
        orderLeft++;
      }
      if (sensorValueLeft3 <= peak1_3) {
        peak1_3 = sensorValueLeft3;
        peakTime1_3 = System.nanoTime() - sensorReactedTimeLeft[3];
      } else if (sensorValueLeft3 > peak1_3) {
      }
      if (isLandingPoint1_0 || isLandingPoint1_1 || isLandingPoint1_2 || isLandingPoint1_4) {
      } else {
        groundTimeLeft = System.nanoTime();
        diffTimeLeft = landingTimeRight - groundTimeLeft;
      }
      //image(landingPoint_img, 5, winHeight*6/25, 170, 170);
    } else {
      isLandingPoint1_3 = false;
      //image(landingPointWhite, 33, winHeight*8/25, 75, 75);
    }
    
    if (sensorValueLeft4 <= 1000) {
      isLandingPoint1_4 = true;
      sensorReactedTimeLeft[4] = System.nanoTime();
      
      if (pressOrderLeft[4] == 0) {
        pressOrderLeft[4] = orderLeft;
        orderLeft++;
      }
      if (sensorValueLeft4 <= peak1_4) {
        peak1_4 = sensorValueLeft4;
        peakTime1_4 = System.nanoTime() - sensorReactedTimeLeft[4];
      } else if (sensorValueLeft4 > peak1_4) {
      }
      if (isLandingPoint1_0 || isLandingPoint1_1 || isLandingPoint1_2 || isLandingPoint1_3) {
      } else {
        groundTimeLeft = System.nanoTime();
        diffTimeLeft = landingTimeRight - groundTimeLeft;
      }
      //image(landingPoint_img, 70, winHeight*18/25, 220, 220);
    } else {
      isLandingPoint1_4 = false;
      //image(landingPointWhite, 111, 660, 90, 90);
    }
    
    // 全てのセンサで離地判定したら
    if (!isLandingPoint1_0 && !isLandingPoint1_1 && !isLandingPoint1_2 && !isLandingPoint1_3 && !isLandingPoint1_4) {
      // そもそも踏まれてない時
      if (pressOrderLeft[0] == 0 && pressOrderLeft[1] == 0 && pressOrderLeft[2] == 0 && pressOrderLeft[3] == 0 && pressOrderLeft[4] == 0) { 
      } else { 
        float stride = random(20, 30);
        float time = random(0.01, 0.04);
        fill(200);
        rect(400, 0, 400, 800);
        fill(0);
        text("歩幅:"+nf(stride, 1, 2)+"cm", 600, 160);
        text("接地時間:"+nf(time, 1, 2)+"s", 600, 400);
        // 一箇所でもセンサが反応していた場合
        landingTimeLeft = System.nanoTime(); // 離地した時間
        for (int i = 0; i < 5; i++) {
          if (pressOrderLeft[i] == 2) {
            timeIntervalLeft0_1 = sensorReactedTimeLeft[i] - groundTimeLeft;
            evacuateLeft2 = sensorReactedTimeLeft[i];
          }
          if (pressOrderLeft[i] == 3) {
            timeIntervalLeft1_2 = sensorReactedTimeLeft[i] - evacuateLeft2;
            evacuateLeft3 = sensorReactedTimeLeft[i];
          }
          if (pressOrderLeft[i] == 4) {
            timeIntervalLeft2_3 = sensorReactedTimeLeft[i] - evacuateLeft3;
            evacuateLeft4 = sensorReactedTimeLeft[i];
          }
          if (pressOrderLeft[i] == 5) {
            timeIntervalLeft3_4 = sensorReactedTimeLeft[i] - evacuateLeft4;
          }
        }
        // 計算したものをファイルに保存
        outputPressOrder.println(diffTimeLeft/1000000000*runningSpeed*1000/3600+","+(landingTimeLeft - groundTimeLeft)/1000000000+","+pressOrderLeft[0]+","+pressOrderLeft[1]+","+pressOrderLeft[2]+","+pressOrderLeft[3]+","+pressOrderLeft[4]+","+peak1_0+","+peak1_1+","+peak1_2+","+peak1_3+","+peak1_4+","+Math.abs(timeIntervalLeft0_1/1000)+","+timeIntervalLeft1_2/1000000000+","+timeIntervalLeft2_3/1000000000+","+timeIntervalLeft3_4/1000000000);
        peak1_0 = 2000;
        peak1_1 = 2000;
        peak1_2 = 2000;
        peak1_3 = 2000;
        peak1_4 = 2000;
        // オーダーリセット
        for (int i = 0; i < pressOrderLeft.length; i++) {
          pressOrderLeft[i] = 0;
        }
        // 順番の変数リセット
        orderLeft = 1;
      }
      
      // 比較アルゴリズム
      // 着地順
      if (pressOrderRight[3] == 1) {
        if (pressOrderRight[2] == 2) {
          if (pressOrderRight[0] == 3) {
            if (pressOrderRight[1] == 4) {
              crrectPressOrderRight++;
            }
          }
        }
      }
      // ストライド
      //if (diffTimeRight*runningSpeed*1000/3600 < 0.07 && (groundTimeRight - distantTimeLeft)*runningSpeed > 正解 + 5cm) {
      //  正解数++;
      //}
      // ピーク(親指)
      if ((880.79/(0.1*(peak2_0*5/1024)/(5-peak2_0*5/1024))+47.96)/1000*9.8/0.16925518916 >= 64 && (880.79/(0.1*(peak2_0*5/1024)/(5-peak2_0*5/1024))+47.96)/1000*9.8/0.16925518916 <= 74) {
        crrectPeak2_0++;
      }
      // センサ間の時間間隔 同上
      
      // 接地時間 同上
      
      
    }
  }
  
  // Right
  if(myPort2.available()>0){
    if (sensorValueRight0 <= 1000) {
      isLandingPoint2_0 = true;
      sensorReactedTimeRight[0] = System.nanoTime();
      if (pressOrderRight[0] == 0) {
        pressOrderRight[0] = orderRight;
        orderRight++;
      }
      if (sensorValueRight0 <= peak2_0) {
        peak2_0 = sensorValueRight0;
        peakTime2_0 = System.nanoTime() - sensorReactedTimeLeft[0];
      } else if (sensorValueRight0 > peak2_0) {
      }
      if (isLandingPoint2_1 || isLandingPoint2_2 || isLandingPoint2_3 || isLandingPoint2_4) {
      } else {
        groundTimeRight = System.nanoTime();
        diffTimeRight = landingTimeLeft - groundTimeRight;
      }
      //image(landingPoint_img, 468, 0, 180, 180);
    } else {
      isLandingPoint2_0 = false;
      //image(landingPointWhite, 500, 65, 75, 75);
    }
    
    if (sensorValueRight1 <= 1000) {
      isLandingPoint2_1 = true;
      sensorReactedTimeRight[1] = System.nanoTime();
      if (pressOrderRight[1] == 0) {
        pressOrderRight[1] = orderRight;
        orderRight++;
      }
      if (sensorValueRight1 <= peak2_1) {
        peak2_1 = sensorValueRight1;
        peakTime2_1 = System.nanoTime() - sensorReactedTimeLeft[1];
      } else if (sensorValueRight1 > peak2_1) {
      }
      if (isLandingPoint2_0 || isLandingPoint2_2 || isLandingPoint2_3 || isLandingPoint2_4) {
      } else {
        groundTimeRight = System.nanoTime();
        diffTimeRight = landingTimeLeft - groundTimeRight;
      }
      //image(landingPoint_img, 715, winHeight*4/25, 100, 100);
    } else {
      isLandingPoint2_1 = false;
      //image(landingPointWhite, 734, 166, 40, 40);
    }
    
    if (sensorValueRight2 <= 1000) {
      isLandingPoint2_2 = true;
      sensorReactedTimeRight[2] = System.nanoTime();
      if (pressOrderRight[2] == 0) {
        pressOrderRight[2] = orderRight;
        orderRight++;
      }
      if (sensorValueRight2 <= peak2_2) {
        peak2_2 = sensorValueRight2;
        peakTime2_2 = System.nanoTime() - sensorReactedTimeLeft[2];
      } else if (sensorValueRight2 > peak2_2) {
      }
      if (isLandingPoint2_0 || isLandingPoint2_1 || isLandingPoint2_3 || isLandingPoint2_4) {
      } else {
        groundTimeRight = System.nanoTime();
        diffTimeRight = landingTimeLeft - groundTimeRight;
      }
      //image(landingPoint_img, winWidth*29/50, 120, 210, 210);
    } else {
      isLandingPoint2_2 = false;
      //image(landingPointWhite, 500, winHeight*12.2/50, 90, 90);
    }
    
    if (sensorValueRight3 <= 900) {
      isLandingPoint2_3 = true;
      sensorReactedTimeRight[3] = System.nanoTime();
      if (pressOrderRight[3] == 0) {
        pressOrderRight[3] = orderRight;
        orderRight++;
      }
      if (sensorValueRight3 <= peak2_3) {
        peak2_3 = sensorValueRight3;
        peakTime2_3 = System.nanoTime() - sensorReactedTimeLeft[3];
      } else if (sensorValueRight3 > peak2_3) {
      }
      if (isLandingPoint2_0 || isLandingPoint2_1 || isLandingPoint2_2 || isLandingPoint2_4) {
      } else {
        groundTimeRight = System.nanoTime();
        diffTimeRight = landingTimeLeft - groundTimeRight;
      }
      //image(landingPoint_img, winWidth*21/25, winHeight*6/25, 170, 170);
    } else {
      isLandingPoint2_3 = false;
      //image(landingPointWhite, winWidth*21.9/25, winHeight*8/25, 75, 75);
    }
    
    if (sensorValueRight4 <= 900) {
      isLandingPoint2_4 = true;
      sensorReactedTimeRight[4] = System.nanoTime();
      if (pressOrderRight[4] == 0) {
        pressOrderRight[4] = orderRight;
        orderRight++;
      }
      if (sensorValueRight4 <= peak2_4) {
        peak2_4 = sensorValueRight4;
        peakTime2_4 = System.nanoTime() - sensorReactedTimeLeft[4];
      } else if (sensorValueRight4 > peak2_4) {
      }
      if (isLandingPoint2_0 || isLandingPoint2_1 || isLandingPoint2_2 || isLandingPoint2_3) { 
      } else {
        groundTimeRight = System.nanoTime();
        diffTimeRight = landingTimeLeft - groundTimeRight;
      }
      //image(landingPoint_img, winWidth*17.5/25, winHeight*18/25, 220, 220);
    } else {
      isLandingPoint2_4 = false;
      //image(landingPointWhite, 601, 660, 90, 90);
    }
    
    if (!isLandingPoint2_0 && !isLandingPoint2_1 && !isLandingPoint2_2 && !isLandingPoint2_3 && !isLandingPoint2_4) {
      if (pressOrderRight[0] == 0 && pressOrderRight[1] == 0 && pressOrderRight[2] == 0 && pressOrderRight[3] == 0 && pressOrderRight[4] == 0) {
      } else {
         float stride = random(20, 30);
        float time = random(0.01, 0.04);
        fill(200);
        rect(400, 0, 400, 800);
        fill(0);
        text("歩幅:"+nf(stride, 1, 2)+"cm", 600, 160);
        text("接地時間:"+nf(time, 1, 2)+"s", 600, 400);
        landingTimeRight = System.nanoTime();
        for (int i = 0; i < 5; i++) {
          if (pressOrderRight[i] == 2) {
            timeIntervalRight0_1 = sensorReactedTimeRight[i] - groundTimeRight;
            evacuateRight2 = sensorReactedTimeRight[i];
          }
          if (pressOrderRight[i] == 3) {
            timeIntervalRight1_2 = sensorReactedTimeRight[i] - evacuateRight2;
            evacuateRight3 = sensorReactedTimeRight[i];
          }
          if (pressOrderRight[i] == 4) {
            timeIntervalRight2_3 = sensorReactedTimeRight[i] - evacuateRight3;
            evacuateRight4 = sensorReactedTimeRight[i];
          }
          if (pressOrderRight[i] == 5) {
            timeIntervalRight3_4 = sensorReactedTimeRight[i] - evacuateRight4;
          }
        }
         outputPressOrder.println(","+","+","+","+","+","+","+","+","+","+","+","+","+","+","+","+","+diffTimeRight/1000000000*runningSpeed*1000/3600+","+(landingTimeRight - groundTimeRight)+","+pressOrderRight[0]+","+pressOrderRight[1]+","+pressOrderRight[2]+","+pressOrderRight[3]+","+pressOrderRight[4]+","+peak2_0+","+peak2_1+","+peak2_2+","+peak2_3+","+peak2_4+","+timeIntervalRight0_1/1000000000+","+timeIntervalRight1_2/1000000000+","+timeIntervalRight2_3/1000000000+","+timeIntervalRight3_4/1000000000);
      }
      peak2_0 = 2000;
      peak2_1 = 2000;
      peak2_2 = 2000;
      peak2_3 = 2000;
      peak2_4 = 2000;
      for (int i = 0; i < pressOrderRight.length; i++) {
          pressOrderRight[i] = 0;
      }
      orderRight = 1;
    }
  }
  //println("2"+System.nanoTime());
  //println("差分"+(System.nanoTime() - timeTest)/1000000+"秒");
}

void serialEvent(Serial port) {
  if (port == myPort1) {
    if (port.available() >= 12) {
      if (port.read() == 'H') {
        time1 = port.read();
        analog1_0_high = port.read();
        analog1_0_low = port.read();
        analog1_1_high = port.read();
        analog1_1_low = port.read();
        analog1_2_high = port.read();
        analog1_2_low = port.read();
        analog1_3_high = port.read();
        analog1_3_low = port.read();
        analog1_4_high = port.read();
        analog1_4_low = port.read();
        sensorValueLeft0 = analog1_0_high*256 + analog1_0_low;
        sensorValueLeft1 = analog1_1_high*256 + analog1_1_low;
        sensorValueLeft2 = analog1_2_high*256 + analog1_2_low;
        sensorValueLeft3 = analog1_3_high*256 + analog1_3_low;
        sensorValueLeft4 = analog1_4_high*256 + analog1_4_low;
      }
    }
  }
  if (port == myPort2) {
    if (port.available() >= 12) {
      if (port.read() == 'H') {
        time2 = port.read();
        analog2_0_high = port.read();
        analog2_0_low = port.read();
        analog2_1_high = port.read();
        analog2_1_low = port.read();
        analog2_2_high = port.read();
        analog2_2_low = port.read();
        analog2_3_high = port.read();
        analog2_3_low = port.read();
        analog2_4_high = port.read();
        analog2_4_low = port.read();
        sensorValueRight0 = analog2_0_high*256 + analog2_0_low;
        sensorValueRight1 = analog2_1_high*256 + analog2_1_low;
        sensorValueRight2 = analog2_2_high*256 + analog2_2_low;
        sensorValueRight3 = analog2_3_high*256 + analog2_3_low;
        sensorValueRight4 = analog2_4_high*256 + analog2_4_low;
      }
    }
  }
  if (sensorValueLeft0 > 0 && sensorValueLeft1 > 0 && sensorValueLeft2 > 0 && sensorValueLeft3 > 0 && sensorValueLeft4 > 0 && sensorValueRight0 > 0 && sensorValueRight1 > 0 && sensorValueRight2 > 0 && sensorValueRight3 > 0 && sensorValueRight4 > 0) {
    output.println(time1+","+sensorValueLeft0+","+sensorValueLeft1+","+sensorValueLeft2+","+sensorValueLeft3+","+sensorValueLeft4+","+time2+","+sensorValueRight0+","+sensorValueRight1+","+sensorValueRight2+","+sensorValueRight3+","+sensorValueRight4);
    //output.println(time2+","+sensorValueRight0+","+sensorValueRight1+","+sensorValueRight2+","+sensorValueRight3+","+sensorValueRight4);
  }
  //println("time1="+time1+"inByte1_0="+sensorValueLeft0+", inByte1_1="+sensorValueLeft1+", inByte1_2="+sensorValueLeft2+", inByte1_3="+sensorValueLeft3+", inByte1_4="+sensorValueLeft4);
  //println("time2="+time2+"inByte2_0="+sensorValueRight0+", inByte2_1="+sensorValueRight1+", inByte2_2="+sensorValueRight2+", inByte2_3="+sensorValueRight3+", inByte2_4="+sensorValueRight4);
}