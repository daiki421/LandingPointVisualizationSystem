import processing.serial.*;

PImage foot_img, landingPoint100, landingPoint80, landingPoint60, landingPoint40, landingPoint20, backButton, landingPointWhite, right_foot, left_foot;
PImage whiteBoardGood_imgLeft, whiteBoardBad_imgLeft, whiteBoardGood_imgRight, whiteBoardBad_imgRight;
Serial myPort1, myPort2;
int analog1_0_high, analog1_0_low, analog1_1_high, analog1_1_low, analog1_2_high, analog1_2_low, analog1_3_high, analog1_3_low, analog1_4_high, analog1_4_low; // Arduinoから送られてきた分割された値
int analog2_0_high, analog2_0_low, analog2_1_high, analog2_1_low, analog2_2_high, analog2_2_low, analog2_3_high, analog2_3_low, analog2_4_high, analog2_4_low;
int sensorValueLeft0, sensorValueLeft1, sensorValueLeft2, sensorValueLeft3, sensorValueLeft4, sensorValueRight0, sensorValueRight1, sensorValueRight2, sensorValueRight3, sensorValueRight4; // 分割された値から算出したセンサ値
PrintWriter output, output2, output3, output4, outputPressOrder; // ファイル変数
boolean isLandingPoint1_0 = false, isLandingPoint1_1 = false, isLandingPoint1_2 = false, isLandingPoint1_3 = false, isLandingPoint1_4 = false;// 圧力センサの接地判定
boolean isLandingPoint2_0 = false, isLandingPoint2_1 = false, isLandingPoint2_2 = false, isLandingPoint2_3 = false, isLandingPoint2_4 = false;
boolean isDrawLeft0 = false, isDrawLeft1 = false, isDrawLeft2 = false, isDrawLeft3 = false, isDrawLeft4 = false, isDrawRight0 = false, isDrawRight1 = false, isDrawRight2 = false, isDrawRight3 = false, isDrawRight4 = false;
double time1 = 0, time2 = 0; // Arduinoで取得した時間
double groundTimeLeft = 0, groundTimeRight = 0; // 地面に初めて接地した時間
double landingTimeRight = 0, landingTimeLeft = 0; // 地面から離れた時間
double diffTime = 0; // 片足が接地してからもう片足が接地するまでの時間
double timeIntervalLeft0_1 = 0, timeIntervalLeft1_2 = 0, timeIntervalLeft2_3 = 0, timeIntervalLeft3_4 = 0, timeIntervalRight0_1 = 0, timeIntervalRight1_2 = 0, timeIntervalRight2_3 = 0, timeIntervalRight3_4 = 0; // 各センサ間の設置時間間隔
double sensorReactedTimeLeft[] = {0, 0, 0, 0, 0}, sensorReactedTimeRight[] = {0, 0, 0, 0, 0}; // 各センサが地面に設置した時間
double evacuateLeft0 = 0, evacuateLeft1 = 0, evacuateLeft2 = 0, evacuateLeft3 = 0, evacuateLeft4 = 0, evacuateRight0 = 0, evacuateRight1 = 0, evacuateRight2 = 0, evacuateRight3 = 0, evacuateRight4 = 0; //センサが反応した時間を一時保存
int runningSpeed = 12;// トレッドミルの時速を指定
int peak1_0 = 2000, peak1_1 = 2000, peak1_2 = 2000, peak1_3 = 2000, peak1_4 = 2000, peak2_0 = 2000, peak2_1 = 2000, peak2_2 = 2000, peak2_3 = 2000, peak2_4 = 2000; // 各圧力センサ値のピーク
double peakTime1_0 = 0, peakTime1_1 = 0, peakTime1_2 = 0, peakTime1_3 = 0, peakTime1_4 = 0, peakTime2_0 = 0, peakTime2_1 = 0, peakTime2_2 = 0, peakTime2_3 = 0, peakTime2_4 = 0;
int pressOrderLeft[] = {0, 0, 0, 0, 0}, pressOrderRight[] = {0, 0, 0, 0, 0}; // 着地点の順番を格納する
int orderLeft = 1, orderRight = 1; // 着地点の順番
double startTime = 0;
double diffGroundTimeLeft = 0, diffLandingTimeLeft = 0, diffGroundTimeRight = 0, diffLandingTimeRight = 0;
float x1, y1;
float x2, y2;
int count = 4;

void setup() {
  startTime = System.nanoTime();
  size(800, 800);
  output = createWriter("Test1.csv");
  outputPressOrder = createWriter("TestOrder1.csv");
  //output = createWriter("SensorData1"+year()+"-"+month()+"-"+day()+"-"+hour()+"-"+minute()+"-"+second()+".csv");
  //output2 = createWriter("SensorData2"+year()+"-"+month()+"-"+day()+"-"+hour()+"-"+minute()+"-"+second()+".csv");
  //output3 = createWriter("SensorData3"+year()+"-"+month()+"-"+day()+"-"+hour()+"-"+minute()+"-"+second()+".csv");
  //output4 = createWriter("SensorData4"+year()+"-"+month()+"-"+day()+"-"+hour()+"-"+minute()+"-"+second()+".csv");
  //outputPressOrder = createWriter("PressOrder"+year()+"-"+month()+"-"+day()+"-"+hour()+"-"+minute()+"-"+second()+".csv");
  output.println("time1,Left0,,Left1,,Left2,,Left3,,Left4,,time2,Right0,,Right1,,Right2,,Right3,,Right4,"+String.valueOf(runningSpeed)+"km/h");
  //output2.println("time1,Left0,,Left1,,Left2,,Left3,,Left4,,time2,Right0,,Right1,,Right2,,Right3,,Right4");
  //output3.println("time1,Left0,,Left1,,Left2,,Left3,,Left4,,time2,Right0,,Right1,,Right2,,Right3,,Right4");
  //output4.println("time1,Left0,,Left1,,Left2,,Left3,,Left4,,time2,Right0,,Right1,,Right2,,Right3,,Right4");
  outputPressOrder.println("TimeLeft,StrideLeft(cm),ContactTimeLeft(s),Left0,Left1,Left2,Left3,Left4,LPeak1,LPTime1,LPeak2,LPTime2,LPeak3,LPTime3,LPeak4,LPTime4,LPeak5,LPTime5,TIL0_1,TIL1_2,TIL2_3,TIL3_4,,TimeRight,StrideRight,ContactTimeRight,Right0,Right1,Right2,Right3,Right4,RPeak1,RPTime1,RPeak2,RPTime2,RPeak3,RPTime3,RPeak4,RPTime4,RPeak5,RPTime5,TIR0_1,TIR1_2,TIR2_3,TIR3_4");
  // システム1号機
  //myPort2 = new Serial(this, "/dev/tty.HC-06-DevB", 9600);
  //myPort2 = new Serial(this, "/dev/tty.HC-06-DevB-1", 9600);
  // システム2号機
  //myPort1 = new Serial(this, "/dev/tty.HC-06-DevB-2", 9600);
  //myPort2 = new Serial(this, "/dev/tty.HC-06-DevB-3", 9600);
  foot_img = loadImage("foot_sole900.jpeg");
  left_foot = loadImage("left_foot.jpg");
  right_foot = loadImage("right_foot.jpg");
  landingPoint100 = loadImage("LandingPoint_a100.png");
  landingPoint80 = loadImage("LandingPoint_a80.png");
  landingPoint60 = loadImage("LandingPoint_a60.png");
  landingPoint40 = loadImage("LandingPoint_a40.png");
  landingPoint20 = loadImage("LandingPoint_a20.png");
  landingPointWhite = loadImage("white.png");

  background(255);
  image(left_foot, 25, 150, 250, 600);
  image(right_foot, 520, 150, 250, 600);
  fill(100, 100, 255);
  rect(325, 0, 150, 800);
  line(315, 800 - 200, 485, 800 - 200);
  line(315, 800 - 400, 485, 800 - 400);
  line(315, 800 - 600, 485, 800 - 600);
  textSize(25);
  fill(0);
  text("50", 285, 600);
  text("100", 275, 400);
  text("150", 275, 200);
  text("50", 475, 600);
  text("100", 475, 400);
  text("150", 475, 200);
  textSize(15);
  text("cm", 295, 210);
  text("cm", 295, 410);
  text("cm", 295, 610);
  text("cm", 495, 210);
  text("cm", 495, 410);
  text("cm", 485, 610);

  // 左足サークル
  noFill();
  ellipse(144, 674, 75, 75);
  ellipse(230, 210, 65, 65);
  ellipse(55, 280, 65, 65);
  ellipse(230, 330, 65, 65);
  ellipse(70, 380, 65, 65);
  // 右足サークル
  ellipse(650, 675, 75, 75);
  ellipse(565, 210, 65, 65);
  ellipse(740, 280, 65, 65);
  ellipse(565, 330, 65, 65);
  ellipse(725, 370, 65, 65);
  // 左足テキスト
  textSize(50);
  fill(0);
  text("1", 128, 690);
  text("4", 40, 300);
  text("3", 215, 350);
  text("2", 55, 400);
  text("5", 215, 225);
  // 右足テキスト
  textSize(50);
  fill(0);
  text("1", 633, 695);
  text("2", 710, 390);
  text("3", 550, 350);
  text("4", 725, 297);
  text("5", 550, 225);
  fill(255);
  // 左足矢印
  //putArrow(true, 4, 2); // かかと-親指下
  //drawArrow(145, 670, 70, 385); // かかと-小指下
  //drawArrow(145, 670, 60, 285); // かかと-小指
  //drawArrow(145, 670, 230, 220); // かかと-親指
  //drawArrow(70, 385, 230, 335); // 小指下-親指下
  //drawArrow(70, 385, 60, 285); // 小指下-小指
  //drawArrow(70, 385, 230, 220); // 小指下-親指
  //drawArrow(230, 335, 70, 385); // 親指下-小指下
  //drawArrow(230, 335, 60, 285); // 親指下-小指
  //drawArrow(230, 335, 230, 220); // 親指下-親指
  //drawArrow(60, 285, 70, 385); // 小指-小指下
  //drawArrow(60, 285, 230, 335); // 小指-親指下
  //drawArrow(60, 285, 230, 220); // 小指-親指
  //drawArrow(230, 220, 70, 385); // 親指-小指下
  //drawArrow(230, 220, 230, 335); // 親指-親指下
  //drawArrow(230, 220, 60, 285); // 親指-小指
  // 右足矢印
  //drawArrow(650, 675, 565, 335); // かかと-親指下
  //drawArrow(650, 675, 720, 385); // かかと-小指下
  //drawArrow(650, 675, 565, 220); // かかと-親指
  //drawArrow(650, 675, 730, 285); // かかと-小指
  //drawArrow(720, 385, 650, 675); // 小指下-かかと
  //drawArrow(720, 385, 565, 335); // 小指下-親指下
  //drawArrow(720, 385, 565, 220); // 小指下-親指
  //drawArrow(720, 385, 730, 285); // 小指下-小指
  //drawArrow(565, 335, 720, 385); // 親指下-小指下
  //drawArrow(565, 335, 730, 285); // 親指下-小指
  //drawArrow(565, 335, 565, 220); // 親指下-親指
  //drawArrow(730, 285, 720, 385); // 小指-小指下
  //drawArrow(730, 285, 565, 335); // 小指-親指下
  //drawArrow(730, 285, 565, 220); // 小指-親指
  //drawArrow(565, 220, 720, 385); // 親指-小指下
  //drawArrow(565, 220, 565, 335); // 親指-親指下
  //drawArrow(565, 220, 730, 285); // 親指-小指
}

void draw() {
  fill(255);
  strokeWeight(1);
  rect(0, 0, 200, 30);
  textSize(20);
  fill(0);
  text((int)((System.nanoTime() - startTime)/1000000000)+"second", 20, 20);
  if (mousePressed) {
    if (mouseX >= 0 && mouseX < 800 && mouseY >= 0 && mouseY <= 800) {
      output.flush(); // データ書き込み
      output.close(); // ファイル閉じる
      output2.flush(); // データ書き込み
      output2.close(); // ファイル閉じる
      output3.flush(); // データ書き込み
      output3.close(); // ファイル閉じる
      output4.flush(); // データ書き込み
      output4.close(); // ファイル閉じる
      outputPressOrder.flush();
      outputPressOrder.close();
      println("File Close");
    }
  }
  delay(300);
  putArrow(true, count, count - 1);
  if (count == 0) {
    count = 5;
    image(left_foot, 25, 150, 250, 600);
    noFill();
    strokeWeight(1);
    ellipse(144, 674, 75, 75);
    ellipse(230, 210, 65, 65);
    ellipse(55, 280, 65, 65);
    ellipse(230, 330, 65, 65);
    ellipse(70, 380, 65, 65);
    textSize(50);
    fill(0);
    text("1", 128, 690);
    text("4", 40, 300);
    text("3", 215, 350);
    text("2", 55, 400);
    text("5", 215, 225);
  }
  count--;
  
  // Left
  // 親指
  //if (myPort1.available()>0) {
  //  if (sensorValueLeft0 <= 1010) {
  //    isLandingPoint1_0 = true;
  //    // 着地した順が格納されてなかったら着地順を入れる
  //    if (pressOrderLeft[0] == 0) {
  //      // センサーが反応した時間取得
  //      sensorReactedTimeLeft[0] = System.nanoTime() - startTime;
  //      pressOrderLeft[0] = orderLeft;
  //      orderLeft++;
  //    }
  //    // 取得したセンサ値が前の値より小さければピークを更新
  //    if (sensorValueLeft0 <= peak1_0) {
  //      peak1_0 = sensorValueLeft0;
  //      peakTime1_0 = (System.nanoTime() - startTime) - sensorReactedTimeLeft[0];
  //    } else if (sensorValueLeft0 > peak1_0) {
  //    }
  //    // 着地時に他のセンサが着地判定していたら
  //    if (isLandingPoint1_1 || isLandingPoint1_2 || isLandingPoint1_3 || isLandingPoint1_4) {
  //      for (int i = 0; i < 5; i++) {
  //        if (pressOrderLeft[i] == orderLeft - 1) {
  //          putArrow(true, 0, i);
  //        }
  //      }
  //    } else {
  //      if (groundTimeLeft != 0) {
  //      } else {
  //        groundTimeLeft = sensorReactedTimeLeft[0]; // 着地時間記録
  //        diffGroundTimeLeft = sensorReactedTimeLeft[0]; // スタートから接地までの時間
  //        diffTime = diffGroundTimeLeft - diffGroundTimeRight; // 右足が接地してから左足が接地するまでの時間
  //        //println("DiffTimeLeft="+nf((float)(diffTime/1000000000), 3, 3));
  //        fill(255, 255, 255);
  //        rect(325, 0, 150, 800);
  //        double stride = diffTime/1000000000*runningSpeed*1000*100/3600; // 秒 * cm/s
  //        fill(100, 100, 255);
  //        rect(325, 800 - (float)stride*4, 150, (float)stride*4);
  //        fill(255);
  //        // println("Stride="+nf((float)stride, 3, 3)+"cm");
  //      }
  //    }
  //    if (isDrawLeft0) {
  //    } else {
  //      if (pressOrderLeft[0] == 1) {
  //        isDrawLeft0 = true;
  //        image(landingPoint100, 200, 180, 60, 60);
  //      } else if (pressOrderLeft[0] == 2) {
  //        isDrawLeft0 = true;
  //        image(landingPoint100, 200, 180, 60, 60);
  //      } else if (pressOrderLeft[0] == 3) {
  //        isDrawLeft0 = true;
  //        image(landingPoint100, 200, 180, 60, 60);
  //      } else if (pressOrderLeft[0] == 4) {
  //        isDrawLeft0 = true;
  //        image(landingPoint100, 200, 180, 60, 60);
  //      } else if (pressOrderLeft[0] == 5) {
  //        isDrawLeft0 = true;
  //        noStroke();
  //        fill(255);
  //        ellipse(230, 210, 60, 60);
  //      }
  //    }
  //  } else {
  //    isLandingPoint1_0 = false;
  //  }
  //  // 小指
  //  if (sensorValueLeft1 <= 1010) {
  //    isLandingPoint1_1 = true;
  //    if (pressOrderLeft[1] == 0) {
  //      sensorReactedTimeLeft[1] = System.nanoTime() - startTime;
  //      pressOrderLeft[1] = orderLeft;
  //      orderLeft++;
  //    }
  //    if (sensorValueLeft1 <= peak1_1) {
  //      peak1_1 = sensorValueLeft1;
  //      peakTime1_1 = (System.nanoTime() - startTime) - sensorReactedTimeLeft[1];
  //    } else if (sensorValueLeft1 > peak1_1) {
  //    }
  //    if (isLandingPoint1_0 || isLandingPoint1_2 || isLandingPoint1_3 || isLandingPoint1_4) {
  //      for (int i = 0; i < 5; i++) {
  //        if (pressOrderLeft[i] == orderLeft - 1) {
  //          putArrow(true, 1, i);
  //        }
  //      }
  //    } else {
  //      if (groundTimeLeft != 0) {
  //      } else {
  //        groundTimeLeft = sensorReactedTimeLeft[1]; // 着地時間記録
  //        diffGroundTimeLeft = sensorReactedTimeLeft[1]; // スタートから接地までの時間
  //        diffTime = diffGroundTimeLeft - diffGroundTimeRight; // 右足を離してから左足が着くまでの時間
  //        //println("DiffTimeLeft="+diffTime/1000000000);
  //        fill(255, 255, 255);
  //        rect(325, 0, 150, 800);
  //        double stride = diffTime/1000000000*runningSpeed*1000*100/3600;
  //        fill(100, 100, 255);
  //        rect(325, 800 - (float)stride*4, 150, (float)stride*4);
  //        fill(255);
  //        //println("Stride="+nf((float)stride, 3, 3)+"cm");
  //      }
  //    }
  //    if (isDrawLeft1) {
  //    } else {
  //      if (pressOrderLeft[1] == 1) {
  //        isDrawLeft1 = true;
  //        image(landingPoint100, 35, 263, 40, 40);
  //      } else if (pressOrderLeft[1] == 2) {
  //        isDrawLeft1 = true;
  //        image(landingPoint100, 35, 263, 40, 40);
  //      } else if (pressOrderLeft[1] == 3) {
  //        isDrawLeft1 = true;
  //        image(landingPoint100, 35, 263, 40, 40);
  //      } else if (pressOrderLeft[1] == 4) {
  //        isDrawLeft1 = true;
  //        noStroke();
  //        fill(255);
  //        ellipse(55, 283, 40, 40);
  //      } else if (pressOrderLeft[1] == 5) {
  //        isDrawLeft1 = true;
  //        image(landingPoint100, 35, 263, 40, 40);
  //      }
  //    }
  //  } else {
  //    isLandingPoint1_1 = false;
  //  }
  //  // 親指下
  //  if (sensorValueLeft2 <= 1000) {
  //    isLandingPoint1_2 = true;
  //    if (pressOrderLeft[2] == 0) {
  //      sensorReactedTimeLeft[2] = System.nanoTime() - startTime;
  //      pressOrderLeft[2] = orderLeft;
  //      orderLeft++;
  //    }
  //    if (sensorValueLeft2 <= peak1_2) {
  //      peak1_2 = sensorValueLeft2;
  //      peakTime1_2 = (System.nanoTime() - startTime) - sensorReactedTimeLeft[2];
  //    } else if (sensorValueLeft2 > peak1_2) {
  //    }
  //    if (isLandingPoint1_0 || isLandingPoint1_1 || isLandingPoint1_3 || isLandingPoint1_4) {
  //      for (int i = 0; i < 5; i++) {
  //        if (pressOrderLeft[i] == orderLeft - 1) {
  //          putArrow(true, 2, i);
  //        }
  //      }
  //    } else {
  //      if (groundTimeLeft != 0) {
  //      } else {
  //        groundTimeLeft = sensorReactedTimeLeft[2]; // 着地時間記録
  //        diffGroundTimeLeft = sensorReactedTimeLeft[2]; // スタートから接地までの時間
  //        diffTime = diffGroundTimeLeft - diffGroundTimeRight; // 左足を離してから右足が着くまでの時間
  //        //println("DiffTimeLeft="+diffTime/1000000000);
  //        fill(255, 255, 255);
  //        rect(325, 0, 150, 800);
  //        double stride = diffTime/1000000000*runningSpeed*1000*100/3600;
  //        fill(100, 100, 255);
  //        rect(325, 800 - (float)stride*4, 150, (float)stride*4);
  //        fill(255);
  //        //println("Stride="+nf((float)stride, 3, 3)+"cm");
  //      }
  //    }
  //    if (isDrawLeft2) {
  //    } else {
  //      if (pressOrderLeft[2] == 1) {
  //        isDrawLeft2 = true;
  //        image(landingPoint100, 200, 300, 60, 60);
  //      } else if (pressOrderLeft[2] == 2) {
  //        isDrawLeft2 = true;
  //        image(landingPoint100, 200, 300, 60, 60);
  //      } else if (pressOrderLeft[2] == 3) {
  //        isDrawLeft2 = true;
  //        noStroke();
  //        fill(255);
  //        ellipse(230, 330, 60, 60);
  //      } else if (pressOrderLeft[2] == 4) {
  //        isDrawLeft2 = true;
  //        image(landingPoint100, 200, 300, 60, 60);
  //      } else if (pressOrderLeft[2] == 5) {
  //        isDrawLeft2 = true;
  //        image(landingPoint100, 200, 300, 60, 60);
  //      }
  //    }
  //  } else {
  //    isLandingPoint1_2 = false;
  //  }
  //  // 小指下
  //  if (sensorValueLeft3 <= 1010) {
  //    isLandingPoint1_3 = true;
  //    if (pressOrderLeft[3] == 0) {
  //      sensorReactedTimeLeft[3] = System.nanoTime() - startTime;
  //      pressOrderLeft[3] = orderLeft;
  //      orderLeft++;
  //    }
  //    if (sensorValueLeft3 <= peak1_3) {
  //      peak1_3 = sensorValueLeft3;
  //      peakTime1_3 = (System.nanoTime() - startTime) - sensorReactedTimeLeft[3];
  //    } else if (sensorValueLeft3 > peak1_3) {
  //    }
  //    if (isLandingPoint1_0 || isLandingPoint1_1 || isLandingPoint1_2 || isLandingPoint1_4) {
  //      for (int i = 0; i < 5; i++) {
  //        if (pressOrderLeft[i] == orderLeft - 1) {
  //          putArrow(true, 3, i);
  //        }
  //      }
  //    } else {
  //      if (groundTimeLeft != 0) {
  //      } else {
  //        groundTimeLeft = sensorReactedTimeLeft[3]; // 着地時間記録
  //        diffGroundTimeLeft = sensorReactedTimeLeft[3]; // スタートから接地までの時間
  //        diffTime = diffGroundTimeLeft - diffGroundTimeRight; // 左足を離してから右足が着くまでの時間
  //        //println("DiffTimeLeft="+diffTime/1000000000);
  //        fill(255, 255, 255);
  //        rect(325, 0, 150, 800);
  //        double stride = diffTime/1000000000*runningSpeed*1000*100/3600;
  //        fill(100, 100, 255);
  //        rect(325, 800 - (float)stride*4, 150, (float)stride*4);
  //        fill(255);
  //        //println("Stride="+nf((float)stride, 3, 3)+"cm");
  //      }
  //    }
  //    if (isDrawLeft3) {
  //    } else {
  //      if (pressOrderLeft[3] == 1) {
  //        isDrawLeft3 = true;
  //        image(landingPoint100, 40, 350, 60, 60);
  //      } else if (pressOrderLeft[3] == 2) {
  //        isDrawLeft3 = true;
  //        noStroke();
  //        fill(255);
  //        ellipse(70, 380, 60, 60);
  //      } else if (pressOrderLeft[3] == 3) {
  //        isDrawLeft3 = true;
  //        image(landingPoint100, 40, 350, 60, 60);
  //      } else if (pressOrderLeft[3] == 4) {
  //        isDrawLeft3 = true;
  //        image(landingPoint100, 40, 350, 60, 60);
  //      } else if (pressOrderLeft[3] == 5) {
  //        isDrawLeft3 = true;
  //        image(landingPoint100, 40, 350, 60, 60);
  //      }
  //    }
  //  } else {
  //    isLandingPoint1_3 = false;
  //  }
  //  // かかと
  //  if (sensorValueLeft4 <= 1010) {
  //    isLandingPoint1_4 = true;
  //    if (pressOrderLeft[4] == 0) {
  //      sensorReactedTimeLeft[4] = System.nanoTime() - startTime;
  //      pressOrderLeft[4] = orderLeft;
  //      orderLeft++;
  //    }
  //    if (sensorValueLeft4 <= peak1_4) {
  //      peak1_4 = sensorValueLeft4;
  //      peakTime1_4 = (System.nanoTime() - startTime) - sensorReactedTimeLeft[4];
  //      println((System.nanoTime() - startTime)+" - "+sensorReactedTimeLeft[4]+" = "+peakTime1_4/1000000000);
  //    } else if (sensorValueLeft4 > peak1_4) {
  //    }
  //    if (isLandingPoint1_0 || isLandingPoint1_1 || isLandingPoint1_2 || isLandingPoint1_3) {
  //      for (int i = 0; i < 5; i++) {
  //        if (pressOrderLeft[i] == orderLeft - 1) {
  //          putArrow(true, 4, i);
  //        }
  //      }
  //    } else {
  //      if (groundTimeLeft != 0) {
  //      } else {
  //        groundTimeLeft = sensorReactedTimeLeft[4]; // 着地時間記録
  //        diffGroundTimeLeft = sensorReactedTimeLeft[4]; // スタートから接地までの時間
  //        diffTime = diffGroundTimeLeft - diffGroundTimeRight; // 左足を離してから右足が着くまでの時間
  //        //println("DiffTimeLeft="+diffTime/1000000000);
  //        fill(255, 255, 255);
  //        rect(325, 0, 150, 800);
  //        double stride = diffTime/1000000000*runningSpeed*1000*100/3600;
  //        fill(100, 100, 255);
  //        rect(325, 800 - (float)stride*4, 150, (float)stride*4);
  //        fill(255);
  //        //println("Stride="+nf((float)stride, 3, 3)+"cm");
  //      }
  //    }
  //    if (isDrawLeft4) {
  //    } else {
  //      if (pressOrderLeft[4] == 1) {
  //        isDrawLeft4 = true;
  //        noStroke();
  //        fill(255);
  //        ellipse(145, 675, 70, 70);
  //      } else if (pressOrderLeft[4] == 2) {
  //        isDrawLeft4 = true;
  //        image(landingPoint100, 110, 640, 70, 70);
  //      } else if (pressOrderLeft[4] == 3) {
  //        isDrawLeft4 = true;
  //        image(landingPoint100, 110, 640, 70, 70);
  //      } else if (pressOrderLeft[4] == 4) {
  //        isDrawLeft4 = true;
  //        image(landingPoint100, 110, 640, 70, 70);
  //      } else if (pressOrderLeft[4] == 5) {
  //        isDrawLeft4 = true;
  //        image(landingPoint100, 110, 640, 70, 70);
  //      }
  //    }
  //  } else {
  //    isLandingPoint1_4 = false;
  //  }

  //  // 全てのセンサで離地判定したら
  //  if (!isLandingPoint1_0 && !isLandingPoint1_1 && !isLandingPoint1_2 && !isLandingPoint1_3 && !isLandingPoint1_4) {
  //    // そもそも踏まれてない時
  //    if (pressOrderLeft[0] == 0 && pressOrderLeft[1] == 0 && pressOrderLeft[2] == 0 && pressOrderLeft[3] == 0 && pressOrderLeft[4] == 0) {
  //    } else { 
  //      noStroke();
  //      fill(255);
  //      ellipse(230, 210, 60, 60);
  //      ellipse(55, 283, 40, 40);
  //      ellipse(230, 330, 60, 60);
  //      ellipse(70, 380, 60, 60);
  //      ellipse(145, 675, 70, 70);
  //      textSize(50);
  //      fill(0);
  //      text("1", 128, 690);
  //      text("2", 55, 400);
  //      text("3", 215, 350);
  //      text("4", 40, 300);
  //      text("5", 215, 225);
  //      fill(255);
  //      isDrawLeft0 = false;
  //      isDrawLeft1 = false;
  //      isDrawLeft2 = false;
  //      isDrawLeft3 = false;
  //      isDrawLeft4 = false;
  //      // 一箇所でもセンサが反応していた場合
  //      if (landingTimeLeft != 0) {
  //      } else {
  //        diffLandingTimeLeft = System.nanoTime() - startTime; // スタートしてから離地までの時間
  //      }
  //      for (int i = 0; i < 5; i++) {
  //        if (pressOrderLeft[i] == 2) {
  //          timeIntervalLeft0_1 = sensorReactedTimeLeft[i] - diffGroundTimeLeft;
  //          evacuateLeft2 = sensorReactedTimeLeft[i];
  //        }
  //        if (pressOrderLeft[i] == 3) {
  //          timeIntervalLeft1_2 = sensorReactedTimeLeft[i] - evacuateLeft2;
  //          evacuateLeft3 = sensorReactedTimeLeft[i];
  //        }
  //        if (pressOrderLeft[i] == 4) {
  //          timeIntervalLeft2_3 = sensorReactedTimeLeft[i] - evacuateLeft3;
  //          evacuateLeft4 = sensorReactedTimeLeft[i];
  //        }
  //        if (pressOrderLeft[i] == 5) {
  //          timeIntervalLeft3_4 = sensorReactedTimeLeft[i] - evacuateLeft4;
  //        }
  //      }
  //      // 計算したものをファイルに保存
  //      if (pressOrderLeft[0] != 0 && pressOrderLeft[1] != 0 && pressOrderLeft[2] != 0 && pressOrderLeft[3] != 0 && pressOrderLeft[4] != 0) {
  //        //outputPressOrder.println((System.nanoTime() - startTime)/1000000000+","+diffTime/1000000000*runningSpeed*1000*100/3600+","+(diffLandingTimeLeft - diffGroundTimeLeft)/1000000000+","+pressOrderLeft[0]+","+pressOrderLeft[1]+","+pressOrderLeft[2]+","+pressOrderLeft[3]+","+pressOrderLeft[4]+","+peak1_0+","+peakTime1_0/1000000000+","+peak1_1+","+peakTime1_1/1000000000+","+peak1_2+","+peakTime1_2/1000000000+","+peak1_3+","+peakTime1_3/1000000000+","+peak1_4+","+peakTime1_4/1000000000+","+timeIntervalLeft0_1/1000000000+","+timeIntervalLeft1_2/1000000000+","+timeIntervalLeft2_3/1000000000+","+timeIntervalLeft3_4/1000000000+","+","+","+","+","+","+","+","+","+","+","+","+","+","+","+","+","+","+","+","+","+","+",");
  //      }
  //    }

  //    peak1_0 = 2000;
  //    peak1_1 = 2000;
  //    peak1_2 = 2000;
  //    peak1_3 = 2000;
  //    peak1_4 = 2000;
  //    // オーダーリセット
  //    for (int i = 0; i < pressOrderLeft.length; i++) {
  //      pressOrderLeft[i] = 0;
  //    }
  //    // 順番の変数リセット
  //    orderLeft = 1;
  //    groundTimeLeft = 0;
  //    landingTimeLeft = 0;
  //  }
  //}

  // Right
  // 親指
  //if (myPort2.available()>0) {
  //  if (sensorValueRight0 <= 1010) {
  //    isLandingPoint2_0 = true;
  //    if (pressOrderRight[0] == 0) {
  //      sensorReactedTimeRight[0] = System.nanoTime() - startTime;
  //      pressOrderRight[0] = orderRight;
  //      orderRight++;
  //    }
  //    if (sensorValueRight0 <= peak2_0) {
  //      peak2_0 = sensorValueRight0;
  //      peakTime2_0 = (System.nanoTime() - startTime) - sensorReactedTimeRight[0];
  //    } else if (sensorValueRight0 > peak2_0) {
  //    }
  //    if (isLandingPoint2_1 || isLandingPoint2_2 || isLandingPoint2_3 || isLandingPoint2_4) {
  //      for (int i = 0; i < 5; i++) {
  //        if (pressOrderRight[i] == orderRight - 1) {
  //          putArrow(false, 0, i);
  //        }
  //      }
  //    } else {
  //      if (groundTimeRight != 0) {
  //      } else {
  //        groundTimeRight = sensorReactedTimeRight[0];
  //        diffGroundTimeRight = sensorReactedTimeRight[0];
  //        diffTime = diffGroundTimeRight - diffGroundTimeLeft;
  //        //println("DiffTimeRight="+diffTime/1000000000);
  //        fill(255, 255, 255);
  //        rect(325, 0, 150, 800);
  //        double stride = diffTime/1000000000*runningSpeed*1000*100/3600;
  //        fill(100, 100, 255);
  //        rect(325, 800 - (float)stride*4, 150, (float)stride*4);
  //        fill(255);
  //        //println("Stride="+nf((float)stride, 3, 3)+"cm");
  //      }
  //    }
  //    if (isDrawRight0) {
  //    } else {
  //      if (pressOrderRight[0] == 1) {
  //        isDrawRight0 = true;
  //        image(landingPoint100, 535, 180, 60, 60);
  //      } else if (pressOrderRight[0] == 2) {
  //        isDrawRight0 = true;
  //        image(landingPoint100, 535, 180, 60, 60);
  //      } else if (pressOrderRight[0] == 3) {
  //        isDrawRight0 = true;
  //        image(landingPoint100, 535, 180, 60, 60);
  //      } else if (pressOrderRight[0] == 4) {
  //        isDrawRight0 = true;
  //        image(landingPoint100, 535, 180, 60, 60);
  //      } else if (pressOrderRight[0] == 5) {
  //        isDrawRight0 = true;
  //        noStroke();
  //        fill(255);
  //        ellipse(565, 210, 60, 60);
  //      }
  //    }
  //  } else {
  //    isLandingPoint2_0 = false;
  //  }
  //  // 小指
  //  if (sensorValueRight1 <= 1010) {
  //    isLandingPoint2_1 = true;
  //    if (pressOrderRight[1] == 0) {
  //      sensorReactedTimeRight[1] = System.nanoTime() - startTime;
  //      pressOrderRight[1] = orderRight;
  //      orderRight++;
  //    }
  //    if (sensorValueRight1 <= peak2_1) {
  //      peak2_1 = sensorValueRight1;
  //      peakTime2_1 = (System.nanoTime() - startTime) - sensorReactedTimeRight[1];
  //    } else if (sensorValueRight1 > peak2_1) {
  //    }
  //    if (isLandingPoint2_0 || isLandingPoint2_2 || isLandingPoint2_3 || isLandingPoint2_4) {
  //      for (int i = 0; i < 5; i++) {
  //        if (pressOrderRight[i] == orderRight - 1) {
  //          putArrow(false, 1, i);
  //        }
  //      }
  //    } else {
  //      if (groundTimeRight != 0) {
  //      } else {
  //        groundTimeRight = sensorReactedTimeRight[1];
  //        diffGroundTimeRight = sensorReactedTimeRight[1];
  //        diffTime = diffGroundTimeRight - diffGroundTimeLeft;
  //        //println("DiffTimeRight="+diffTime/1000000000);
  //        fill(255, 255, 255);
  //        rect(325, 0, 150, 800);
  //        double stride = diffTime/1000000000*runningSpeed*1000*100/3600;
  //        fill(100, 100, 255);
  //        rect(325, 800 - (float)stride*4, 150, (float)stride*4);
  //        fill(255);
  //        //println("Stride="+nf((float)stride, 3, 3)+"cm");
  //      }
  //    }
  //    if (isDrawRight1) {
  //    } else {
  //      if (pressOrderRight[1] == 1) {
  //        isDrawRight1 = true;
  //        image(landingPoint100, 720, 260, 40, 40);
  //      } else if (pressOrderRight[1] == 2) {
  //        isDrawRight1 = true;
  //        image(landingPoint100, 720, 260, 40, 40);
  //      } else if (pressOrderRight[1] == 3) {
  //        isDrawRight1 = true;
  //        image(landingPoint100, 720, 260, 40, 40);
  //      } else if (pressOrderRight[1] == 4) {
  //        isDrawRight1 = true;
  //        noStroke();
  //        fill(255);
  //        ellipse(740, 280, 40, 40);
  //      } else if (pressOrderRight[1] == 5) {
  //        isDrawRight1 = true;
  //        image(landingPoint100, 720, 260, 40, 40);
  //      }
  //    }
  //  } else {
  //    isLandingPoint2_1 = false;
  //  }
  //  // 親指下
  //  if (sensorValueRight2 <= 1010) {
  //    isLandingPoint2_2 = true;
  //    if (pressOrderRight[2] == 0) {
  //      sensorReactedTimeRight[2] = System.nanoTime() - startTime;
  //      pressOrderRight[2] = orderRight;
  //      orderRight++;
  //    }
  //    if (sensorValueRight2 <= peak2_2) {
  //      peak2_2 = sensorValueRight2;
  //      peakTime2_2 = (System.nanoTime() - startTime) - sensorReactedTimeRight[2];
  //    } else if (sensorValueRight2 > peak2_2) {
  //    }
  //    if (isLandingPoint2_0 || isLandingPoint2_1 || isLandingPoint2_3 || isLandingPoint2_4) {
  //      for (int i = 0; i < 5; i++) {
  //        if (pressOrderRight[i] == orderRight - 1) {
  //          putArrow(false, 2, i);
  //        }
  //      }
  //    } else {
  //      if (groundTimeRight != 0) {
  //      } else {
  //        groundTimeRight = sensorReactedTimeRight[2];
  //        diffGroundTimeRight = sensorReactedTimeRight[2];
  //        diffTime = diffGroundTimeRight - diffGroundTimeLeft;
  //        //println("DiffTimeRight="+diffTime/1000000000);
  //        fill(255, 255, 255);
  //        rect(325, 0, 150, 800);
  //        double stride = diffTime/1000000000*runningSpeed*1000*100/3600;
  //        fill(100, 100, 255);
  //        rect(325, 800 - (float)stride*4, 150, (float)stride*4);
  //        fill(255);
  //        //println("Stride="+nf((float)stride, 3, 3)+"cm");
  //      }
  //    }
  //    if (isDrawRight2) {
  //    } else {
  //      if (pressOrderRight[2] == 1) {
  //        isDrawRight2 = true;
  //        image(landingPoint100, 535, 300, 60, 60);
  //      } else if (pressOrderRight[2] == 2) {
  //        isDrawRight2 = true;
  //        image(landingPoint100, 535, 300, 60, 60);
  //      } else if (pressOrderRight[2] == 3) {
  //        isDrawRight2 = true;
  //        noStroke();
  //        fill(255);
  //        ellipse(565, 330, 60, 60);
  //      } else if (pressOrderRight[2] == 4) {
  //        isDrawRight2 = true;
  //        image(landingPoint100, 535, 300, 60, 60);
  //      } else if (pressOrderRight[2] == 5) {
  //        isDrawRight2 = true;
  //        image(landingPoint100, 535, 300, 60, 60);
  //      }
  //    }
  //  } else {
  //    isLandingPoint2_2 = false;
  //  }
  //  // 小指下
  //  if (sensorValueRight3 <= 1010) { // システム1：900, システム2：1000
  //    isLandingPoint2_3 = true;
  //    if (pressOrderRight[3] == 0) {
  //      sensorReactedTimeRight[3] = System.nanoTime() - startTime;
  //      pressOrderRight[3] = orderRight;
  //      orderRight++;
  //    }
  //    if (sensorValueRight3 <= peak2_3) {
  //      peak2_3 = sensorValueRight3;
  //      peakTime2_3 = (System.nanoTime() - startTime) - sensorReactedTimeRight[3];
  //    } else if (sensorValueRight3 > peak2_3) {
  //    }
  //    if (isLandingPoint2_0 || isLandingPoint2_1 || isLandingPoint2_2 || isLandingPoint2_4) {
  //      for (int i = 0; i < 5; i++) {
  //        if (pressOrderRight[i] == orderRight - 1) {
  //          putArrow(false, 3, i);
  //        }
  //      }
  //    } else {
  //      if (groundTimeRight != 0) {
  //      } else {
  //        groundTimeRight = sensorReactedTimeRight[3];
  //        diffGroundTimeRight = sensorReactedTimeRight[3];
  //        diffTime = diffGroundTimeRight - diffGroundTimeLeft;
  //        fill(255, 255, 255);
  //        rect(325, 0, 150, 800);
  //        double stride = diffTime/1000000000*runningSpeed*1000*100/3600;
  //        fill(100, 100, 255);
  //        rect(325, 800 - (float)stride*4, 150, (float)stride*4);
  //        fill(255);
  //        //println("Stride="+nf((float)stride, 3, 3)+"cm");
  //      }
  //    }
  //    if (isDrawRight3) {
  //    } else {
  //      if (pressOrderRight[3] == 1) {
  //        isDrawRight3 = true;
  //        image(landingPoint100, 695, 340, 60, 60);
  //      } else if (pressOrderRight[3] == 2) {
  //        isDrawRight3 = true;
  //        noStroke();
  //        fill(255);
  //        ellipse(725, 370, 60, 60);
  //      } else if (pressOrderRight[3] == 3) {
  //        isDrawRight3 = true;
  //        image(landingPoint100, 695, 340, 60, 60);
  //      } else if (pressOrderRight[3] == 4) {
  //        isDrawRight3 = true;
  //        image(landingPoint100, 695, 340, 60, 60);
  //      } else if (pressOrderRight[3] == 5) {
  //        isDrawRight3 = true;
  //        image(landingPoint100, 695, 340, 60, 60);
  //      }
  //    }
  //  } else {
  //    isLandingPoint2_3 = false;
  //  }
  //  // かかと
  //  if (sensorValueRight4 <= 1015) { // システム1：900, システム2：1000
  //    isLandingPoint2_4 = true;
  //    if (pressOrderRight[4] == 0) {
  //      sensorReactedTimeRight[4] = System.nanoTime() - startTime;
  //      pressOrderRight[4] = orderRight;
  //      orderRight++;
  //    }
  //    if (sensorValueRight4 <= peak2_4) {
  //      peak2_4 = sensorValueRight4;
  //      peakTime2_4 = (System.nanoTime() - startTime) - sensorReactedTimeRight[4];
  //    } else if (sensorValueRight4 > peak2_4) {
  //    }
  //    if (isLandingPoint2_0 || isLandingPoint2_1 || isLandingPoint2_2 || isLandingPoint2_3) {
  //      for (int i = 0; i < 5; i++) {
  //        if (pressOrderRight[i] == orderRight - 1) {
  //          putArrow(false, 4, i);
  //        }
  //      }
  //    } else {
  //      if (groundTimeRight != 0) {
  //      } else {
  //        groundTimeRight = sensorReactedTimeRight[4];
  //        diffGroundTimeRight = sensorReactedTimeRight[4];
  //        diffTime = diffGroundTimeRight - diffGroundTimeLeft;
  //        fill(255, 255, 255);
  //        rect(325, 0, 150, 800);
  //        double stride = diffTime/1000000000*runningSpeed*1000*100/3600;
  //        fill(100, 100, 255);
  //        rect(325, 800 - (float)stride*4, 150, (float)stride*4);
  //        fill(255);
  //        //println("Stride="+nf((float)stride, 3, 3)+"cm");
  //      }
  //    }
  //    if (isDrawRight4) {
  //    } else {
  //      if (pressOrderRight[4] == 1) {
  //        isDrawRight4 = true;
  //        noStroke();
  //        fill(255);
  //        ellipse(650, 675, 70, 70);
  //      } else if (pressOrderRight[4] == 2) {
  //        isDrawRight4 = true;
  //        image(landingPoint100, 615, 640, 70, 70);
  //      } else if (pressOrderRight[4] == 3) {
  //        isDrawRight4 = true;
  //        image(landingPoint100, 615, 640, 70, 70);
  //      } else if (pressOrderRight[4] == 4) {
  //        isDrawRight4 = true;
  //        image(landingPoint100, 615, 640, 70, 70);
  //      } else if (pressOrderRight[4] == 5) {
  //        isDrawRight4 = true;
  //        image(landingPoint100, 615, 640, 70, 70);
  //      }
  //    }
  //  } else {
  //    isLandingPoint2_4 = false;
  //  }

  //  if (!isLandingPoint2_0 && !isLandingPoint2_1 && !isLandingPoint2_2 && !isLandingPoint2_3 && !isLandingPoint2_4) {
  //    if (pressOrderRight[0] == 0 && pressOrderRight[1] == 0 && pressOrderRight[2] == 0 && pressOrderRight[3] == 0 && pressOrderRight[4] == 0) {
  //    } else {
  //      noStroke();
  //      ellipse(565, 210, 60, 60);
  //      ellipse(740, 280, 40, 40);
  //      ellipse(565, 330, 60, 60);
  //      ellipse(725, 370, 60, 60);
  //      ellipse(650, 675, 70, 70);
  //      textSize(50);
  //      fill(0);
  //      text("1", 633, 695);
  //      text("2", 710, 390);
  //      text("3", 550, 350);
  //      text("4", 725, 297);
  //      text("5", 550, 225);
  //      fill(255);
  //      isDrawRight0 = false;
  //      isDrawRight1 = false;
  //      isDrawRight2 = false;
  //      isDrawRight3 = false;
  //      isDrawRight4 = false;
  //      if (landingTimeRight != 0) {
  //      } else {
  //        diffLandingTimeRight = System.nanoTime() - startTime; // 離地した時間
  //      }
  //      for (int i = 0; i < 5; i++) {
  //        if (pressOrderRight[i] == 2) {
  //          timeIntervalRight0_1 = sensorReactedTimeRight[i] - diffGroundTimeRight;
  //          evacuateRight2 = sensorReactedTimeRight[i];
  //        }
  //        if (pressOrderRight[i] == 3) {
  //          timeIntervalRight1_2 = sensorReactedTimeRight[i] - evacuateRight2;
  //          evacuateRight3 = sensorReactedTimeRight[i];
  //        }
  //        if (pressOrderRight[i] == 4) {
  //          timeIntervalRight2_3 = sensorReactedTimeRight[i] - evacuateRight3;
  //          evacuateRight4 = sensorReactedTimeRight[i];
  //        }
  //        if (pressOrderRight[i] == 5) {
  //          timeIntervalRight3_4 = sensorReactedTimeRight[i] - evacuateRight4;
  //        }
  //      }
  //      if (pressOrderRight[0] != 0 && pressOrderRight[1] != 0 && pressOrderRight[2] != 0 && pressOrderRight[3] != 0 && pressOrderRight[4] != 0) {
  //        //outputPressOrder.println(","+","+","+","+","+","+","+","+","+","+","+","+","+","+","+","+","+","+","+","+","+","+","+(System.nanoTime() - startTime)/1000000000+","+diffTime/1000000000*runningSpeed*1000*100/3600+","+(diffLandingTimeRight - diffGroundTimeRight)/1000000000+","+pressOrderRight[0]+","+pressOrderRight[1]+","+pressOrderRight[2]+","+pressOrderRight[3]+","+pressOrderRight[4]+","+peak2_0+","+peakTime2_0/1000000000+","+peak2_1+","+peakTime2_1/1000000000+","+peak2_2+","+peakTime2_2/1000000000+","+peak2_3+","+peakTime2_3/1000000000+","+peak2_4+","+peakTime2_4/1000000000+","+timeIntervalRight0_1/1000000000+","+timeIntervalRight1_2/1000000000+","+timeIntervalRight2_3/1000000000+","+timeIntervalRight3_4/1000000000);
  //      }
  //      peak2_0 = 2000;
  //      peak2_1 = 2000;
  //      peak2_2 = 2000;
  //      peak2_3 = 2000;
  //      peak2_4 = 2000;
  //      for (int i = 0; i < pressOrderRight.length; i++) {
  //        pressOrderRight[i] = 0;
  //      }
  //      orderRight = 1;
  //      groundTimeRight = 0;
  //      landingTimeRight = 0;
  //    }
  //  }
  //}
}

//void serialEvent(Serial port) {
//  if (port == myPort1) {
//    if (port.available() >= 12) {
//      if (port.read() == 'H') {
//        time1 = port.read();
//        analog1_0_high = port.read();
//        analog1_0_low = port.read();
//        analog1_1_high = port.read();
//        analog1_1_low = port.read();
//        analog1_2_high = port.read();
//        analog1_2_low = port.read();
//        analog1_3_high = port.read();
//        analog1_3_low = port.read();
//        analog1_4_high = port.read();
//        analog1_4_low = port.read();
//        sensorValueLeft0 = analog1_0_high*256 + analog1_0_low;
//        sensorValueLeft1 = analog1_1_high*256 + analog1_1_low;
//        sensorValueLeft2 = analog1_2_high*256 + analog1_2_low;
//        sensorValueLeft3 = analog1_3_high*256 + analog1_3_low;
//        sensorValueLeft4 = analog1_4_high*256 + analog1_4_low;
//      }
//    }
//  } else {
//  }
//  if (port == myPort2) {
//    if (port.available() >= 12) {
//      if (port.read() == 'H') {
//        time2 = port.read();
//        analog2_0_high = port.read();
//        analog2_0_low = port.read();
//        analog2_1_high = port.read();
//        analog2_1_low = port.read();
//        analog2_2_high = port.read();
//        analog2_2_low = port.read();
//        analog2_3_high = port.read();
//        analog2_3_low = port.read();
//        analog2_4_high = port.read();
//        analog2_4_low = port.read();
//        sensorValueRight0 = analog2_0_high*256 + analog2_0_low;
//        sensorValueRight1 = analog2_1_high*256 + analog2_1_low;
//        sensorValueRight2 = analog2_2_high*256 + analog2_2_low;
//        sensorValueRight3 = analog2_3_high*256 + analog2_3_low;
//        sensorValueRight4 = analog2_4_high*256 + analog2_4_low;
//      }
//    }
//  } else {
//  }
//  //if ((System.nanoTime() - startTime)/1000000000 >= 0 && (System.nanoTime() - startTime)/1000000000 < 550) {
//  //  if (sensorValueLeft0 > 0 && sensorValueLeft1 > 0 && sensorValueLeft2 > 0 && sensorValueLeft3 > 0 && sensorValueLeft4 > 0 && sensorValueRight0 > 0 && sensorValueRight1 > 0 && sensorValueRight2 > 0 && sensorValueRight3 > 0 && sensorValueRight4 > 0) {
//  //    output.println((System.nanoTime() - startTime)/1000000000+","+sensorValueLeft0+","+","+sensorValueLeft1+","+","+sensorValueLeft2+","+","+sensorValueLeft3+","+","+sensorValueLeft4+","+","+(System.nanoTime() - startTime)/1000000000+","+sensorValueRight0+","+","+sensorValueRight1+","+","+sensorValueRight2+","+","+sensorValueRight3+","+","+sensorValueRight4);
//  //  }
//  //} else if ((System.nanoTime() - startTime)/1000000000 >= 550 && (System.nanoTime() - startTime)/1000000000 < 1100) {
//  //  output.flush();
//  //  output.close();
//  //  if (sensorValueLeft0 > 0 && sensorValueLeft1 > 0 && sensorValueLeft2 > 0 && sensorValueLeft3 > 0 && sensorValueLeft4 > 0 && sensorValueRight0 > 0 && sensorValueRight1 > 0 && sensorValueRight2 > 0 && sensorValueRight3 > 0 && sensorValueRight4 > 0) {
//  //    output2.println((System.nanoTime() - startTime)/1000000000+","+sensorValueLeft0+","+","+sensorValueLeft1+","+","+sensorValueLeft2+","+","+sensorValueLeft3+","+","+sensorValueLeft4+","+","+(System.nanoTime() - startTime)/1000000000+","+sensorValueRight0+","+","+sensorValueRight1+","+","+sensorValueRight2+","+","+sensorValueRight3+","+","+sensorValueRight4);
//  //  }
//  //} else if ((System.nanoTime() - startTime)/1000000000 >= 1100 && (System.nanoTime() - startTime)/1000000000 < 1650) {
//  //  output2.flush();
//  //  output2.close();
//  //  if (sensorValueLeft0 > 0 && sensorValueLeft1 > 0 && sensorValueLeft2 > 0 && sensorValueLeft3 > 0 && sensorValueLeft4 > 0 && sensorValueRight0 > 0 && sensorValueRight1 > 0 && sensorValueRight2 > 0 && sensorValueRight3 > 0 && sensorValueRight4 > 0) {
//  //    output3.println((System.nanoTime() - startTime)/1000000000+","+sensorValueLeft0+","+","+sensorValueLeft1+","+","+sensorValueLeft2+","+","+sensorValueLeft3+","+","+sensorValueLeft4+","+","+(System.nanoTime() - startTime)/1000000000+","+sensorValueRight0+","+","+sensorValueRight1+","+","+sensorValueRight2+","+","+sensorValueRight3+","+","+sensorValueRight4);
//  //  }
//  //} else {
//  //  output3.flush();
//  //  output3.close();
//  //  if (sensorValueLeft0 > 0 && sensorValueLeft1 > 0 && sensorValueLeft2 > 0 && sensorValueLeft3 > 0 && sensorValueLeft4 > 0 && sensorValueRight0 > 0 && sensorValueRight1 > 0 && sensorValueRight2 > 0 && sensorValueRight3 > 0 && sensorValueRight4 > 0) {
//  //    output4.println((System.nanoTime() - startTime)/1000000000+","+sensorValueLeft0+","+","+sensorValueLeft1+","+","+sensorValueLeft2+","+","+sensorValueLeft3+","+","+sensorValueLeft4+","+","+(System.nanoTime() - startTime)/1000000000+","+sensorValueRight0+","+","+sensorValueRight1+","+","+sensorValueRight2+","+","+sensorValueRight3+","+","+sensorValueRight4);
//  //  }
//  //}
//  println("time1="+nf((float)(System.nanoTime() - startTime)/1000000000, 3, 3)+"inByte1_0="+sensorValueLeft0+", inByte1_1="+sensorValueLeft1+", inByte1_2="+sensorValueLeft2+", inByte1_3="+sensorValueLeft3+", inByte1_4="+sensorValueLeft4);
//  println("time2="+nf((float)(System.nanoTime() - startTime)/1000000000, 3, 3)+"inByte2_0="+sensorValueRight0+", inByte2_1="+sensorValueRight1+", inByte2_2="+sensorValueRight2+", inByte2_3="+sensorValueRight3+", inByte2_4="+sensorValueRight4);
//}

void drawArrow(float x1, float y1, float x2, float y2) {
  float a = dist(x1, y1, x2, y2) / 50;
  pushMatrix();
  translate(x2, y2);
  rotate(atan2(y2 - y1, x2 - x1));
  strokeWeight(5.5);
  triangle(- a * 3, - a, 0, 0, - a * 3, a);
  popMatrix();
  line(x1, y1, x2, y2);
}
void putArrow(boolean isLeft, int end, int tip) {
  if (isLeft) {
    if (end == 0) {
      x1 = 230;
      y1 = 220;
    } else if (end == 1) {
      x1 = 60;
      y1 = 285;
    } else if (end == 2) {
      x1 = 230;
      y1 = 335;
    } else if (end == 3) {
      x1 = 70;
      y1 = 385;
    } else if (end == 4) {
      x1 = 145;
      y1 = 670;
    }

    if (tip == 0) {
      x2 = 230;
      y2 = 220;
    } else if (tip == 1) {
      x2 = 60;
      y2 = 285;
    } else if (tip == 2) {
      x2 = 230;
      y2 = 335;
    } else if (tip == 3) {
      x2 = 70;
      y2 = 385;
    } else if (tip == 4) {
      x2 = 145;
      y2 = 670;
    }
  } else {
    if (end == 0) {
      x1 = 230;
      y1 = 220;
    } else if (end == 1) {
      x1 = 60;
      y1 = 285;
    } else if (end == 2) {
      x1 = 230;
      y1 = 335;
    } else if (end == 3) {
      x1 = 70;
      y1 = 385;
    } else if (end == 4) {
      x1 = 145;
      y1 = 670;
    }

    if (tip == 0) {
      x2 = 230;
      y2 = 220;
    } else if (tip == 1) {
      x2 = 60;
      y2 = 285;
    } else if (tip == 2) {
      x2 = 230;
      y2 = 335;
    } else if (tip == 3) {
      x2 = 70;
      y2 = 385;
    } else if (tip == 4) {
      x2 = 145;
      y2 = 670;
    }
  }
  //println(x1+","+y1+","+x2+","+y2);
  drawArrow(x1, y1, x2, y2);
}