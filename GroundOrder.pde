import processing.serial.*;
import java.util.Map;
public static final float STRIDE_OFFSET = 30;
public static final float DRAWING_RATIO = 5.7;
// 毎回速度に応じて変える
public static final int SENSOR_ORDER1 = 5;
public static final int SENSOR_ORDER2 = 3;
public static final int SENSOR_ORDER3 = 4;
public static final int SENSOR_ORDER4 = 2;
public static final int SENSOR_ORDER5 = 1;

PImage landingPoint100, landingPoint80, landingPoint60, landingPoint40, landingPoint20, landingPointWhite, right_foot, left_foot;
Serial myPort1, myPort2;
int analog1_0_high, analog1_0_low, analog1_1_high, analog1_1_low, analog1_2_high, analog1_2_low, analog1_3_high, analog1_3_low, analog1_4_high, analog1_4_low; // Arduinoから送られてきた分割された値
int analog2_0_high, analog2_0_low, analog2_1_high, analog2_1_low, analog2_2_high, analog2_2_low, analog2_3_high, analog2_3_low, analog2_4_high, analog2_4_low;
int sensorValueLeft0, sensorValueLeft1, sensorValueLeft2, sensorValueLeft3, sensorValueLeft4, sensorValueRight0, sensorValueRight1, sensorValueRight2, sensorValueRight3, sensorValueRight4; // 分割された値から算出したセンサ値
PrintWriter output, output2, outputPressOrder; // ファイル変数
boolean isLandingPoint1_0 = false, isLandingPoint1_1 = false, isLandingPoint1_2 = false, isLandingPoint1_3 = false, isLandingPoint1_4 = false; // 圧力センサの接地判定
boolean isLandingPoint2_0 = false, isLandingPoint2_1 = false, isLandingPoint2_2 = false, isLandingPoint2_3 = false, isLandingPoint2_4 = false;
boolean isDrawLeft0 = false, isDrawLeft1 = false, isDrawLeft2 = false, isDrawLeft3 = false, isDrawLeft4 = false, isDrawRight0 = false, isDrawRight1 = false, isDrawRight2 = false, isDrawRight3 = false, isDrawRight4 = false;
double time1 = 0, time2 = 0; // Arduinoで取得した時間
double groundTimeLeft = 0, groundTimeRight = 0; // 地面に初めて接地した時間
double landingTimeRight = 0, landingTimeLeft = 0; // 地面から離れた時間
double diffTime = 0; // 片足が接地してからもう片足が接地するまでの時間
double sensorReactedTimeLeft[] = {0, 0, 0, 0, 0}, sensorReactedTimeRight[] = {0, 0, 0, 0, 0}; // 各センサが地面に設置した時間
double evacuateLeft2 = 0, evacuateLeft3 = 0, evacuateLeft4 = 0, evacuateRight2 = 0, evacuateRight3 = 0, evacuateRight4 = 0; //センサが反応した時間を一時保存
int runningSpeed = 6;// トレッドミルの時速を指定
int peak1_0 = 2000, peak1_1 = 2000, peak1_2 = 2000, peak1_3 = 2000, peak1_4 = 2000, peak2_0 = 2000, peak2_1 = 2000, peak2_2 = 2000, peak2_3 = 2000, peak2_4 = 2000; // 各圧力センサ値のピーク
double peakTime1_0 = 0, peakTime1_1 = 0, peakTime1_2 = 0, peakTime1_3 = 0, peakTime1_4 = 0, peakTime2_0 = 0, peakTime2_1 = 0, peakTime2_2 = 0, peakTime2_3 = 0, peakTime2_4 = 0;
int pressOrderLeft[] = {0, 0, 0, 0, 0}, pressOrderRight[] = {0, 0, 0, 0, 0}; // 着地点の順番を格納する
int orderLeft = 1, orderRight = 1; // 着地点の順番
double startTime = 0;
double diffGroundTimeLeft = 0, diffLandingTimeLeft = 0, diffGroundTimeRight = 0, diffLandingTimeRight = 0;
float x1, y1;
float x2, y2;
double dummy = 0;
HashMap<Integer, Double> orderTimeL = new HashMap<Integer, Double>();
HashMap<Integer, Double> orderTimeR = new HashMap<Integer, Double>();
double left1_2, left2_3, left3_4, left4_5, right1_2, right2_3, right3_4, right4_5;

double minStride[] = {0, 0, 0}, maxStride[] = {0, 0, 0}, stride = 0;
boolean firstTouchLeft=false, firstTouchRight=false;

boolean isDrawStrideL = false, isDrawStrideR = false;

void setup() {
  minStride[0] = 40;
  minStride[1] = 71;
  minStride[2] = 93;
  maxStride[0] = 100;
  maxStride[1] = 82;
  maxStride[2] = 110;
  startTime = System.nanoTime();
  size(800, 800);
  //output = createWriter("Test1.csv");
  //outputPressOrder = createWriter("TestOrder1.csv");
  output = createWriter("SensorData1"+year()+"-"+month()+"-"+day()+"-"+hour()+"-"+minute()+"-"+second()+".csv");
  output2 = createWriter("SensorData2"+year()+"-"+month()+"-"+day()+"-"+hour()+"-"+minute()+"-"+second()+".csv");
  outputPressOrder = createWriter("PressOrder"+year()+"-"+month()+"-"+day()+"-"+hour()+"-"+minute()+"-"+second()+".csv");
  output.println("time1,Left0,,Left1,,Left2,,Left3,,Left4,,time2,Right0,,Right1,,Right2,,Right3,,Right4,"+String.valueOf(runningSpeed)+"km/h");
  output2.println("time1,Left0,,Left1,,Left2,,Left3,,Left4,,time2,Right0,,Right1,,Right2,,Right3,,Right4");
  outputPressOrder.println("TimeLeft,StrideLeft(cm),ContactTimeLeft(s),Left0,Left1,Left2,Left3,Left4,LPeak1,LPTime1,LPeak2,LPTime2,LPeak3,LPTime3,LPeak4,LPTime4,LPeak5,LPTime5,TIL0_1,TIL1_2,TIL2_3,TIL3_4,,TimeRight,StrideRight,ContactTimeRight,Right0,Right1,Right2,Right3,Right4,RPeak1,RPTime1,RPeak2,RPTime2,RPeak3,RPTime3,RPeak4,RPTime4,RPeak5,RPTime5,TIR0_1,TIR1_2,TIR2_3,TIR3_4,"+String.valueOf(runningSpeed)+"km/h");
  // システム1号機
  myPort2 = new Serial(this, "/dev/tty.HC-06-DevB-4", 9600); // 2
  //myPort1 = new Serial(this, "/dev/tty.HC-06-DevB-5", 9600); // 1*
  // システム2号機
  myPort1 = new Serial(this, "/dev/tty.HC-06-DevB-2", 9600); // 0
  //myPort1 = new Serial(this, "/dev/tty.HC-06-DevB-3", 9600); // 3
  left_foot = loadImage("left_foot.jpg");
  right_foot = loadImage("right_foot.jpg");
  landingPointWhite = loadImage("white.png");
  landingPoint100 = loadImage("LandingPoint_a100.png");
  background(255);
  image(left_foot, 25, 150, 250, 600);
  image(right_foot, 520, 150, 250, 600);
  // 歩幅フィードバック
  fill(125, 125, 125);
  rect(325, 0, 150, 800);
  line(315, 200, 485, 200);
  line(315, 400, 485, 400);
  line(315, 600, 485, 600);
  if (runningSpeed == 3) {
    drawStride(30, 200);
  } else if (runningSpeed == 6) {
    drawStride(60, 100);
  } else if (runningSpeed == 9) {
    drawStride(80, 140);
  } else {
    drawStride(30, 170);
  }
  drawSensorPos(true, SENSOR_ORDER1, SENSOR_ORDER2, SENSOR_ORDER3, SENSOR_ORDER4, SENSOR_ORDER5);
  drawSensorPos(false, SENSOR_ORDER1, SENSOR_ORDER2, SENSOR_ORDER3, SENSOR_ORDER4, SENSOR_ORDER5);
  // ハッシュマップ初期化
  for (Integer i = 0; i < 5; i++) {
    orderTimeL.put(i+1, dummy);
    orderTimeR.put(i+1, dummy);
  }
  fill(255);
}

void draw() {
  fill(255);
  strokeWeight(1);
  stroke(0);
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
      outputPressOrder.flush();
      outputPressOrder.close();
      println("File Close");
    }
  }
  // Left
  // 親指
  if (myPort1.available()>0) {
    if (sensorValueLeft0 <= 1010) {
      isLandingPoint1_0 = true;
      //image(landingPoint100, 200, 180, 60, 60);
      // センサの接地順序格納
      if (pressOrderLeft[0] == 0) {
        sensorReactedTimeLeft[0] = System.nanoTime() - startTime;
        pressOrderLeft[0] = orderLeft;
        orderTimeL.put(pressOrderLeft[0], sensorReactedTimeLeft[0]);
        // 最初に触れたセンサか判定
        if (orderLeft == 1) {
          image(left_foot, 25, 150, 250, 600);
          strokeWeight(1);
          drawSensorPos(true, SENSOR_ORDER1, SENSOR_ORDER2, SENSOR_ORDER3, SENSOR_ORDER4, SENSOR_ORDER5);
          groundTimeLeft = sensorReactedTimeLeft[0];
          println("GroundTimeLeft="+groundTimeLeft);
          diffTime = groundTimeLeft - groundTimeRight; // 左足を離してから右足が着くまでの時間
          println("DiffTimeL = "+nf((float)diffTime/1000000000, 3, 3));
          stride = diffTime/1000000000*runningSpeed*1000*100/3600;
          println("Stride="+nf((float)stride, 3, 0));
          fill(100, 100, 255);
          if (stride < 30) {
            rect(325, 800, 150, 0);
          } else {
            if (!isDrawStrideL) {
              fill(255, 255, 255);
              stroke(0);
              rect(325, 0, 150, 800);
              if (runningSpeed == 3) {
                if (stride >= minStride[0] && stride <= maxStride[0]) {
                  fill(255, 0, 0);
                } else {
                  fill(125, 125, 125);
                }
              } else if (runningSpeed == 6) {
                if (stride >= minStride[1] && stride <= maxStride[1]) {
                  fill(255, 0, 0);
                } else {
                  fill(125, 125, 125);
                }
              } else if (runningSpeed == 9) {
                if (stride >= minStride[2] && stride <= maxStride[2]) {
                  fill(255, 0, 0);
                } else {
                  fill(125, 125, 125);
                }
              }
              rect(325, 800 - ((float)stride - 30)*5.7, 150, ((float)stride - 30)*5.7);
              textSize(35);
              fill(0);
              text(String.valueOf((int)stride)+"cm", 360, 800 - ((float)stride - 30)*DRAWING_RATIO + STRIDE_OFFSET);
              isDrawStrideL = true;
              isDrawStrideR = false;
            }
          }
          fill(255);
        } else {
          // このセンサの１つ前のセンサを探して矢印描画
          if (isDrawLeft0) {
          } else {
            for (int i = 0; i < 5; i++) {
              if (pressOrderLeft[i] == orderLeft - 1) {
                putArrow(true, i, 0);
                isDrawLeft0 = true;
                //println("3:"+i+"-"+3);
              }
            }
          }
        }
        orderLeft++;
      }
      // 取得したセンサ値が前の値より小さければピークを更新
      if (sensorValueLeft0 <= peak1_0) {
        peak1_0 = sensorValueLeft0;
        peakTime1_0 = (System.nanoTime() - startTime) - sensorReactedTimeLeft[0];
      }
    } else {
      isLandingPoint1_0 = false;
    }
    if (isLandingPoint1_0) {
      noFill();
      stroke(255, 0, 0);
      strokeWeight(0.5);
      ellipse(230, 210, 65, 65);
    } else {
      noFill();
      stroke(0);
      ellipse(230, 210, 65, 65);
    }
    // 小指
    if (sensorValueLeft1 <= 1010) {
      isLandingPoint1_1 = true;
      if (pressOrderLeft[1] == 0) {
        sensorReactedTimeLeft[1] = System.nanoTime() - startTime;
        pressOrderLeft[1] = orderLeft;
        orderTimeL.put(pressOrderLeft[1], sensorReactedTimeLeft[1]);
        // 最初に接地しているか判定
        if (orderLeft == 1) {
          image(left_foot, 25, 150, 250, 600);
          //noFill();
          strokeWeight(1);
          drawSensorPos(true, SENSOR_ORDER1, SENSOR_ORDER2, SENSOR_ORDER3, SENSOR_ORDER4, SENSOR_ORDER5);
          groundTimeLeft = sensorReactedTimeLeft[1];
          println("GroundTimeLeft="+groundTimeLeft);
          diffTime = groundTimeLeft - groundTimeRight; // 左足を離してから右足が着くまでの時間
          println("DiffTimeL = "+nf((float)diffTime/1000000000, 3, 3));
          stride = diffTime/1000000000*runningSpeed*1000*100/3600;
          println("Stride="+nf((float)stride, 3, 3));
          fill(100, 100, 255);
          if (stride < 30) {
            rect(325, 800, 150, 0);
          } else {
            if (!isDrawStrideL) {
              fill(255, 255, 255);
              stroke(0);
              rect(325, 0, 150, 800);
              if (runningSpeed == 3) {
                if (stride >= minStride[0] && stride <= maxStride[0]) {
                  fill(255, 0, 0);
                } else {
                  fill(125, 125, 125);
                }
              } else if (runningSpeed == 6) {
                if (stride >= minStride[1] && stride <= maxStride[1]) {
                  fill(255, 0, 0);
                } else {
                  fill(125, 125, 125);
                }
              } else if (runningSpeed == 9) {
                if (stride >= minStride[2] && stride <= maxStride[2]) {
                  fill(255, 0, 0);
                } else {
                  fill(125, 125, 125);
                }
              }
              rect(325, 800 - ((float)stride - 30)*5.7, 150, ((float)stride - 30)*5.7);
              textSize(35);
              fill(0);
              text(String.valueOf((int)stride)+"cm", 360, 800 - ((float)stride - 30)*DRAWING_RATIO + STRIDE_OFFSET);
              isDrawStrideL = true;
              isDrawStrideR = false;
            }
          }
          fill(255);
        } else {
          // このセンサの１つ前のセンサを探して矢印描画
          if (isDrawLeft1) {
          } else {
            for (int i = 0; i < 5; i++) {
              if (pressOrderLeft[i] == orderLeft - 1) {
                putArrow(true, i, 1);
                isDrawLeft1 = true;
                //println("3:"+i+"-"+3);
              }
            }
          }
        }
        orderLeft++;
      }
      if (sensorValueLeft1 <= peak1_1) {
        peak1_1 = sensorValueLeft1;
        peakTime1_1 = (System.nanoTime() - startTime) - sensorReactedTimeLeft[1];
      } else if (sensorValueLeft1 > peak1_1) {
      }
    } else {
      isLandingPoint1_1 = false;
    }
    if (isLandingPoint1_1) {
      noFill();
      stroke(255, 0, 0);
      strokeWeight(0.5);
      ellipse(55, 280, 65, 65);
    } else {
      noFill();
      stroke(0.5);
      ellipse(55, 280, 65, 65);
    }
    // 親指下
    if (sensorValueLeft2 <=1010) {
      isLandingPoint1_2 = true;
      if (pressOrderLeft[2] == 0) {
        sensorReactedTimeLeft[2] = System.nanoTime() - startTime;
        pressOrderLeft[2] = orderLeft;
        orderTimeL.put(pressOrderLeft[2], sensorReactedTimeLeft[2]);
        // 最初に接地しているか判定
        if (orderLeft == 1) {
          image(left_foot, 25, 150, 250, 600);
          //noFill();
          strokeWeight(1);
          drawSensorPos(true, SENSOR_ORDER1, SENSOR_ORDER2, SENSOR_ORDER3, SENSOR_ORDER4, SENSOR_ORDER5);
          groundTimeLeft = sensorReactedTimeLeft[2];
          println("GroundTimeLeft="+groundTimeLeft);
          diffTime = groundTimeLeft - groundTimeRight; // 左足を離してから右足が着くまでの時間
          println("DiffTimeL = "+nf((float)diffTime/1000000000, 3, 3));
          stride = diffTime/1000000000*runningSpeed*1000*100/3600;
          println("Stride="+nf((float)stride, 3, 3));
          fill(100, 100, 255);
          if (stride < 30) {
            rect(325, 800, 150, 0);
          } else {
            if (!isDrawStrideL) {
              fill(255, 255, 255);
              stroke(0);
              rect(325, 0, 150, 800);
              if (runningSpeed == 3) {
                if (stride >= minStride[0] && stride <= maxStride[0]) {
                  fill(255, 0, 0);
                } else {
                  fill(125, 125, 125);
                }
              } else if (runningSpeed == 6) {
                if (stride >= minStride[1] && stride <= maxStride[1]) {
                  fill(255, 0, 0);
                } else {
                  fill(125, 125, 125);
                }
              } else if (runningSpeed == 9) {
                if (stride >= minStride[2] && stride <= maxStride[2]) {
                  fill(255, 0, 0);
                } else {
                  fill(125, 125, 125);
                }
              }
              rect(325, 800 - ((float)stride - 30)*5.7, 150, ((float)stride - 30)*5.7);
              textSize(35);
              fill(0);
              text(String.valueOf((int)stride)+"cm", 360, 800 - ((float)stride - 30)*DRAWING_RATIO + STRIDE_OFFSET);
              isDrawStrideL = true;
              isDrawStrideR = false;
            }
          }
          fill(255);
        } else {
          // このセンサの１つ前のセンサを探して矢印描画
          if (isDrawLeft2) {
          } else {
            for (int i = 0; i < 5; i++) {
              if (pressOrderLeft[i] == orderLeft - 1) {
                putArrow(true, i, 2);
                isDrawLeft2 = true;
              }
            }
          }
        }
        orderLeft++;
      }
      if (sensorValueLeft2 <= peak1_2) {
        peak1_2 = sensorValueLeft2;
        peakTime1_2 = (System.nanoTime() - startTime) - sensorReactedTimeLeft[2];
      } else if (sensorValueLeft2 > peak1_2) {
      }
    } else {
      isLandingPoint1_2 = false;
    }
    if (isLandingPoint1_2) {
      noFill();
      stroke(255, 0, 0);
      strokeWeight(0.5);
      ellipse(230, 330, 65, 65);
    } else {
      noFill();
      stroke(0.5);
      ellipse(230, 330, 65, 65);
    }
    // 小指下
    if (sensorValueLeft3 <= 1010) {
      isLandingPoint1_3 = true;
      if (pressOrderLeft[3] == 0) {
        sensorReactedTimeLeft[3] = System.nanoTime() - startTime;
        pressOrderLeft[3] = orderLeft;
        orderTimeL.put(pressOrderLeft[3], sensorReactedTimeLeft[3]);
        // 最初に接地しているか判定
        if (orderLeft == 1) {
          image(left_foot, 25, 150, 250, 600);
          //noFill();
          strokeWeight(1);
          drawSensorPos(true, SENSOR_ORDER1, SENSOR_ORDER2, SENSOR_ORDER3, SENSOR_ORDER4, SENSOR_ORDER5);
          groundTimeLeft = sensorReactedTimeLeft[3];
          println("GroundTimeLeft="+groundTimeLeft);
          diffTime = groundTimeLeft - groundTimeRight; // 左足を離してから右足が着くまでの時間
          println("DiffTimeL = "+nf((float)diffTime/1000000000, 3, 3));
          stride = diffTime/1000000000*runningSpeed*1000*100/3600;
          println("Stride="+nf((float)stride, 3, 3));
          fill(100, 100, 255);
          if (stride < 30) {
            rect(325, 800, 150, 0);
          } else {
            if (!isDrawStrideL) {
              fill(255, 255, 255);
              stroke(0);
              rect(325, 0, 150, 800);
              if (runningSpeed == 3) {
                if (stride >= minStride[0] && stride <= maxStride[0]) {
                  fill(255, 0, 0);
                } else {
                  fill(125, 125, 125);
                }
              } else if (runningSpeed == 6) {
                if (stride >= minStride[1] && stride <= maxStride[1]) {
                  fill(255, 0, 0);
                } else {
                  fill(125, 125, 125);
                }
              } else if (runningSpeed == 9) {
                if (stride >= minStride[2] && stride <= maxStride[2]) {
                  fill(255, 0, 0);
                } else {
                  fill(125, 125, 125);
                }
              }
              rect(325, 800 - ((float)stride - 30)*5.7, 150, ((float)stride - 30)*5.7);
              textSize(35);
              fill(0);
              text(String.valueOf((int)stride)+"cm", 360, 800 - ((float)stride - 30)*DRAWING_RATIO + STRIDE_OFFSET);
              isDrawStrideL = true;
              isDrawStrideR = false;
            }
          }
          fill(255);
        } else {
          // このセンサの１つ前のセンサを探して矢印描画
          if (isDrawLeft3) {
          } else {
            for (int i = 0; i < 5; i++) {
              if (pressOrderLeft[i] == orderLeft - 1) {
                putArrow(true, i, 3);
                isDrawLeft3 = true;
              }
            }
          }
        }
        orderLeft++;
      }
      if (sensorValueLeft3 <= peak1_3) {
        peak1_3 = sensorValueLeft3;
        peakTime1_3 = (System.nanoTime() - startTime) - sensorReactedTimeLeft[3];
      } else if (sensorValueLeft3 > peak1_3) {
      }
    } else {
      isLandingPoint1_3 = false;
    }
    if (isLandingPoint1_3) {
      noFill();
      stroke(255, 0, 0);
      strokeWeight(0.5);
      ellipse(70, 380, 65, 65);
    } else {
      noFill();
      stroke(0.5);
      ellipse(70, 380, 65, 65);
    }
    // かかと
    if (sensorValueLeft4 <= 1010) {
      isLandingPoint1_4 = true;
      // 接地した順序を格納
      if (pressOrderLeft[4] == 0) {
        sensorReactedTimeLeft[4] = System.nanoTime() - startTime;
        pressOrderLeft[4] = orderLeft;
        orderTimeL.put(pressOrderLeft[4], sensorReactedTimeLeft[4]);
        // 最初に接地しているか判定
        if (orderLeft == 1) {
          image(left_foot, 25, 150, 250, 600);
          //noFill();
          strokeWeight(1);
          drawSensorPos(true, SENSOR_ORDER1, SENSOR_ORDER2, SENSOR_ORDER3, SENSOR_ORDER4, SENSOR_ORDER5);
          groundTimeLeft = sensorReactedTimeLeft[4];
          println("GroundTimeLeft="+groundTimeLeft);
          diffTime = groundTimeLeft - groundTimeRight; // 左足を離してから右足が着くまでの時間
          println("DiffTimeL = "+nf((float)diffTime/1000000000, 3, 3));
          stride = diffTime/1000000000*runningSpeed*1000*100/3600;
          println("Stride="+nf((float)stride, 3, 3));
          fill(100, 100, 255);
          if (stride < 30) {
            rect(325, 800, 150, 0);
          } else {
            if (!isDrawStrideL) {
              fill(255, 255, 255);
              stroke(0);
              rect(325, 0, 150, 800);
              if (runningSpeed == 3) {
                if (stride >= minStride[0] && stride <= maxStride[0]) {
                  fill(255, 0, 0);
                } else {
                  fill(125, 125, 125);
                }
              } else if (runningSpeed == 6) {
                if (stride >= minStride[1] && stride <= maxStride[1]) {
                  fill(255, 0, 0);
                } else {
                  fill(125, 125, 125);
                }
              } else if (runningSpeed == 9) {
                if (stride >= minStride[2] && stride <= maxStride[2]) {
                  fill(255, 0, 0);
                } else {
                  fill(125, 125, 125);
                }
              }
              rect(325, 800 - ((float)stride - 30)*5.7, 150, ((float)stride - 30)*5.7);
              textSize(35);
              fill(0);
              text(String.valueOf((int)stride)+"cm", 360, 800 - ((float)stride - 30)*DRAWING_RATIO + STRIDE_OFFSET);
              isDrawStrideL = true;
              isDrawStrideR = false;
            }
          }
          fill(255);
        } else {
          // このセンサの１つ前のセンサを探して矢印描画
          if (isDrawLeft4) {
          } else {
            for (int i = 0; i < 5; i++) {
              if (pressOrderLeft[i] == orderLeft - 1) {
                putArrow(true, i, 4);
                isDrawLeft4 = true;
              }
            }
          }
        }
        orderLeft++;
      }
      if (sensorValueLeft4 <= peak1_4) {
        peak1_4 = sensorValueLeft4;
        peakTime1_4 = (System.nanoTime() - startTime) - sensorReactedTimeLeft[4];
      }
    } else {
      isLandingPoint1_4 = false;
    }
    if (isLandingPoint1_4) {
      noFill();
      stroke(255, 0, 0);
      strokeWeight(0.5);
      ellipse(144, 674, 75, 75);
    } else {
      noFill();
      stroke(0.5);
      ellipse(144, 674, 75, 75);
    }
    // 全てのセンサで離地判定したら
    if (!isLandingPoint1_0 && !isLandingPoint1_1 && !isLandingPoint1_2 && !isLandingPoint1_3 && !isLandingPoint1_4) {
      // そもそも踏まれてない時
      if (pressOrderLeft[0] == 0 && pressOrderLeft[1] == 0 && pressOrderLeft[2] == 0 && pressOrderLeft[3] == 0 && pressOrderLeft[4] == 0) {
      } else {
        isDrawLeft0 = false;
        isDrawLeft1 = false;
        isDrawLeft2 = false;
        isDrawLeft3 = false;
        isDrawLeft4 = false;
        fill(255);
        // 一箇所でもセンサが反応していた場合
        if (landingTimeLeft != 0) {
        } else {
          landingTimeLeft = System.nanoTime() - startTime; // スタートしてから離地までの時間
        }

        if ((orderTimeL.get(2) - orderTimeL.get(1))/1000000000 < 0) {
          left1_2 = 0;
        } else {
          left1_2 = (orderTimeL.get(2) - orderTimeL.get(1))/1000000000;
        }
        if ((orderTimeL.get(3) - orderTimeL.get(2))/1000000000 < 0) {
          left2_3 = 0;
        } else {
          left2_3 = (orderTimeL.get(3) - orderTimeL.get(2))/1000000000;
        }
        if ((orderTimeL.get(4) - orderTimeL.get(3))/1000000000 < 0) {
          left3_4 = 0;
        } else {
          left3_4 = (orderTimeL.get(4) - orderTimeL.get(3))/1000000000;
        }
        if ((orderTimeL.get(5) - orderTimeL.get(4))/1000000000 < 0) {
          left4_5 = 0;
        } else {
          left4_5 = (orderTimeL.get(5) - orderTimeL.get(4))/1000000000;
        }
        // 計算したものをファイルに保存
        outputPressOrder.println((System.nanoTime() - startTime)/1000000000+","+diffTime/1000000000*runningSpeed*1000*100/3600+","+(landingTimeLeft - groundTimeLeft)/1000000000+","+pressOrderLeft[0]+","+pressOrderLeft[1]+","+pressOrderLeft[2]+","+pressOrderLeft[3]+","+pressOrderLeft[4]+","+peak1_0+","+peakTime1_0/1000000000+","+peak1_1+","+peakTime1_1/1000000000+","+peak1_2+","+peakTime1_2/1000000000+","+peak1_3+","+peakTime1_3/1000000000+","+peak1_4+","+peakTime1_4/1000000000+","+left1_2+","+left2_3+","+left3_4+","+left4_5+","+","+","+","+","+","+","+","+","+","+","+","+","+","+","+","+","+","+","+","+","+","+","+",");
      }
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
      //groundTimeLeft = 0;
      landingTimeLeft = 0;
      firstTouchLeft = false;
    }
  }

  // Right
  // 親指
  if (myPort2.available()>0) {
    if (sensorValueRight0 <= 990) {
      isLandingPoint2_0 = true;
      // 接地した順序を格納
      if (pressOrderRight[0] == 0) {
        sensorReactedTimeRight[0] = System.nanoTime() - startTime;
        pressOrderRight[0] = orderRight;
        orderTimeR.put(pressOrderRight[0], sensorReactedTimeRight[0]);
        // 最初に接地しているか判定
        if (orderRight == 1) {
          image(right_foot, 520, 150, 250, 600);
          //noFill();
          strokeWeight(1);
          drawSensorPos(false, SENSOR_ORDER1, SENSOR_ORDER2, SENSOR_ORDER3, SENSOR_ORDER4, SENSOR_ORDER5);
          groundTimeRight = sensorReactedTimeRight[0];
          println("GroundTimeRight="+groundTimeRight);
          diffTime = groundTimeRight - groundTimeLeft;
          println("DiffTimeR = "+nf((float)diffTime/1000000000, 3, 3));
          stride = diffTime/1000000000*runningSpeed*1000*100/3600;
          println("Stride="+nf((float)stride, 3, 3));
          fill(100, 100, 255);
          if (stride < 30) {
            rect(325, 800, 150, 0);
          } else {
            if (!isDrawStrideR) {
              fill(255, 255, 255);
              stroke(0);
              rect(325, 0, 150, 800);
              if (runningSpeed == 3) {
                if (stride >= minStride[0] && stride <= maxStride[0]) {
                  fill(255, 0, 0);
                } else {
                  fill(125, 125, 125);
                }
              } else if (runningSpeed == 6) {
                if (stride >= minStride[1] && stride <= maxStride[1]) {
                  fill(255, 0, 0);
                } else {
                  fill(125, 125, 125);
                }
              } else if (runningSpeed == 9) {
                if (stride >= minStride[2] && stride <= maxStride[2]) {
                  fill(255, 0, 0);
                } else {
                  fill(125, 125, 125);
                }
              }
              rect(325, 800 - ((float)stride - 30)*5.7, 150, ((float)stride - 30)*5.7);
              textSize(35);
              fill(0);
              text(String.valueOf((int)stride)+"cm", 360, 800 - ((float)stride - 30)*DRAWING_RATIO + STRIDE_OFFSET);
              isDrawStrideR = true;
              isDrawStrideL = false;
            }
          }
          fill(255);
        } else {
          // このセンサの１つ前のセンサを探して矢印描画
          if (isDrawRight0) {
          } else {
            for (int i = 0; i < 5; i++) {
              if (pressOrderRight[i] == orderRight - 1) {
                putArrow(false, i, 0);
                isDrawRight0 = true;
              }
            }
          }
        }
        orderRight++;
      }
      if (sensorValueRight0 <= peak2_0) {
        peak2_0 = sensorValueRight0;
        peakTime2_0 = (System.nanoTime() - startTime) - sensorReactedTimeRight[0];
      } else if (sensorValueRight0 > peak2_0) {
      }
    } else {
      isLandingPoint2_0 = false;
    }
    if (isLandingPoint2_0) {
      noFill();
      stroke(255, 0, 0);
      strokeWeight(0.5);
      ellipse(565, 210, 65, 65); // 親指
    } else {
      noFill();
      stroke(0.5);
      ellipse(565, 210, 65, 65); // 親指
    }
    // 小指
    if (sensorValueRight1 <= 990) {
      isLandingPoint2_1 = true;
      // このセンサ以外のセンサがまだ接地していない場合
      if (pressOrderRight[1] == 0) {
        sensorReactedTimeRight[1] = System.nanoTime() - startTime;
        pressOrderRight[1] = orderRight;
        orderTimeR.put(pressOrderRight[1], sensorReactedTimeRight[1]);
        // 最初に接地しているか判定
        if (orderRight == 1) {
          image(right_foot, 520, 150, 250, 600);
          //noFill();
          strokeWeight(1);
          drawSensorPos(false, SENSOR_ORDER1, SENSOR_ORDER2, SENSOR_ORDER3, SENSOR_ORDER4, SENSOR_ORDER5);
          groundTimeRight = sensorReactedTimeRight[1];
          println("GroundTimeRight="+groundTimeRight);
          diffTime = groundTimeRight - groundTimeLeft;
          println("DiffTimeR = "+nf((float)diffTime/1000000000, 3, 3));
          stride = diffTime/1000000000*runningSpeed*1000*100/3600;
          println("Stride="+nf((float)stride, 3, 3));
          fill(100, 100, 255);
          if (stride < 30) {
            rect(325, 800, 150, 0);
          } else {
            if (!isDrawStrideR) {
              fill(255, 255, 255);
              stroke(0);
              rect(325, 0, 150, 800);
              if (runningSpeed == 3) {
                if (stride >= minStride[0] && stride <= maxStride[0]) {
                  fill(255, 0, 0);
                } else {
                  fill(125, 125, 125);
                }
              } else if (runningSpeed == 6) {
                if (stride >= minStride[1] && stride <= maxStride[1]) {
                  fill(255, 0, 0);
                } else {
                  fill(125, 125, 125);
                }
              } else if (runningSpeed == 9) {
                if (stride >= minStride[2] && stride <= maxStride[2]) {
                  fill(255, 0, 0);
                } else {
                  fill(125, 125, 125);
                }
              }
              rect(325, 800 - ((float)stride - 30)*5.7, 150, ((float)stride - 30)*5.7);
              textSize(35);
              fill(0);
              text(String.valueOf((int)stride)+"cm", 360, 800 - ((float)stride - 30)*DRAWING_RATIO + STRIDE_OFFSET);
              isDrawStrideR = true;
              isDrawStrideL = false;
            }
          }
          fill(255);
        } else {
          // このセンサの１つ前のセンサを探して矢印描画
          if (isDrawRight1) {
          } else {
            for (int i = 0; i < 5; i++) {
              if (pressOrderRight[i] == orderRight - 1) {
                putArrow(false, i, 1);
                isDrawRight1 = true;
              }
            }
          }
        }
        orderRight++;
      }
      if (sensorValueRight1 <= peak2_1) {
        peak2_1 = sensorValueRight1;
        peakTime2_1 = (System.nanoTime() - startTime) - sensorReactedTimeRight[1];
      } else if (sensorValueRight1 > peak2_1) {
      }
    } else {
      isLandingPoint2_1 = false;
    }
    if (isLandingPoint2_1) {
      noFill();
      stroke(255, 0, 0);
      strokeWeight(0.5);
      ellipse(740, 280, 65, 65); // 小指
    } else {
      noFill();
      stroke(0.5);
      ellipse(740, 280, 65, 65); // 小指
    }
    // 親指下
    if (sensorValueRight2 <= 990) {
      isLandingPoint2_2 = true;
      // このセンサ以外のセンサがまだ接地していない場合
      if (pressOrderRight[2] == 0) {
        sensorReactedTimeRight[2] = System.nanoTime() - startTime;
        pressOrderRight[2] = orderRight;
        orderTimeR.put(pressOrderRight[2], sensorReactedTimeRight[2]);
        // 最初に接地しているか判定
        if (orderRight == 1) {
          image(right_foot, 520, 150, 250, 600);
          strokeWeight(1);
          drawSensorPos(false, SENSOR_ORDER1, SENSOR_ORDER2, SENSOR_ORDER3, SENSOR_ORDER4, SENSOR_ORDER5);
          groundTimeRight = sensorReactedTimeRight[2];
          println("groundTimeRight="+groundTimeRight);
          diffTime = groundTimeRight - groundTimeLeft;
          println("DiffTimeR = "+nf((float)diffTime/1000000000, 3, 3));
          stride = diffTime/1000000000*runningSpeed*1000*100/3600;
          println("Stride="+nf((float)stride, 3, 3));
          fill(100, 100, 255);
          if (stride < 30) {
            rect(325, 800, 150, 0);
          } else {
            if (!isDrawStrideR) {
              fill(255, 255, 255);
              stroke(0);
              rect(325, 0, 150, 800);
              if (runningSpeed == 3) {
                if (stride >= minStride[0] && stride <= maxStride[0]) {
                  fill(255, 0, 0);
                } else {
                  fill(125, 125, 125);
                }
              } else if (runningSpeed == 6) {
                if (stride >= minStride[1] && stride <= maxStride[1]) {
                  fill(255, 0, 0);
                } else {
                  fill(125, 125, 125);
                }
              } else if (runningSpeed == 9) {
                if (stride >= minStride[2] && stride <= maxStride[2]) {
                  fill(255, 0, 0);
                } else {
                  fill(125, 125, 125);
                }
              }
              rect(325, 800 - ((float)stride - 30)*5.7, 150, ((float)stride - 30)*5.7);
              textSize(35);
              fill(0);
              text(String.valueOf((int)stride)+"cm", 360, 800 - ((float)stride - 30)*DRAWING_RATIO + STRIDE_OFFSET);
              isDrawStrideR = true;
              isDrawStrideL = false;
            }
          }
          fill(255);
        } else {
          // このセンサの１つ前のセンサを探して矢印描画
          if (isDrawRight2) {
          } else {
            for (int i = 0; i < 5; i++) {
              if (pressOrderRight[i] == orderRight - 1) {
                putArrow(false, i, 2);
                isDrawRight2 = true;
                //println("3:"+i+"-"+3);
              }
            }
          }
        }
        orderRight++;
      }
      if (sensorValueRight2 <= peak2_2) {
        peak2_2 = sensorValueRight2;
        peakTime2_2 = (System.nanoTime() - startTime) - sensorReactedTimeRight[2];
      } else if (sensorValueRight2 > peak2_2) {
      }
    } else {
      isLandingPoint2_2 = false;
    }
    if (isLandingPoint2_2) {
      noFill();
      stroke(255, 0, 0);
      strokeWeight(0.5);
      ellipse(565, 330, 65, 65); // 親指下
    } else {
      noFill();
      stroke(0.5);
      ellipse(565, 330, 65, 65); // 親指下
    }
    // 小指下
    if (sensorValueRight3 <= 990) { 
      isLandingPoint2_3 = true;
      // このセンサ以外のセンサがまだ接地していない場合
      if (pressOrderRight[3] == 0) {
        sensorReactedTimeRight[3] = System.nanoTime() - startTime;
        pressOrderRight[3] = orderRight;
        orderTimeR.put(pressOrderRight[3], sensorReactedTimeRight[3]);
        // 最初に接地しているか判定
        if (orderRight == 1) {
          image(right_foot, 520, 150, 250, 600);
          strokeWeight(1);
          drawSensorPos(false, SENSOR_ORDER1, SENSOR_ORDER2, SENSOR_ORDER3, SENSOR_ORDER4, SENSOR_ORDER5);
          groundTimeRight = sensorReactedTimeRight[3];
          println("groundTimeRight="+groundTimeRight);
          diffTime = groundTimeRight - groundTimeLeft;
          println("DiffTimeR = "+nf((float)diffTime/1000000000, 3, 3));
          stride = diffTime/1000000000*runningSpeed*1000*100/3600;
          println("Stride="+nf((float)stride, 3, 3));
          fill(100, 100, 255);
          if (stride < 30) {
            rect(325, 800, 150, 0);
          } else {
            if (!isDrawStrideR) {
              fill(255, 255, 255);
              stroke(0);
              rect(325, 0, 150, 800);
              if (runningSpeed == 3) {
                if (stride >= minStride[0] && stride <= maxStride[0]) {
                  fill(255, 0, 0);
                } else {
                  fill(125, 125, 125);
                }
              } else if (runningSpeed == 6) {
                if (stride >= minStride[1] && stride <= maxStride[1]) {
                  fill(255, 0, 0);
                } else {
                  fill(125, 125, 125);
                }
              } else if (runningSpeed == 9) {
                if (stride >= minStride[2] && stride <= maxStride[2]) {
                  fill(255, 0, 0);
                } else {
                  fill(125, 125, 125);
                }
              }
              rect(325, 800 - ((float)stride - 30)*5.7, 150, ((float)stride - 30)*5.7);
              textSize(35);
              fill(0);
              text(String.valueOf((int)stride)+"cm", 360, 800 - ((float)stride - 30)*DRAWING_RATIO + STRIDE_OFFSET);
              isDrawStrideR = true;
              isDrawStrideL = false;
            }
          }
          fill(255);
        } else {
          // このセンサの１つ前のセンサを探して矢印描画
          if (isDrawRight3) {
          } else {
            for (int i = 0; i < 5; i++) {
              if (pressOrderRight[i] == orderRight - 1) {
                putArrow(false, i, 3);
                isDrawRight3 = true;
                //println("3:"+i+"-"+3);
              }
            }
          }
        }
        orderRight++;
      }
      if (sensorValueRight3 <= peak2_3) {
        peak2_3 = sensorValueRight3;
        peakTime2_3 = (System.nanoTime() - startTime) - sensorReactedTimeRight[3];
      } else if (sensorValueRight3 > peak2_3) {
      }
    } else {
      isLandingPoint2_3 = false;
    }
    if (isLandingPoint2_3) {
      noFill();
      stroke(255, 0, 0);
      strokeWeight(0.5);
      ellipse(725, 370, 65, 65); // 小指下
    } else {
      noFill();
      stroke(0.5);
      ellipse(725, 370, 65, 65); // 小指下
    }
    // かかと
    if (sensorValueRight4 <= 990) {
      isLandingPoint2_4 = true;
      // このセンサ以外のセンサがまだ接地していない場合
      if (pressOrderRight[4] == 0) {
        sensorReactedTimeRight[4] = System.nanoTime() - startTime;
        pressOrderRight[4] = orderRight;
        orderTimeR.put(pressOrderRight[4], sensorReactedTimeRight[4]);
        // 最初に接地しているか判定
        if (orderRight == 1) {
          image(right_foot, 520, 150, 250, 600);
          strokeWeight(1);
          drawSensorPos(false, SENSOR_ORDER1, SENSOR_ORDER2, SENSOR_ORDER3, SENSOR_ORDER4, SENSOR_ORDER5);
          groundTimeRight = sensorReactedTimeRight[4];
          println("groundTimeRight="+groundTimeRight);
          diffTime = groundTimeRight - groundTimeLeft;
          println("DiffTimeR = "+nf((float)diffTime/1000000000, 3, 3));
          stride = diffTime/1000000000*runningSpeed*1000*100/3600;
          println("Stride="+nf((float)stride, 3, 3));
          fill(100, 100, 255);
          if (stride < 30) {
            rect(325, 800, 150, 0);
          } else {
            if (!isDrawStrideR) {
              fill(255, 255, 255);
              stroke(0);
              rect(325, 0, 150, 800);
              if (runningSpeed == 3) {
                if (stride >= minStride[0] && stride <= maxStride[0]) {
                  fill(255, 0, 0);
                } else {
                  fill(125, 125, 125);
                }
              } else if (runningSpeed == 6) {
                if (stride >= minStride[1] && stride <= maxStride[1]) {
                  fill(255, 0, 0);
                } else {
                  fill(125, 125, 125);
                }
              } else if (runningSpeed == 9) {
                if (stride >= minStride[2] && stride <= maxStride[2]) {
                  fill(255, 0, 0);
                } else {
                  fill(125, 125, 125);
                }
              }
              rect(325, 800 - ((float)stride - 30)*5.7, 150, ((float)stride - 30)*5.7);
              textSize(35);
              fill(0);
              text(String.valueOf((int)stride)+"cm", 360, 800 - ((float)stride - 30)*DRAWING_RATIO + STRIDE_OFFSET);
              isDrawStrideR = true;
              isDrawStrideL = false;
            }
          }
          fill(255);
        } else {
          // このセンサの１つ前のセンサを探して矢印描画
          if (isDrawRight4) {
          } else {
            for (int i = 0; i < 5; i++) {
              if (pressOrderRight[i] == orderRight - 1) {
                putArrow(false, i, 4);
                isDrawRight4 = true;
                //println("3:"+i+"-"+3);
              }
            }
          }
        }
        orderRight++;
      }
      if (sensorValueRight4 <= peak2_4) {
        peak2_4 = sensorValueRight4;
        peakTime2_4 = (System.nanoTime() - startTime) - sensorReactedTimeRight[4];
      } else if (sensorValueRight4 > peak2_4) {
      }
    } else {
      isLandingPoint2_4 = false;
    }
    if (isLandingPoint2_4) {
      noFill();
      stroke(255, 0, 0);
      strokeWeight(0.5);
      ellipse(650, 675, 75, 75); // かかと
    } else {
      noFill();
      stroke(0.5);
      ellipse(650, 675, 75, 75); // かかと
    }
    // 右足が全て地面から離れたら
    if (!isLandingPoint2_0 && !isLandingPoint2_1 && !isLandingPoint2_2 && !isLandingPoint2_3 && !isLandingPoint2_4) {
      if (pressOrderRight[0] == 0 && pressOrderRight[1] == 0 && pressOrderRight[2] == 0 && pressOrderRight[3] == 0 && pressOrderRight[4] == 0) {
      } else {
        fill(255);
        isDrawRight0 = false;
        isDrawRight1 = false;
        isDrawRight2 = false;
        isDrawRight3 = false;
        isDrawRight4 = false;
        if (landingTimeRight != 0) {
        } else {
          landingTimeRight = System.nanoTime() - startTime; // 離地した時間
        }
        if ((orderTimeR.get(2) - orderTimeR.get(1))/1000000000 < 0) {
          right1_2 = 0;
        } else {
          right1_2 = (orderTimeR.get(2) - orderTimeR.get(1))/1000000000;
        }
        if ((orderTimeR.get(3) - orderTimeR.get(2))/1000000000 < 0) {
          right2_3 = 0;
        } else {
          right2_3 = (orderTimeR.get(3) - orderTimeR.get(2))/1000000000;
        }
        if ((orderTimeR.get(4) - orderTimeR.get(3))/1000000000 < 0) {
          right3_4 = 0;
        } else {
          right3_4 = (orderTimeR.get(4) - orderTimeR.get(3))/1000000000;
        }
        if ((orderTimeR.get(5) - orderTimeR.get(4))/1000000000 < 0) {
          right4_5 = 0;
        } else {
          right4_5 = (orderTimeR.get(5) - orderTimeR.get(4))/1000000000;
        }
        outputPressOrder.println(","+","+","+","+","+","+","+","+","+","+","+","+","+","+","+","+","+","+","+","+","+","+","+(System.nanoTime() - startTime)/1000000000+","+diffTime/1000000000*runningSpeed*1000*100/3600+","+(landingTimeRight - groundTimeRight)/1000000000+","+pressOrderRight[0]+","+pressOrderRight[1]+","+pressOrderRight[2]+","+pressOrderRight[3]+","+pressOrderRight[4]+","+peak2_0+","+peakTime2_0/1000000000+","+peak2_1+","+peakTime2_1/1000000000+","+peak2_2+","+peakTime2_2/1000000000+","+peak2_3+","+peakTime2_3/1000000000+","+peak2_4+","+peakTime2_4/1000000000+","+right1_2+","+right2_3+","+right3_4+","+right4_5);
        peak2_0 = 2000;
        peak2_1 = 2000;
        peak2_2 = 2000;
        peak2_3 = 2000;
        peak2_4 = 2000;
        for (int i = 0; i < pressOrderRight.length; i++) {
          pressOrderRight[i] = 0;
        }
        orderRight = 1;
        //groundTimeRight = 0;
        landingTimeRight = 0;
      }
    }
  }
}

void serialEvent(Serial port) {
  if (port == myPort1) {
    //println("available1:"+port.available());
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
  } else {
    //println("Not found port1");
  }
  if (port == myPort2) {
    //println(", available2:"+port.available());
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
  } else {
    //println("Not found port2");
  }
  if ((System.nanoTime() - startTime)/1000000000 >= 0 && (System.nanoTime() - startTime)/1000000000 < 550) {
    if (sensorValueLeft0 > 0 && sensorValueLeft1 > 0 && sensorValueLeft2 > 0 && sensorValueLeft3 > 0 && sensorValueLeft4 > 0 && sensorValueRight0 > 0 && sensorValueRight1 > 0 && sensorValueRight2 > 0 && sensorValueRight3 > 0 && sensorValueRight4 > 0) {
      output.println((System.nanoTime() - startTime)/1000000000+","+sensorValueLeft0+","+","+sensorValueLeft1+","+","+sensorValueLeft2+","+","+sensorValueLeft3+","+","+sensorValueLeft4+","+","+(System.nanoTime() - startTime)/1000000000+","+sensorValueRight0+","+","+sensorValueRight1+","+","+sensorValueRight2+","+","+sensorValueRight3+","+","+sensorValueRight4);
    }
  } else if ((System.nanoTime() - startTime)/1000000000 >= 550 && (System.nanoTime() - startTime)/1000000000 < 1100) {
    output.flush();
    output.close();
    if (sensorValueLeft0 > 0 && sensorValueLeft1 > 0 && sensorValueLeft2 > 0 && sensorValueLeft3 > 0 && sensorValueLeft4 > 0 && sensorValueRight0 > 0 && sensorValueRight1 > 0 && sensorValueRight2 > 0 && sensorValueRight3 > 0 && sensorValueRight4 > 0) {
      output2.println((System.nanoTime() - startTime)/1000000000+","+sensorValueLeft0+","+","+sensorValueLeft1+","+","+sensorValueLeft2+","+","+sensorValueLeft3+","+","+sensorValueLeft4+","+","+(System.nanoTime() - startTime)/1000000000+","+sensorValueRight0+","+","+sensorValueRight1+","+","+sensorValueRight2+","+","+sensorValueRight3+","+","+sensorValueRight4);
    }
  }
  println("time1="+nf((float)(System.nanoTime() - startTime)/1000000000, 3, 0)+",inByte1_0="+sensorValueLeft0+", inByte1_1="+sensorValueLeft1+", inByte1_2="+sensorValueLeft2+", inByte1_3="+sensorValueLeft3+", inByte1_4="+sensorValueLeft4);
  println("time2="+nf((float)(System.nanoTime() - startTime)/1000000000, 3, 0)+",inByte2_0="+sensorValueRight0+", inByte2_1="+sensorValueRight1+", inByte2_2="+sensorValueRight2+", inByte2_3="+sensorValueRight3+", inByte2_4="+sensorValueRight4);
}

void drawStride(int min, int max) {
  textSize(25);
  fill(0);
  text(String.valueOf(min), 285, 780);
  text(String.valueOf(min+(max - min)/4), 285, 590);
  text(String.valueOf(min+(max - min)/4*2), 280, 400);
  text(String.valueOf(min+(max - min)/4*3), 280, 200);
  text(String.valueOf(max), 280, 30);
  text(String.valueOf(min), 475, 780);
  text(String.valueOf(min+(max - min)/4), 475, 590);
  text(String.valueOf(min+(max - min)/4*2), 470, 400);
  text(String.valueOf(min+(max - min)/4*3), 470, 200);
  text(String.valueOf(max), 470, 30);
  textSize(15);
  text("cm", 295, 790);
  text("cm", 285, 600);
  text("cm", 295, 410);
  text("cm", 285, 210);
  text("cm", 295, 40);
  text("cm", 485, 790);
  text("cm", 485, 600);
  text("cm", 485, 410);
  text("cm", 485, 210);
  text("cm", 480, 40);
}

void drawSensorPos(boolean isLeft, int finger1, int finger2, int finger3, int finger4, int finger5) { // 親指、小指、親指下、小指下、かかとの順番を入れる
  if (isLeft) {
    // 左足サークル
    noFill();
    ellipse(144, 674, 75, 75); // かかと
    ellipse(230, 210, 65, 65); // 親指
    ellipse(55, 280, 65, 65); //小指
    ellipse(230, 330, 65, 65); // 親指下
    ellipse(70, 380, 65, 65); // 小指下
    // 左足テキスト
    textSize(50);
    fill(0);
    text(String.valueOf(finger1), 215, 225); // 親指
    text(String.valueOf(finger2), 40, 300); // 小指
    text(String.valueOf(finger3), 215, 350); // 親指下
    text(String.valueOf(finger4), 55, 400); // 小指下
    text(String.valueOf(finger5), 128, 690); // かかと
  } else {
    // 右足サークル
    noFill();
    ellipse(650, 675, 75, 75); // かかと
    ellipse(565, 210, 65, 65); // 親指
    ellipse(740, 280, 65, 65); // 小指
    ellipse(565, 330, 65, 65); // 親指下
    ellipse(725, 370, 65, 65); // 小指下
    // 右足テキスト
    textSize(50);
    fill(0);
    text(String.valueOf(finger1), 550, 225); // 親指
    text(String.valueOf(finger2), 725, 297); // 小指
    text(String.valueOf(finger3), 550, 350); // 親指下
    text(String.valueOf(finger4), 710, 390); // 小指下
    text(String.valueOf(finger5), 633, 695); // かかと
  }
}

void drawArrow(float x1, float y1, float x2, float y2) {
  float a = dist(x1, y1, x2, y2) / 50;
  stroke(0);
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
      if (end != 0) {
        x2 = 230;
        y2 = 220;
      } else {
        x2 = x1;
        y2 = y1;
      }
    } else if (tip == 1) {
      if (end != 1) {
        x2 = 60;
        y2 = 285;
      } else {
        x2 = x1;
        y2 = y1;
      }
    } else if (tip == 2) {
      if (end != 2) {
        x2 = 230;
        y2 = 335;
      } else {
        x2 = x1;
        y2 = y1;
      }
    } else if (tip == 3) {
      if (end != 3) {
        x2 = 70;
        y2 = 385;
      } else {
        x2 = x1;
        y2 = y1;
      }
    } else if (tip == 4) {
      if (end != 4) {
        x2 = 145;
        y2 = 670;
      } else {
        x2 = x1;
        y2 = y1;
      }
    }
  } else {
    if (end == 0) {
      x1 = 565;
      y1 = 220;
    } else if (end == 1) {
      x1 = 730;
      y1 = 285;
    } else if (end == 2) {
      x1 = 565;
      y1 = 335;
    } else if (end == 3) {
      x1 = 720;
      y1 = 385;
    } else if (end == 4) {
      x1 = 650;
      y1 = 675;
    }

    if (tip == 0) {
      if (end != 0) {
        x2 = 565;
        y2 = 220;
      } else {
        x2 = x1;
        y2 = y1;
      }
    } else if (tip == 1) {
      if (end != 1) {
        x2 = 730;
        y2 = 285;
      } else {
        x2 = x1;
        y2 = y1;
      }
    } else if (tip == 2) {
      if (end != 2) {
        x2 = 565;
        y2 = 335;
      } else {
        x2 = x1;
        y2 = y1;
      }
    } else if (tip == 3) {
      if (end != 3) {
        x2 = 720;
        y2 = 385;
      } else {
        x2 = x1;
        y2 = y1;
      }
    } else if (tip == 4) {
      if (end != 4) {
        x2 = 650;
        y2 = 675;
      } else {
        x2 = x1;
        y2 = y1;
      }
    }
  }
  drawArrow(x1, y1, x2, y2);
}