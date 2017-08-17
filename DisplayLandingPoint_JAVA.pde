import processing.serial.*;

PImage foot_img, landingPoint_img, backButton, landingPointWhite, good_imgLeft, bad_imgLeft, good_imgRight, bad_imgRight;
PImage whiteBoardGood_imgLeft, whiteBoardBad_imgLeft, whiteBoardGood_imgRight, whiteBoardBad_imgRight, one_img, two_img, three_img;
PImage left_red, left_blue, right_red, right_blue;
Serial myPort1, myPort2;
int analog1_0_high, analog1_0_low, analog1_1_high, analog1_1_low, analog1_2_high, analog1_2_low, analog1_3_high, analog1_3_low, analog1_4_high, analog1_4_low;// get from SerialPort
int analog2_0_high, analog2_0_low, analog2_1_high, analog2_1_low, analog2_2_high, analog2_2_low, analog2_3_high, analog2_3_low, analog2_4_high, analog2_4_low;
int portIndex = 2;
int inByte1_0, inByte1_1, inByte1_2, inByte1_3, inByte1_4, inByte1_5, inByte2_0, inByte2_1, inByte2_2, inByte2_3, inByte2_4, inByte2_5;
int winWidth = 800, winHeight = 800;
PrintWriter output, outputPeak;
boolean isLandingPoint1_0 = false, isLandingPoint1_1 = false, isLandingPoint1_2 = false, isLandingPoint1_3 = false, isLandingPoint1_4 = false;
boolean isLandingPoint2_0 = false, isLandingPoint2_1 = false, isLandingPoint2_2 = false, isLandingPoint2_3 = false, isLandingPoint2_4 = false;
long sTime, lTime;
long time1 = 0, time2 = 0, time = 0;
int storage1_0 = 0, storage1_1 = 0, storage1_2 = 0, storage1_3 = 0, storage1_4 = 0, storage2_0 = 0, storage2_1 = 0, storage2_2 = 0, storage2_3 = 0, storage2_4 = 0;
int peak1_0 = 2000, peak1_1 = 2000, peak1_2 = 2000, peak1_3 = 2000, peak1_4 = 2000, peak2_0 = 2000, peak2_1 = 2000, peak2_2 = 2000, peak2_3 = 2000, peak2_4 = 2000;
int pressOrderRight[] = {10, 10, 10, 10, 10};
int pressOrderLeft[] = {10, 10, 10, 10, 10};
long groundTimeLeft = 0, landingTimeLeft = 0, groundTimeRight = 0, landingTimeRight = 0, diffLeft = 0, diffRight = 0;

void setup() {
  size(800, 800);
  //output = createWriter(year()+"-"+month()+"-"+day()+"-"+hour()+"-"+minute()+"-"+second()+".csv");
  output = createWriter("Test.csv");
  //outputPeak = createWriter("p"+year()+"-"+month()+"-"+day()+"-"+hour()+"-"+minute()+"-"+second()+".csv");
  //outputPeak = createWriter("PeakTest.csv");
  output.println("time1,inByte1_0,inByte1_1,inByte1_2,inByte1_3,inByte1_4,time2,inByte2_0,inByte2_1,inByte2_2,inByte2_3,inByte2_4");
  //outputPeak.println("inByte1_0,inByte1_1,inByte1_2,inByte1_3,inByte1_4,inByte2_0,inByte2_1,inByte2_2,inByte2_3,inByte2_4");
  myPort1 = new Serial(this, "/dev/tty.HC-06-DevB", 9600);
  myPort2 = new Serial(this, "/dev/tty.HC-06-DevB-2", 9600);
  good_imgLeft = loadImage("good.png");
  bad_imgLeft = loadImage("bad.png");
  good_imgRight = loadImage("good.png");
  bad_imgRight = loadImage("bad.png");
  whiteBoardGood_imgLeft = loadImage("white_board_good.png");
  whiteBoardBad_imgLeft = loadImage("white_board_good.png");
  whiteBoardGood_imgRight = loadImage("white_board_good.png");
  whiteBoardBad_imgRight = loadImage("white_board_good.png");
  foot_img = loadImage("foot_sole900.jpeg");
  landingPoint_img = loadImage("Landing_Point.png");
  landingPointWhite = loadImage("white.png");
  one_img = loadImage("one.png");
  two_img = loadImage("two.png");
  three_img = loadImage("three.png");
  left_red = loadImage("leftred.png");
  left_blue = loadImage("leftblue.png");
  right_red = loadImage("rightred.png");
  right_blue = loadImage("rightblue.png");
  image(foot_img, 0, 0, 800, 800);
  
  //sTime = millis();
  //image(one_img, 270, 400, 100, 100);
  //image(two_img, 270, 500, 100, 100);
  //image(three_img, 270, 600, 100, 100);
  //image(good_imgLeft, 270, 440, 100, 100);
  //image(bad_imgLeft, 270, 540, 100, 100);
  //image(good_imgRight, 430, 440, 100, 100);
  //image(bad_imgRight, 430, 540, 100, 100);
  //image(whiteBoardGood_imgLeft, 270, 440, 100, 100);
  //image(whiteBoardBad_imgLeft, 270, 540, 100, 100);
  //image(whiteBoardGood_imgRight, 430, 440, 100, 100);
  //image(whiteBoardBad_imgRight, 430, 540, 100, 100);
}

void draw() {
  //lTime = millis() - sTime;
  //if(myPort1.available()>0){
  //  //image(whiteBoardGood_imgLeft, 270, 440, 100, 100);
  //  //image(whiteBoardBad_imgLeft, 270, 540, 100, 100);
  //  image(foot_img, 0, 0, 800, 800);
  //  if (inByte1_0 <= 900) {
  //    if (isLandingPoint1_3) {
  //      //image(good_imgLeft, 270, 440, 100, 100);
  //    } else {
  //      isLandingPoint1_0 = true;
  //      //image(bad_imgLeft, 270, 540, 100, 100);
  //    }
  //    //image(landingPoint_img, 190, 0, 180, 180);
  //    image(left_red, 0, 0);
  //  } else {
  //    isLandingPoint1_0 = false;
  //    //image(landingPointWhite, winWidth*28/100, 65, 75, 75);
  //  }
    
  //  if (inByte1_1 <= 1000) {
  //    if (isLandingPoint1_3) {
  //      //image(good_imgLeft, 270, 440, 100, 100);
  //    } else {
  //      isLandingPoint1_1 = true;
  //      //image(bad_imgLeft, 270, 540, 100, 100);
  //    }
  //    //image(landingPoint_img, 9, winHeight*4/25, 100, 100);
  //    image(left_red, 0, 0);
  //  } else {
  //    isLandingPoint1_1 = false;
  //    //image(landingPointWhite, 28, 166, 40, 40);
  //  }
    
  //  if (inByte1_2 <= 900) {
  //    if (isLandingPoint1_3) {
  //      //image(good_imgLeft, 270, 440, 100, 100);
  //    } else {
  //      isLandingPoint1_2 = true;
  //      //image(bad_imgLeft, 270, 540, 100, 100);
  //    }
  //    //image(landingPoint_img, 170, 120, 210, 210);
  //    image(left_red, 0, 0);
  //  } else {
  //    isLandingPoint1_2 = false;
  //    //image(landingPointWhite, 200, winHeight*6/25, 100, 100);
  //  }
    
  //  if (inByte1_3 <= 1000) {
  //    if (isLandingPoint1_0 || isLandingPoint1_1 || isLandingPoint1_2 || isLandingPoint1_4) {
  //      //image(bad_imgLeft, 270, 540, 100, 100);
  //    } else {
  //      isLandingPoint1_3 = true;
  //      //image(good_imgLeft, 270, 440, 100, 100);
  //    }
  //    //image(landingPoint_img, 5, winHeight*6/25, 170, 170);
  //    image(left_red, 0, 0);
  //  } else {
  //    isLandingPoint1_3 = false;
  //    //image(landingPointWhite, 33, winHeight*8/25, 75, 75);
  //  }
    
  //  if (inByte1_4 <= 900) {
  //    if (isLandingPoint1_3) {
  //      //image(good_imgLeft, 270, 440, 100, 100);
  //    } else {
  //      isLandingPoint1_4 = true;
  //      //image(bad_imgLeft, 270, 540, 100, 100);
  //    }
  //    //image(landingPoint_img, 70, winHeight*18/25, 220, 220);
  //    image(left_blue, 0, 0);
  //  } else {
  //    isLandingPoint1_4 = false;
  //    //image(landingPointWhite, 111, 660, 90, 90);
  //  }
  //}
  //if(myPort2.available()>0){
  //  //image(whiteBoardGood_imgRight, 430, 440, 100, 100);
  //  //image(whiteBoardBad_imgRight, 430, 540, 100, 100);
    
  //  if (inByte2_0 <= 1000) {
  //    if (isLandingPoint2_3) {
  //      //image(good_imgRight, 430, 440, 100, 100);
  //    } else {
  //      isLandingPoint2_0 = true;
  //      //image(bad_imgRight, 430, 540, 100, 100);
  //    }
  //    //image(landingPoint_img, 468, 0, 180, 180);
  //    image(right_red, 0, 0);
  //  } else {
  //    isLandingPoint2_0 = false;
  //    //image(landingPointWhite, 500, 65, 75, 75);
  //  }
    
  //  if (inByte2_1 <= 1000) {
  //    if (isLandingPoint2_3) {
  //      //image(good_imgRight, 430, 440, 100, 100);
  //    } else {
  //      isLandingPoint2_1 = true;
  //      //image(bad_imgRight, 430, 540, 100, 100);
  //    }
  //    //image(landingPoint_img, 715, winHeight*4/25, 100, 100);
  //    image(right_red, 0, 0);
  //  } else {
  //    isLandingPoint2_1 = false;
  //    //image(landingPointWhite, 734, 166, 40, 40);
  //  }
    
  //  if (inByte2_2 <= 1000) {// 900
  //    if (isLandingPoint2_3) {
  //      //image(good_imgRight, 430, 440, 100, 100);
  //    } else {
  //      isLandingPoint2_2 = true;
  //      //image(bad_imgRight, 430, 540, 100, 100);
  //    }
  //    //image(landingPoint_img, winWidth*29/50, 120, 210, 210);
  //    image(right_red, 0, 0);
  //  } else {
  //    isLandingPoint2_2 = false;
  //    //image(landingPointWhite, 500, winHeight*12.2/50, 90, 90);
  //  }
    
  //  if (inByte2_3 <= 900) {
  //    if (isLandingPoint2_0 || isLandingPoint2_1 || isLandingPoint2_2 || isLandingPoint2_4) {
  //      //image(bad_imgRight, 430, 540, 100, 100);
  //    } else {
  //      //println(isLandingPoint2_0+","+isLandingPoint2_1+","+isLandingPoint2_2+","+isLandingPoint2_4);
  //      isLandingPoint2_3 = true;
  //      //image(good_imgRight, 430, 440, 100, 100);
  //    }
  //    //image(landingPoint_img, winWidth*21/25, winHeight*6/25, 170, 170);
  //    image(right_red, 0, 0);
  //  } else {
  //    isLandingPoint2_3 = false;
  //    //image(landingPointWhite, winWidth*21.9/25, winHeight*8/25, 75, 75);
  //  }
    
  //  if (inByte2_4 <= 900) {
  //    if (isLandingPoint2_3) {
  //      //image(good_imgRight, 430, 440, 100, 100);
  //    } else {
  //      isLandingPoint2_4 = true;
  //      //image(bad_imgRight, 430, 540, 100, 100);
  //    }
  //    //image(landingPoint_img, winWidth*17.5/25, winHeight*18/25, 220, 220);
  //    image(right_blue, 0, 0);
  //  } else {
  //    isLandingPoint2_4 = false;
  //    //image(landingPointWhite, 601, 660, 90, 90);
  //  }
  //}
  
  if (mousePressed) {
      if (mouseX>=0 && mouseX<800 && mouseY>=0 && mouseY<=800) {
        //outputPeak.flush();
        //outputPeak.close();
        output.flush();
        output.close();
        println("File Close");
      }
  }
  
  //Left
  if(myPort1.available()>0){
    image(whiteBoardGood_imgLeft, 270, 440, 100, 100);
    image(whiteBoardBad_imgLeft, 270, 540, 100, 100);
    for (int i = 0; i < pressOrderLeft.length; i++) {
        pressOrderLeft[i] = 10;
    }
    if (inByte1_0 <= 900) {
      if (isLandingPoint1_3) {
        //image(good_imgLeft, 270, 440, 100, 100);
      } else {
        isLandingPoint1_0 = true;
        if (inByte1_0 <= peak1_0) {
          peak1_0 = inByte1_0;
        } else if (inByte1_0 > peak1_0) {
          //outputPeak.println(peak1_0);
        }
        if (isLandingPoint1_1 || isLandingPoint1_2 || isLandingPoint1_3 || isLandingPoint1_4) {
          
        } else {
          groundTimeLeft = millis();
        }
        //image(bad_imgLeft, 270, 540, 100, 100);
      }
      for (int i = 0; i < pressOrderLeft.length; i++) {
        if (pressOrderLeft[i] == 10) {
          pressOrderLeft[i] = 0;
          break;
        } else {
          continue;
        }
      }
      image(landingPoint_img, 190, 0, 180, 180);
    } else {
      isLandingPoint1_0 = false;
      image(landingPointWhite, winWidth*28/100, 65, 75, 75);
    }
    
    if (inByte1_1 <= 1000) {
      if (isLandingPoint1_3) {
        //image(good_imgLeft, 270, 440, 100, 100);
      } else {
        isLandingPoint1_1 = true;
        if (inByte1_1 <= peak1_1) {
          peak1_1 = inByte1_1;
        } else if (inByte1_1 > peak1_1) {
          //outputPeak.println(","+peak1_1);
        }
        if (isLandingPoint1_0 || isLandingPoint1_2 || isLandingPoint1_3 || isLandingPoint1_4) {
          
        } else {
          groundTimeLeft = millis();
        }
        //image(bad_imgLeft, 270, 540, 100, 100);
      }
      for (int i = 0; i < pressOrderLeft.length; i++) {
        if (pressOrderLeft[i] == 10) {
          pressOrderLeft[i] = 1;
          break;
        } else {
          continue;
        }
      }
      image(landingPoint_img, 9, winHeight*4/25, 100, 100);
    } else {
      isLandingPoint1_1 = false;
      image(landingPointWhite, 28, 166, 40, 40);
    }
    
    if (inByte1_2 <= 900) {
      if (isLandingPoint1_3) {
        //image(good_imgLeft, 270, 440, 100, 100);
      } else {
        isLandingPoint1_2 = true;
        if (inByte1_2 <= peak1_2) {
          peak1_2 = inByte1_2;
        } else if (inByte1_2 > peak1_2) {
          //outputPeak.println(","+","+peak1_2);
        }
        if (isLandingPoint1_0 || isLandingPoint1_1 || isLandingPoint1_3 || isLandingPoint1_4) {
          
        } else {
          groundTimeLeft = millis();
        }
        //image(bad_imgLeft, 270, 540, 100, 100);
      }
      for (int i = 0; i < pressOrderLeft.length; i++) {
        if (pressOrderLeft[i] == 10) {
          pressOrderLeft[i] = 2;
          break;
        } else {
          continue;
        }
      }
      image(landingPoint_img, 170, 120, 210, 210);
    } else {
      isLandingPoint1_2 = false;
      image(landingPointWhite, 200, winHeight*6/25, 100, 100);
    }
    
    if (inByte1_3 <= 1000) {
      if (isLandingPoint1_0 || isLandingPoint1_1 || isLandingPoint1_2 || isLandingPoint1_4) {
        //image(bad_imgLeft, 270, 540, 100, 100);
      } else {
        isLandingPoint1_3 = true;
        if (inByte1_3 <= peak1_3) {
          peak1_3 = inByte1_3;
        } else if (inByte1_3 > peak1_3) {
          //outputPeak.println(","+","+","+peak1_3);
        }
        if (isLandingPoint1_0 || isLandingPoint1_1 || isLandingPoint1_2 || isLandingPoint1_4) {
          
        } else {
          groundTimeLeft = millis();
        }
        //image(good_imgLeft, 270, 440, 100, 100);
      }
      for (int i = 0; i < pressOrderLeft.length; i++) {
        if (pressOrderLeft[i] == 10) {
          pressOrderLeft[i] = 3;
          break;
        } else {
          continue;
        }
      }
      image(landingPoint_img, 5, winHeight*6/25, 170, 170);
    } else {
      isLandingPoint1_3 = false;
      image(landingPointWhite, 33, winHeight*8/25, 75, 75);
    }
    
    if (inByte1_4 <= 900) {
      if (isLandingPoint1_3) {
        //image(good_imgLeft, 270, 440, 100, 100);
      } else {
        isLandingPoint1_4 = true;
        if (inByte1_4 <= peak1_4) {
          peak1_4 = inByte1_4;
        } else if (inByte1_4 > peak1_4) {
          //outputPeak.println(","+","+","+","+peak1_4);
        }
        if (isLandingPoint1_0 || isLandingPoint1_1 || isLandingPoint1_2 || isLandingPoint1_3) {
          
        } else {
          groundTimeLeft = millis();
        }
        //image(bad_imgLeft, 270, 540, 100, 100);
      }
      for (int i = 0; i < pressOrderLeft.length; i++) {
        if (pressOrderLeft[i] == 10) {
          pressOrderLeft[i] = 4;
          break;
        } else {
          continue;
        }
      }
      image(landingPoint_img, 70, winHeight*18/25, 220, 220);
    } else {
      isLandingPoint1_4 = false;
      image(landingPointWhite, 111, 660, 90, 90);
    }
    if (!isLandingPoint1_0 && !isLandingPoint1_1 && !isLandingPoint1_2 && !isLandingPoint1_3 && !isLandingPoint1_4) {
      landingTimeLeft = millis();
      diffLeft = landingTimeLeft - groundTimeLeft;
      peak1_0 = 2000;
      peak1_1 = 2000;
      peak1_2 = 2000;
      peak1_3 = 2000;
      peak1_4 = 2000;
    }
    output.println(diffLeft+","+pressOrderLeft[0]+","+pressOrderLeft[1]+","+pressOrderLeft[2]+","+pressOrderLeft[3]+","+pressOrderLeft[4]+","+","+","+","+","+",");
  }
  
  // Right
  if(myPort2.available()>0){
    image(whiteBoardGood_imgRight, 430, 440, 100, 100);
    image(whiteBoardBad_imgRight, 430, 540, 100, 100);
    for (int i = 0; i < pressOrderRight.length; i++) {
        pressOrderRight[i] = 10;
    }
    if (inByte2_0 <= 950) {
      if (isLandingPoint2_3) {
        //image(good_imgRight, 430, 440, 100, 100);
      } else {
        isLandingPoint2_0 = true;
        if (inByte2_0 <= peak2_0) {
          peak2_0 = inByte2_0;
        } else if (inByte2_0 > peak2_0) {
          //outputPeak.println(","+","+","+","+","+peak2_0);
        }
        if (isLandingPoint2_1 || isLandingPoint2_2 || isLandingPoint2_3 || isLandingPoint2_4) {
          
        } else {
          groundTimeRight = millis();
        }
        //image(bad_imgRight, 430, 540, 100, 100);
      }
      for (int i = 0; i < pressOrderRight.length; i++) {
        if (pressOrderRight[i] == 10) {
          pressOrderRight[i] = 0;
          break;
        } else {
          continue;
        }
      }
      image(landingPoint_img, 468, 0, 180, 180);
    } else {
      isLandingPoint2_0 = false;
      image(landingPointWhite, 500, 65, 75, 75);
    }
    
    if (inByte2_1 <= 950) {
      if (isLandingPoint2_3) {
        //image(good_imgRight, 430, 440, 100, 100);
      } else {
        isLandingPoint2_1 = true;
        if (inByte2_1 <= peak2_1) {
          peak2_1 = inByte2_1;
        } else if (inByte2_1 > peak2_1) {
          //outputPeak.println(","+","+","+","+","+","+peak2_1);
        }
        if (isLandingPoint2_0 || isLandingPoint2_2 || isLandingPoint2_3 || isLandingPoint2_4) {
          
        } else {
          groundTimeRight = millis();
        }
        //image(bad_imgRight, 430, 540, 100, 100);
      }
      for (int i = 0; i < pressOrderRight.length; i++) {
        if (pressOrderRight[i] == 10) {
          pressOrderRight[i] = 1;
          break;
        } else {
          continue;
        }
      }
      image(landingPoint_img, 715, winHeight*4/25, 100, 100);
    } else {
      isLandingPoint2_1 = false;
      image(landingPointWhite, 734, 166, 40, 40);
    }
    
    if (inByte2_2 <= 950) {// 900
      if (isLandingPoint2_3) {
        //image(good_imgRight, 430, 440, 100, 100);
      } else {
        isLandingPoint2_2 = true;
        if (inByte2_2 <= peak2_2) {
          peak2_2 = inByte2_2;
        } else if (inByte2_2 > peak2_2) {
          //outputPeak.println(","+","+","+","+","+","+","+peak2_2);
        }
        if (isLandingPoint2_0 || isLandingPoint2_1 || isLandingPoint2_3 || isLandingPoint2_4) {
          
        } else {
          groundTimeRight = millis();
        }
        //image(bad_imgRight, 430, 540, 100, 100);
      }
      for (int i = 0; i < pressOrderRight.length; i++) {
        if (pressOrderRight[i] == 10) {
          pressOrderRight[i] = 2;
          break;
        } else {
          continue;
        }
      }
      image(landingPoint_img, winWidth*29/50, 120, 210, 210);
    } else {
      isLandingPoint2_2 = false;
      image(landingPointWhite, 500, winHeight*12.2/50, 90, 90);
    }
    
    if (inByte2_3 <= 800) {
      if (isLandingPoint2_0 || isLandingPoint2_1 || isLandingPoint2_2 || isLandingPoint2_4) {
        //image(bad_imgRight, 430, 540, 100, 100);
      } else {
        //println(isLandingPoint2_0+","+isLandingPoint2_1+","+isLandingPoint2_2+","+isLandingPoint2_4);
        isLandingPoint2_3 = true;
        if (inByte2_3 <= peak2_3) {
          peak2_3 = inByte2_3;
        } else if (inByte2_3 > peak2_3) {
          //outputPeak.println(","+","+","+","+","+","+","+","+peak2_3);
        }
        if (isLandingPoint2_0 || isLandingPoint2_1 || isLandingPoint2_2 || isLandingPoint2_4) {
          
        } else {
          groundTimeRight = millis();
        }
        //image(good_imgRight, 430, 440, 100, 100);
      }
      for (int i = 0; i < pressOrderRight.length; i++) {
        if (pressOrderRight[i] == 10) {
          pressOrderRight[i] = 3;
          break;
        } else {
          continue;
        }
      }
      image(landingPoint_img, winWidth*21/25, winHeight*6/25, 170, 170);
    } else {
      isLandingPoint2_3 = false;
      image(landingPointWhite, winWidth*21.9/25, winHeight*8/25, 75, 75);
    }
    
    if (inByte2_4 <= 800) {
      if (isLandingPoint2_3) {
        //image(good_imgRight, 430, 440, 100, 100);
      } else {
        isLandingPoint2_4 = true;
        if (inByte2_4 <= peak2_4) {
          peak2_4 = inByte2_4;
        } else if (inByte2_4 > peak2_4) {
          //outputPeak.println(","+","+","+","+","+","+","+","+","+peak2_4);
        }
        if (isLandingPoint2_0 || isLandingPoint2_1 || isLandingPoint2_2 || isLandingPoint2_3) {
          
        } else {
          groundTimeRight = millis();
        }
        //image(bad_imgRight, 430, 540, 100, 100);
      }
      for (int i = 0; i < pressOrderRight.length; i++) {
        if (pressOrderRight[i] == 10) {
          pressOrderRight[i] = 4;
          break;
        } else {
          continue;
        }
      }
      image(landingPoint_img, winWidth*17.5/25, winHeight*18/25, 220, 220);
    } else {
      isLandingPoint2_4 = false;
      image(landingPointWhite, 601, 660, 90, 90);
    }
    if (!isLandingPoint2_0 && !isLandingPoint2_1 && !isLandingPoint2_2 && !isLandingPoint2_3 && !isLandingPoint2_4) {
      landingTimeRight = millis();
      diffRight = landingTimeRight - groundTimeRight;
      peak2_0 = 2000;
      peak2_1 = 2000;
      peak2_2 = 2000;
      peak2_3 = 2000;
      peak2_4 = 2000;
    }
  }
  output.println(","+","+","+","+","+","+diffRight+","+pressOrderRight[0]+","+pressOrderRight[1]+","+pressOrderRight[2]+","+pressOrderRight[3]+","+pressOrderRight[4]);
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
        inByte1_0 = analog1_0_high*256 + analog1_0_low;
        inByte1_1 = analog1_1_high*256 + analog1_1_low;
        inByte1_2 = analog1_2_high*256 + analog1_2_low;
        inByte1_3 = analog1_3_high*256 + analog1_3_low;
        inByte1_4 = analog1_4_high*256 + analog1_4_low;
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
        inByte2_0 = analog2_0_high*256 + analog2_0_low;
        inByte2_1 = analog2_1_high*256 + analog2_1_low;
        inByte2_2 = analog2_2_high*256 + analog2_2_low;
        inByte2_3 = analog2_3_high*256 + analog2_3_low;
        inByte2_4 = analog2_4_high*256 + analog2_4_low;
      }
    }
  }
  output.println(time1+","+inByte1_0+","+inByte1_1+","+inByte1_2+","+inByte1_3+","+inByte1_4+","+time2+","+inByte2_0+","+inByte2_1+","+inByte2_2+","+inByte2_3+","+inByte2_4);
  println("time1="+time1+"inByte1_0="+inByte1_0+", inByte1_1="+inByte1_1+", inByte1_2="+inByte1_2+", inByte1_3="+inByte1_3+", inByte1_4="+inByte1_4);
  println("time2="+time2+"inByte2_0="+inByte2_0+", inByte2_1="+inByte2_1+", inByte2_2="+inByte2_2+", inByte2_3="+inByte2_3+", inByte2_4="+inByte2_4);
}