void loop() {
  static unsigned long timer,powerTimer = 0;

  if (millis() > powerTimer) {
    
    powerTimer += POWER_INTERVAL;
    
    //Motor Current
    I_M = ((readADC(5) * VREF)/4095.0);    
    //I_M = I_M + 0.01;
    I_M = I_M - V_5/4.96;
    //I_M = I_M *1000; // convert to mV
    //I_M = I_M/1000; //divide differential gain
    //I_M = I_M - 0.1; // offset error 
    //I_M = I_M + 0.02;
    I_M = I_M*6.7;
    I_M = I_M/100; //divide resistance for current
    I_mAvg += I_M;
    count = count+1;
    
    //Motor Voltage
    V_M = ((readADC(2) * VREF)/4095.0); // Voltage divider circuit
    V_M = V_M+0.03;//quantization error + error from connecting to pin
    V_M = V_M - 0.013;
    V_M = V_M* 6.15;

    //5V current
    I_5 = ((readADC(3) * VREF)/4095.0) ; //this time only gain 100
    I_5 = I_5+ 0.03; // quant error
    I_5 = I_5 - V_5/4.96; // reference voltage get actual differential
    I_5 = I_5*1000; //convert to mV
    I_5 = I_5/100; // divide differential gain
    I_5 = I_5 -0.1; // offset error

    I_5 = I_5/10; // convert to current value
    I_5Avg += I_5;

    //Battery Voltage
    V_B = ((readADC(4) * VREF)/4095.0)+0.034; //0.03 quantization error
    V_B = V_B - 0.02; //input offset voltage
    V_B = V_B *6.17; //voltage divider
    
    //5V
    V_5 = ((readADC(0) * VREF)/4095.0); 
    V_5 = V_5 + 0.017;// quantization error
    V_5 = V_5 -0.007; //input offset voltage
    V_5 = V_5*4.96; //Voltage divider
    //Serial.println(V_5);
    //CouloumbCount = CouloumbCount + (I_M + I_5)*POWER_INTERVAL/1000; //rectangle approximation otherwise previous values must be saved

  }



  if (millis() > timer) {
    
    I_mAvg/=count;
    I_5Avg/=count;
    
    CoulombCount = CoulombCount + (I_mAvg + I_5Avg)*(millis()-prev_power_time)/1000; //rectangle approximation otherwise previous values must be saved
    

    timer += 1000;
    Serial.print("Current Kp: ");
    Serial.println(Kp);
    Serial.print("Current battery: ");
    Serial.println(percentage-(CoulombCount/ratedCoulomb)*100);
    Serial.print("Current turn: ");
    Serial.println(turn);


    if (clientSocket < 0) {                // reconnect if needed
      Socketconnect();
    }




    // Optionally send Kp value to server every second
    if (clientSocket > 0) {
      char bigbuffer[512];
      snprintf(bigbuffer, sizeof(bigbuffer),
              "Kp=%.2f\nBAT:%f\nBATVOLT:%f\n5VVOLT:%f\nMOTVOLT:%f\nMOTCURR:%f\n5VCURR:%f\n5V_Power:%f\nMotor_Power:%f\n",
              Kp,
              percentage-(CoulombCount/ratedCoulomb) * 100,
              V_B,
              V_5,
              V_M,
              I_mAvg,
              I_5Avg,
              V_5*I_5Avg,
              V_M*I_mAvg);
      //percentage -= 0.01;

      Serial.print(bigbuffer);
      send(clientSocket, bigbuffer, strlen(bigbuffer), 0);


    }
    I_mAvg = 0;
    I_5Avg = 0;
    count = 0;
    prev_power_time = millis();
  }
}