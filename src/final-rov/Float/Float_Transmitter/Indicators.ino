void Radio_Indicate(){

  if( RadioState ==HIGH )
 {
    if( (millis()- rememberTime) >= 50){   
    RadioState = LOW;
    rememberTime=millis();
    }
 }
 else
 {   
    if( (millis()- rememberTime) >= 100){     
    RadioState =HIGH;
    rememberTime=millis();
    }
 }

 // Robojax LED blink with millis()
 digitalWrite(RadioIndicator,RadioState);// turn the LED ON or OFF
 
}

void Saving_Indicate(){
digitalWrite(Activation_Indicator, LOW);
if( SavingState == HIGH )
 {
    if( (millis()- rememberTime) >= 10){   
    SavingState = LOW;
    rememberTime2=millis();
    }
 }
 else
 {   
    if( (millis()- rememberTime) >= 10){     
    SavingState =HIGH;
    rememberTime2=millis();
    }
 }

 // Robojax LED blink with millis()
 digitalWrite(Activation_Indicator,SavingState);// turn the LED ON or OFF

}