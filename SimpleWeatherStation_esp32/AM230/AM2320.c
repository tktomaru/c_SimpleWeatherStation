
//############  DHT   ############


double readTemperature(Adafruit_AM2320 am2320)
{
  int ii = 0;
  double temp = 0;
  u8x8.drawString(0, 3, "readTemperature");
  temp = am2320.readTemperature();
  DBG(temp); DBG("\n");
  for ( ii = 0; (ii < 3) && (isnan(temp)); ii++ ) {
    delay(1000);
    temp = am2320.readTemperature();
    DBG(temp); DBG("\n");
    if(isnan(temp)){
      digitalWrite(HARD_RESET, LOW);
      delay(1000);
      digitalWrite(HARD_RESET, HIGH);
    }
  }
  if (isnan(temp)) {
    temp = 0;
  }
  return temp;
}

double readHumidity(Adafruit_AM2320 am2320)
{
  int ii = 0;
  double humid = 0;
  u8x8.drawString(0, 3, "readHumidity   ");
  humid = am2320.readHumidity();
  DBG(humid); DBG("\n");
  for ( ii = 0; (ii < 3) && (isnan(humid)); ii++ ) {
    delay(1000);
    humid = am2320.readHumidity();
    DBG(humid); DBG("\n");
    if(isnan(humid)){
      digitalWrite(HARD_RESET, LOW);
      delay(1000);
      digitalWrite(HARD_RESET, HIGH);
    }
  }
  if (isnan(humid)) {
    humid = 0;
  }
  return humid;
}