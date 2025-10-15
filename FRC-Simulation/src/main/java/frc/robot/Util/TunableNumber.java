package frc.robot.Util;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import frc.robot.Constants;

public class TunableNumber implements DoubleSupplier{
  private static final String basekey = "/Tuning";
  private final String key;
  private final boolean tuningMode = Constants.LIVE_TUNING;
  private double defaultValue;
  private boolean hasDefaultValue = false;
  private LoggedNetworkNumber networkNumber;

  public TunableNumber(String valueKey){
    this.key = basekey + valueKey;
  }

  public TunableNumber(String valueKey, double defaultValue){
    this(valueKey);
    setDefaultValue(defaultValue);
  }

  public void setDefaultValue(double value){
    if(!hasDefaultValue){
      this.hasDefaultValue = true;
      this.defaultValue = value;

      if(tuningMode){
        networkNumber = new LoggedNetworkNumber(key, defaultValue);
      }
    }
  }

  public double get(){
    if(!hasDefaultValue){
      return 0.0;
    } else{
    return tuningMode ? networkNumber.get() : defaultValue;
    }
  }

  public boolean hasChanged(){
    if(!tuningMode || !hasDefaultValue){
      return false;
    }
    double currentValue = get();
    if (currentValue != defaultValue){
      this.defaultValue = currentValue;
      return true;
    }
    return false;
  }

  @Override
  public double getAsDouble() {
    return get();
  }
  
}
