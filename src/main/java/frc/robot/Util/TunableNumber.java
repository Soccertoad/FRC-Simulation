package frc.robot.Util;

import java.util.Arrays;
import java.util.function.DoubleSupplier;

import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants;

public class TunableNumber implements DoubleSupplier {
  private static final String DIRECTORY = "/Tunable";
  private final NetworkTable NTable = NetworkTableInstance.getDefault().getTable(DIRECTORY);
  private final String key;

  private DoubleEntry NEntry;
  private DoubleTopic NTopic;

  private boolean hasDefault = false;
  private double defaultValue;
  private double tunableValue;  
  

  public TunableNumber(String m_key){
    this.key = m_key;
  }

  public TunableNumber(String m_key, double m_defaultValue){
    this(m_key);
    initalizeDefault(m_defaultValue);
  }

  public void initalizeDefault(double m_defaultValue){
    if(!hasDefault){

      hasDefault = true;
      this.defaultValue = m_defaultValue;

      if(Constants.LIVE_TUNING){
        NTopic = NTable.getDoubleTopic(key);
        NEntry = NTopic.getEntry(m_defaultValue);
      }
    }
  }

  public double get(){
    if(!hasDefault){
      return 0.0;
    } else {
      return Constants.LIVE_TUNING ? NEntry.get() : defaultValue;
    }
  }

  public boolean hasChanged(){
    if(!Constants.LIVE_TUNING) return false;

    double currentValue = get();
    double lastValue = tunableValue;
    if(currentValue != lastValue){
      tunableValue = currentValue;
      NEntry.set(tunableValue);
      return true;
    }
    return false;
  }

  public static boolean hasChanged(TunableNumber... tunables){
    if(Arrays.stream(tunables).anyMatch(tunable-> tunable.hasChanged())){
      return true;
    }
    return false;
  }

  @Override
  public double getAsDouble() {
    return get();
  }
}
