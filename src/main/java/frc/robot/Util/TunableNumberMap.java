// package frc.robot.Util;

// import java.util.Arrays;
// import java.util.HashMap;
// import java.util.Map;
// import java.util.function.DoubleSupplier;

// import edu.wpi.first.networktables.DoubleEntry;
// import edu.wpi.first.networktables.NetworkTable;
// import edu.wpi.first.networktables.NetworkTableInstance;
// import frc.robot.Constants;

// public class TunableNumberMap implements DoubleSupplier {
//   private static final String DIRECTORY = "/Tunable";
//   private final NetworkTable NTable = NetworkTableInstance.getDefault().getTable(DIRECTORY);

//   private final String key;
//   private boolean hasDefault = false;
//   private double defaultValue;
//   private DoubleEntry NEntry;

//   private Map<Integer, Double> tunableValues = new HashMap<>(); 
  

//   public TunableNumberMap(String m_key){
//     this.key = m_key;
//   }

//   public TunableNumberMap(String m_key, double m_defaultValue){
//     this(m_key);
//     initalizeDefault(m_defaultValue);
//   }

//   public void initalizeDefault(double m_defaultValue){
//     if(!hasDefault){

//       hasDefault = true;
//       this.defaultValue = m_defaultValue;

//       if(Constants.LIVE_TUNING){
//         NEntry = NTable.getDoubleTopic(key).getEntry(m_defaultValue);      }
//     }
//   }

//   public double get(){
//     if(!hasDefault){
//       return 0.0;
//     } else {
//       return Constants.LIVE_TUNING ? NEntry.get() : defaultValue;
//     }
//   }

//   public boolean hasChanged(int id){
//     if(!Constants.LIVE_TUNING) return false;

//     double currentValue = get();
//     Double lastValue = tunableValues.get(id);
//     if(lastValue == null || currentValue != lastValue){
//       NEntry.set(currentValue);
//       tunableValues.put(id, currentValue);
//       return true;
//     }
//     return false;
//   }

//   public static boolean hasChanged(int id, TunableNumberMap... tunables){
//     if(Arrays.stream(tunables).allMatch(tunable-> tunable.hasChanged(id))){
//       return true;
//     }
//     return false;
//   }

//   @Override
//   public double getAsDouble() {
//     return get();
//   }
// }
