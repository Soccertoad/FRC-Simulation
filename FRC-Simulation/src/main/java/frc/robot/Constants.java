package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

public class Constants {

  public static final Mode SIM_MODE = Mode.SIM;

  public static final Mode CURRENT_MODE = RobotBase.isReal() ? Mode.REAL : SIM_MODE;

  public enum Mode {
    REAL,

    SIM,
    
    REPLAY
  }
}
