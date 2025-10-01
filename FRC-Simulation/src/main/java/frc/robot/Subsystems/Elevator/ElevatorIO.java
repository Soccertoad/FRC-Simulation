package frc.robot.Subsystems.Elevator;

import edu.wpi.first.units.measure.Distance;

public interface ElevatorIO {

  public class ElevatorIOInputs{
    public double tempCelcius = 0;
    public double positionMeters = 0;
    public double velocityMetersPerSecond = 0;

    public double appliedVolts = 0;
    public double statorCurrent = 0;
    public double supplyCurrent = 0;
  }

  default void updateInputs(ElevatorIOInputs inputs) {}

  default void goToDistance(Distance meters) {}

  default void setPID(double p, double i, double d, double feedforward) {}

  default void stop() {} 

  default void goToPostion(double position) {}

  default void setVoltage(double voltage) {}
}
