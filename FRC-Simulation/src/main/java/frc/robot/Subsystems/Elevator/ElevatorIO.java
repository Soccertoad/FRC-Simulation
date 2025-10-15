package frc.robot.Subsystems.Elevator;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;

public interface ElevatorIO {

  @AutoLog
  public class ElevatorIOInputs{
    public double tempCelcius = 0;
    public double positionMeters = 0;
    public double velocityMetersPerSecond = 0;
    public double desiredPositionMeters = 0;

    public double appliedVolts = 0;
    public double statorCurrent = 0;
    public double supplyCurrent = 0;
  }

  default void updateInputs(ElevatorIOInputs inputs) {}

  default void setPID(double p, double i, double d, double feedforward) {}

  default void stop() {} 

  default void setPosition(Distance distance) {}

  default void setVoltage(Voltage voltage) {}
}
