package frc.robot.Subsystems.Elevator;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;

public final class ElevatorConstants {
  public static final DCMotor motorSim = DCMotor.getKrakenX60(1);
  public static final Distance chainPitch = Inches.of(0.35);
  public static final Distance maxExtension = Feet.of(9.0);
  public static final Mass carrageMass = Pounds.of(5);

  public static final int stages = 3;
  public static final int driveSprocketTeeth = 12;
  public static final int supportSprocketTeeth = 22;
  public static final double gearBoxReduction = 25;
  //Gear ratio 25:1 and drive Sprocket 12 teeth / Support/Input Sprocket Teeth
  public static final double totalReduction = ((gearBoxReduction * driveSprocketTeeth) / supportSprocketTeeth);
  public static final double drumCircumferenceMeters = driveSprocketTeeth * chainPitch.in(Meters);
  public static final double drumRadiusMeters = drumCircumferenceMeters / Math.PI / 2;
}
