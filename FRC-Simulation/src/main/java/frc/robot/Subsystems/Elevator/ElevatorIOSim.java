package frc.robot.Subsystems.Elevator;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class ElevatorIOSim implements ElevatorIO {

  private ElevatorSim physicsSim;
  private PIDController controller;
  private ElevatorFeedforward feedforward;
  // private LinearSystemId plant ;
  private double voltage = 0;
  private double desiredPosition;
  DCMotor motorSim = DCMotor.getKrakenX60(1);
  Distance chainLength = Inches.of(0.35);
  int sprocketTeeth = 12;
  int stages = 3;
  //Gear ratio 25:1 and drive Sprocket 12 teeth / Support/Input Sprocket Teeth
  double gearRatio = ((25 * 12) / 22);
  Mass carrageMass = Pounds.of(12);
  


  public ElevatorIOSim(){
    // plant = LinearSystemId.createElevatorSystem(
    //   DCMotor.getKrakenX60(1),
    //   0.001,
    //   1.3522961568046956 / 2,
    //   25
    // );

    physicsSim = new ElevatorSim(motorSim, gearRatio, carrageMass, stages, sprocketTeeth, gearRatio, false, desiredPosition, null)
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs){

  }

}
