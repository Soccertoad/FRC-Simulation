package frc.robot.Subsystems.Elevator;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Distance;
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
  double gearRatio = ((25 * 12) / 22);


  public ElevatorIOSim(){
    // plant = LinearSystemId.createElevatorSystem(
    //   DCMotor.getKrakenX60(1),
    //   0.001,
    //   1.3522961568046956 / 2,
    //   25
    // );

    physicsSim = new ElevatorSim(, voltage, voltage, voltage, voltage, voltage, false, voltage, null)
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs){

  }

}
