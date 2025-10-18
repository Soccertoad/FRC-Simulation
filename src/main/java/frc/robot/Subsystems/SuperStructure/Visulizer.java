package frc.robot.Subsystems.SuperStructure;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Subsystems.Elevator.ElevatorConstants;

public class Visulizer {

  private String name;
  private LoggedMechanism2d mech = new LoggedMechanism2d(
    Meters.convertFrom(34, Inches), 
    ElevatorConstants.maxExtension.in(Meters), 
    new Color8Bit(Color.kBlack)
  );
  private LoggedMechanismRoot2d elevatorRoot;
  private LoggedMechanismLigament2d elevator;

  private Angle uprightElevator = Degree.of(90);
  
  public Visulizer(String name){
    this.name = name;
    elevatorRoot = mech.getRoot(
      name + " Elevator Root", 
      Meters.convertFrom(15, Inches), 
      Meters.convertFrom(0, Inches)
    );

    elevator = elevatorRoot.append(
      new LoggedMechanismLigament2d(
        name + " Elevator", 
        ElevatorConstants.maxExtension.in(Meters), 
        ElevatorConstants.startingAngle.in(Degree), 
        4, 
        new Color8Bit(Color.kDarkCyan)
      )
    );
  }

  public void update(double elevatorHeightMeters, boolean isTilted){
    if(!isTilted){
      elevator.setAngle(uprightElevator.in(Degree));
    }
    elevator.setLength(elevatorHeightMeters);
    Logger.recordOutput("Mech2d/" + name, mech);
  }

  public void update(Distance elevatorHeightMeters, boolean isTilted){
    update(elevatorHeightMeters.in(Meters), isTilted);
  }


}
