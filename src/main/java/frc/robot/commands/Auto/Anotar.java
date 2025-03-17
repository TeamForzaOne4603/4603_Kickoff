// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.NewElevatorConstants;
import frc.robot.subsystems.CoralShooter;
import frc.robot.subsystems.NewElevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Anotar extends Command {
  /** Creates a new Anotar. */
  private NewElevator elevator = NewElevator.getInstance();
  private CoralShooter coralShooter = CoralShooter.getInstance();
  private double position;
  private boolean hascoral = true;
  private boolean justInCase = false;
  
 
  public Anotar(double sp) {
    addRequirements(elevator,coralShooter);
    this.position =sp;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elevator.setPosition(position);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (elevator.isInPosition() && coralShooter.getColor()) {
      coralShooter.posiciones();
    } else if(!coralShooter.getColor() && hascoral == true){
      hascoral = false;
    } else if (hascoral = false){coralShooter.setSpeed(0);
      elevator.setPosition(NewElevatorConstants.kStowHeight);
      justInCase = true;
    }
  }

  // Called once the command ends or is interrupted. 
  @Override
  public void end(boolean interrupted) {coralShooter.setSpeed(0);}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return hascoral = false && elevator.isInPosition() && justInCase == true;
  }
}
