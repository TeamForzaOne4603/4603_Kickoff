// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import com.ctre.phoenix6.hardware.core.CoreTalonFX;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.NewElevatorConstants;
import frc.robot.subsystems.CoralShooter;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.NewElevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Completo extends Command {
  /** Creates a new Desesperacion. */
  private CoralShooter coral = CoralShooter.getInstance();
  private NewElevator elevator = NewElevator.getInstance();
  private DriveTrain drive = DriveTrain.getInstance();
  private Timer timer = new Timer();


  public Completo() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(coral,elevator,drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (timer.get()<1.5) {
      drive.controlledDrive(0.4, 0, false);
    } else if (timer.get() < 4) {
      drive.controlledDrive(0, 0,false);
      elevator.goToPosition(NewElevatorConstants.kL3Height);
    } else if(timer.get() < 5.5){
      coral.shootPosition();
    } else if (timer.get() < 8) {
      coral.setSpeed(0);
      elevator.goToPosition(NewElevatorConstants.kStowHeight);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
