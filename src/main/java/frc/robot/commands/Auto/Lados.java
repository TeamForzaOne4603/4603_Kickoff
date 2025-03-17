// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.NewElevatorConstants;
import frc.robot.subsystems.CoralShooter;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.NewElevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Lados extends Command {
  private DriveTrain drive = DriveTrain.getInstance();
  private CoralShooter coral =  CoralShooter.getInstance();
  private NewElevator elevator = NewElevator.getInstance();
  private double setpoint;
  private Timer time = new Timer();
  public Lados(double position) {
    addRequirements(drive);
    this.setpoint = position;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    time.reset();
    time.start();
    //drive.goSimple();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (time.get() < 2) {
      
      drive.controlledDrive(0.4, 0, false);
    } else if (time.get() >= 2 && time.get() <= 4.7){
      drive.controlledDrive(0, 0, false);
      elevator.setPosition(NewElevatorConstants.kL3Height);
    } else if (time.get() > 4.7 && time.get() <= 6) {
      coral.posiciones();
    }else if (time.get() > 6 && time.get() <= 9) {
      coral.setSpeed(0);
      elevator.setPosition(NewElevatorConstants.kStowHeight);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.controlledDrive(0, 0, false);
    coral.setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return time.get() > 11;

  }
}
