// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralShooter;
import frc.robot.subsystems.DriveTrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Recto extends Command {
  /** Creates a new Recto. */
  private DriveTrain drive = DriveTrain.getInstance();
  private CoralShooter coral =  CoralShooter.getInstance();
  private double setpoint;
  private Timer time = new Timer();
  public Recto(double time) {
    addRequirements(drive);
    this.setpoint = time;
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
    if (time.get() < setpoint) {
      
      drive.controlledDrive(0.6, 0, false);
    } else if (time.get() >= setpoint){
      drive.controlledDrive(0, 0, false);
      coral.posiciones();
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
    return time.get() > setpoint + 1.5;

  }
}
