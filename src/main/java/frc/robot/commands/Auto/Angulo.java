// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//apollo pistola de silicon

package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Angulo extends Command {
  /** Creates a new Angulo. */
  private DriveTrain drive = DriveTrain.getInstance();
  private double setpoint;
  public Angulo(double position) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
    this.setpoint = position;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drive.goToAngle(setpoint);
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.controlledDrive(0, 0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return drive.isInAngle(setpoint);
  }
}
