// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralShooter;
import frc.robot.subsystems.NewElevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ThrowInLevel extends Command {
  /** Creates a new ThrowInLevel. */
  private NewElevator elevator = NewElevator.getInstance();
  private CoralShooter coralShooter = CoralShooter.getInstance();
  private Timer timer = new Timer();
  private double setPoint;
  private boolean hasthrown = false;
  private boolean goesDown = false;
  private boolean timerstart = false;
  private boolean check1 = false;
  public ThrowInLevel(double position) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevator,coralShooter);
    this.setPoint = position;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    elevator.setPosition(setPoint);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (elevator.isInPosition() && timerstart == false) {
     
      timer.start();
      timerstart = true;
    } else if (timerstart && timer.get() >0.6&& check1 == false) {
      hasthrown = true;
      check1 = true;
    } else if (elevator.isInPosition() && hasthrown == true && timer.get() >0.8 && coralShooter.getColor() && check1 == true) {
      coralShooter.setSpeed(0.5);
    }
    if (!coralShooter.getColor() && elevator.getSetpoint() != 0) {
      coralShooter.setSpeed(0);
      elevator.setPosition(0);
      goesDown = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return elevator.isInPosition()&& goesDown == true;
  }
}
