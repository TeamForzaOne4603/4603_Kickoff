// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralShooter;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CoralIntake extends Command {
  /** Creates a new CoralIntake. */
  private CoralShooter coral = CoralShooter.getInstance();
  private Timer timer = new Timer();
  private boolean check = false;
  public CoralIntake() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(coral);
  }



  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    check = false;
    timer.stop();
    timer.reset();
    timer.stop();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
     if (check == false && !coral.getSensor()) {
      coral.setSpeed(0.2);
      check = true;
      timer.start();
      //inicia el timer en cuanto el sensor amarillo detecte coral
    } 
    if(check == false && coral.getLaser() < 90) {
      coral.setSpeed(0.2);
      //solo inicia si deteca el coral
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    coral.setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.get() > 0.16;
    //return !coral.getSensor() && coral.getLaser() > 50;

  }
}
