// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralShooter;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CoralIntake extends Command {
  /** Creates a new CoralIntake. */
  private CoralShooter coral = CoralShooter.getInstance();
  private Timer timer = new Timer();
  private boolean check = false;
  private boolean detect = false;
  public CoralIntake() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(coral);
  }

  public CoralIntake(BooleanSupplier prueba) {
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

  @Override
  public void execute() {
    //Starts intake when the laserCAN detects coral
    if (timer.get() > 0.17) {
      coral.setSpeed(0.1);
      detect = true;
    }
    if(check == false && coral.getLaser() < 90) {
      
      coral.setSpeed(0.2);
    }

    //Starts timer when the colorsensor detects a coral
    if (check == false && coral.getColor() ) {
      check = true;
      timer.start();
      coral.setSpeed(0.135);
    } 

  }

  @Override
  public void end(boolean interrupted) {
    coral.setSpeed(0);
  }

  @Override
  public boolean isFinished() {
    //Ends 0.065 seconds after the colorsensor detects the coral
    return timer.get()>0.03;
  }
}
