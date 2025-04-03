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
  private Timer timer2 = new Timer();
  private boolean timecheck = false;
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
    timecheck = false;
    check = false;
    detect = false;
    timer.stop();
    timer2.stop();
    timer.reset();
    timer2.reset();
  }

  @Override
  public void execute() {
    //Starts intake when the laserCAN detects coral
    if (timer.get() > 0.015) {
      coral.setSpeed(0.12);
      detect = true;
    }
    if(check == false && coral.getLaser() < 90) {
      
      coral.setSpeed(0.25);
    }

    //Starts timer when the colorsensor detects a coral
    if (check == false && coral.getColor() ) {
      check = true;
      timer.start();
      coral.setSpeed(0.14);
    } 

    if (coral.getLaser() > 180 && timecheck == false) {
      timer2.start();
      timecheck = true;
    } else if (!(coral.getLaser() > 180)) {
      timer2.reset();
      timecheck = false;
    }

  }

  @Override
  public void end(boolean interrupted) {
    coral.setSpeed(0);
  }

  @Override
  public boolean isFinished() {
    //Ends 0.065 seconds after the colorsensor detects the coral
    return detect == true  && coral.getLaser() > 180 && check == true && timer2.get()>0.04;
  }
}
