// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Pruebas_Intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralShooter;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeMustang extends Command {
  private CoralShooter coral = CoralShooter.getInstance();
  private boolean check = false;
  private boolean invalid = false;
  
  private Timer timer2 = new Timer();
  private boolean timecheck = false;

  public IntakeMustang() {
    addRequirements(coral);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    check = false;
    invalid = false;
    
    timecheck = false;
    timer2.reset();
    if (!coral.getColor()) {
      coral.setSpeed(0.3);
    } else {
      invalid = true;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (coral.getColor()) {
      coral.setSpeed(0.15);
      check = true;
    }

    if (coral.getLaser() > 150 && timecheck == false) {
      timer2.start();
      timecheck = true;
    } else if (!(coral.getLaser() > 150)) {
      timer2.reset();
      timecheck = false;
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
    return invalid && (coral.getLaser() > 150 && check == true && timer2.get() > 0.1);
  }
}
