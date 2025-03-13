// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DriveTrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ClimbCmd extends Command {
  /** Creates a new ClimbCmd. */
  private Climber climber = Climber.getInstance();
  private DriveTrain driveTrain = DriveTrain.getInstance();
  private CommandXboxController xbox;

  private boolean check_A = false;
  private boolean check_B = false;
  private boolean check_X = false;
  private boolean check_Y = false;

  public ClimbCmd(CommandXboxController control) {
    addRequirements(climber,driveTrain);
    this.xbox = control;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveTrain.controlledDrive(-xbox.getLeftY(),-xbox.getRightX());

    
    if (xbox.a().getAsBoolean()) {
      climber.brazo(0.3);
      check_A = true;
    } else if (check_A == true) {
      climber.brazo(0);
      check_A = false;
    }

    if (xbox.b().getAsBoolean()) {
      climber.brazo(-0.3);
      check_B = true;
    } else if (check_B == true) {
      climber.brazo(0);
      check_B = false;
    }

    if (xbox.x().getAsBoolean()) {
      climber.Spool(-0.3);
      check_X = true;
    } else if (check_X == true) {
      climber.Spool(0);
      check_X = false;
    }

    if (xbox.y().getAsBoolean()) {
      climber.Spool(0.3);
      check_Y = true;
    } else if (check_Y == true) {
      climber.Spool(0);
      check_Y = false;
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
