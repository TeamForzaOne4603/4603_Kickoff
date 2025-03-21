// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class climbing extends Command {
  /** Creates a new climbing. */
  private Climber climber = Climber.getInstance();

  private boolean check_A = false;
  private boolean check_B = false;
  private boolean check_X = false;
  private boolean check_Y = false;
  private BooleanSupplier a;
  private BooleanSupplier b;
  private BooleanSupplier x;
  private BooleanSupplier y;


  public climbing( BooleanSupplier A, BooleanSupplier B, BooleanSupplier X, BooleanSupplier Y) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climber);
    this.a = A;
    this.b =B;
    this.x = X;
    this.y = Y;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    if (a.getAsBoolean()) {
      climber.Brazo(0.3);
      check_A = true;
    } else if (check_A == true) {
      climber.Brazo(0);
      check_A = false;
    }

    if (b.getAsBoolean()) {
      climber.Brazo(-0.3);
      check_B = true;
    } else if (check_B == true) {
      climber.Brazo(0);
      check_B = false;
    }

    if (x.getAsBoolean()) {
      climber.spool(-0.3);
      check_X = true;
    } else if (check_X == true) {
      climber.Spool(0);
      check_X = false;
    }

    if (y.getAsBoolean()) {
      climber.spool(0.3);
      check_Y = true;
    } else if (check_Y == true) {
      climber.spool(0);
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
