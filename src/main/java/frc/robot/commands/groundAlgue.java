// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Desalguificador;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class groundAlgue extends Command {
  /** Creates a new groundAlgue. */
  private Timer timer = new Timer();
  private Desalguificador algas = Desalguificador.getInstance();
  private double position = 0;
  public groundAlgue(double Position) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(algas);
    this.position = Position;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.stop();
    timer.reset();
    timer.start();
    algas.setPosition(position);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (timer.get() > 2) {
      algas.setAlgue(0.5);
    }

    /*if (algas.isInPosition()) {
      algas.setAlgue(0.5);
    }*/
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    algas.setAlgue(0);
    algas.setPosition(0); 
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
