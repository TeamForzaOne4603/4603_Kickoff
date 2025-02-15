// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.CoralShooter;
import frc.robot.subsystems.Desalguificador;
import frc.robot.subsystems.DriveTrain;

public class RobotContainer {
  
  private final CommandXboxController joy_drive = new CommandXboxController(1);
  private final CommandXboxController joy_op = new CommandXboxController(2);
  private final DriveTrain tankDrive = new DriveTrain();
  

  public RobotContainer() {
    tankDrive.setDefaultCommand(new RunCommand(() -> tankDrive.controlledDrive(joy_drive.getLeftY(),joy_drive.getRightX()), tankDrive));
    configureBindings();
  }

  private void configureBindings() {
    /*joy_drive.a().whileTrue(new RunCommand(() -> Elevator.getInstance().goToElevatorL2(), Elevator.getInstance()));
    joy_drive.b().whileTrue(new RunCommand(() -> Elevator.getInstance().goToAlgaeLow(), Elevator.getInstance()));
    joy_drive.x().whileTrue(new RunCommand(() -> Elevator.getInstance().goToElevatorL4(), Elevator.getInstance()));*/
    joy_op.a().whileTrue(Desalguificador.getInstance().pruebaBrazo());
    joy_op.b().whileTrue(Desalguificador.getInstance().pruebaAlgas());
    joy_op.x().whileTrue(CoralShooter.getInstance().pruebaAlgas());
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
