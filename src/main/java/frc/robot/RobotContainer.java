// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.CoralIntake;
import frc.robot.commands.groundAlgue;
import frc.robot.subsystems.CoralShooter;
import frc.robot.subsystems.Desalguificador;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Elevator;

public class RobotContainer {
  
  private final CommandXboxController joy_drive = new CommandXboxController(1);
  private final CommandXboxController joy_op = new CommandXboxController(2);
  private final CommandXboxController joy_Elevador = new CommandXboxController(3);
  private final DriveTrain tankDrive = new DriveTrain();
  private final CoralIntake comandoCoral = new CoralIntake();
  private final Desalguificador algas = Desalguificador.getInstance();
  

  public RobotContainer() {
    tankDrive.setDefaultCommand(new RunCommand(() -> tankDrive.controlledDrive(joy_drive.getLeftY(),joy_drive.getRightX()), tankDrive));
    configureBindings();
  }

  private void configureBindings() {
    joy_Elevador.a().whileTrue(new RunCommand(() -> Elevator.getInstance().goToElevatorL2(), Elevator.getInstance()));
    joy_Elevador.y().whileTrue(new RunCommand(() -> Elevator.getInstance().goToElevatorL3(), Elevator.getInstance()));
    joy_Elevador.b().whileTrue(new RunCommand(() -> Elevator.getInstance().goToElevatorStow(), Elevator.getInstance()));
    joy_Elevador.x().whileTrue(new RunCommand(() -> Elevator.getInstance().goToElevatorL4(), Elevator.getInstance()));
    joy_Elevador.rightBumper().whileTrue(new RunCommand(() -> Elevator.getInstance().goToAlgaeHigh(), Elevator.getInstance()));
    joy_op.b().whileTrue(Desalguificador.getInstance().pruebaAlgas());
    joy_op.x().onTrue(comandoCoral);
    joy_op.a().whileTrue(CoralShooter.getInstance().pruebaTirar());
    joy_drive.b().whileTrue(algas.pruebaAlgas());
    joy_drive.y().whileTrue(algas.pruebaBrazo(90)).toggleOnFalse(algas.pruebaBrazo(0));
    joy_op.rightBumper().whileTrue(new groundAlgue(90));
   /*  joy_Elevador.a().whileTrue(Elevator.getInstance().goUp());
    joy_Elevador.b().whileTrue(Elevator.getInstance().goDown());*/
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");  }
}
