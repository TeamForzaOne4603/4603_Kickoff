// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.CoralIntake;
import frc.robot.subsystems.Algae;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CoralShooter;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Elevator;

public class RobotContainer {
  
  private final CommandXboxController joy_drive = new CommandXboxController(1);
  private final CommandXboxController joy_op = new CommandXboxController(2);
  private final CommandXboxController joy_Alga = new CommandXboxController(3);
  private final DriveTrain tankDrive = new DriveTrain();
  //private final CoralIntake comandoCoral = new CoralIntake();
  

  public RobotContainer() {
    tankDrive.setDefaultCommand(new RunCommand(() -> tankDrive.controlledDrive(joy_op.getLeftY(),joy_op.getRightX()), tankDrive));
    configureBindings();
  }

  private void configureBindings() {
    //joy_op.a().whileTrue(new RunCommand(() -> Elevator.getInstance().goToElevatorL2(), Elevator.getInstance()));
    //joy_op.y().whileTrue(new RunCommand(() -> Elevator.getInstance().goToElevatorL3(), Elevator.getInstance()));
    //joy_op.b().whileTrue(new RunCommand(() -> Elevator.getInstance().goToElevatorL4(), Elevator.getInstance()));
    //joy_op.x().whileTrue(new RunCommand(() -> Elevator.getInstance().goToElevatorStow(), Elevator.getInstance()));

     //joy_op.leftStick().onTrue(comandoCoral);
   // joy_op.povDown().whileTrue(CoralShooter.getInstance().pruebaTirar());
    joy_op.a().whileTrue(CoralShooter.getInstance().shoot(0.15));
    joy_op.b().whileTrue(CoralShooter.getInstance().shoot(0.3));
    joy_op.x().whileTrue(CoralShooter.getInstance().shoot(0.2));
    joy_op.x().whileTrue(CoralShooter.getInstance().shoot(0.4));
    joy_drive.a().onTrue(Elevator.getInstance().goL2());

    joy_Alga.a().whileTrue(Algae.getInstance().goToPosition(0));
    joy_Alga.b().whileTrue(Algae.getInstance().goToPosition(60));
    joy_Alga.y().whileTrue(Algae.getInstance().goToPosition(100));
    joy_Alga.x().whileTrue(Algae.getInstance().goToPosition(80));
    joy_Alga.leftBumper().whileTrue(Algae.getInstance().take(-1));
    joy_Alga.rightBumper().whileTrue(Algae.getInstance().take(1));
    joy_Alga.rightTrigger().whileTrue(Algae.getInstance().shoot());
    joy_Alga.povUp().whileTrue(Algae.getInstance().reset());

   // joy_op.rightBumper().whileTrue(Elevator.getInstance().goUp());
    //joy_op.leftBumper().whileTrue(Elevator.getInstance().goDown());
    //joy_op.povUp().onTrue(Elevator.getInstance().resetPosition());

    /*joy_op.b().whileTrue(Algae.getInstance().goToPosition(0));
    joy_op.x().whileTrue(Algae.getInstance().goDown());
    joy_op.y().whileFalse(Algae.getInstance().reset());*/
    //joy_drive.a().whileTrue(Climber.getInstance().bajar());
    //joy_drive.b().whileTrue(Climber.getInstance().subir());
   /*  joy_Elevador.a().whileTrue(Elevator.getInstance().goUp());
    joy_Elevador.b().whileTrue(Elevator.getInstance().goDown());*/
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");  }
}
