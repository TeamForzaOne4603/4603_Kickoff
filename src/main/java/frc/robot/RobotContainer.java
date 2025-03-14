// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.ClimbCmd;
import frc.robot.commands.CoralIntake;
import frc.robot.subsystems.Algae;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CoralShooter;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.NewElevator;

public class RobotContainer {
  
  private final CommandXboxController joy_drive = new CommandXboxController(1);
  private final CommandXboxController joy_op = new CommandXboxController(2);
  private final CommandXboxController joy_Alga = new CommandXboxController(3);
  private final CommandXboxController joy_Elevator = new CommandXboxController(4);
  private final DriveTrain tankDrive = DriveTrain.getInstance();
  private final CoralIntake comandoCoral = new CoralIntake();
  private final ClimbCmd comandoDrive = new ClimbCmd(joy_drive);
  SendableChooser<Command> m_chooser = new SendableChooser<>();
  private Command AutoCentro = new Command() {};
  private Command AutoDerecha = new Command() {};
  private Command AutoIzquierda = new Command() {};
  private Command PruebaPathPlanner = new   PathPlannerAuto("null");




  

  public RobotContainer() {
    tankDrive.setDefaultCommand(comandoDrive);
    //tankDrive.setDefaultCommand(new RunCommand(() -> tankDrive.controlledDrive(-joy_drive.getLeftY(),-joy_drive.getRightX()), tankDrive));
    configureBindings();

    m_chooser.setDefaultOption("Centro", AutoCentro);
    m_chooser.addOption("Derecha", AutoDerecha);
    m_chooser.addOption("Izquierda", AutoIzquierda);
    m_chooser.addOption("Pathplanner", PruebaPathPlanner);
    SmartDashboard.putData(m_chooser);
  }

  private void configureBindings() {

     joy_Elevator.leftStick().onTrue(comandoCoral);
    joy_drive.rightTrigger().whileTrue(CoralShooter.getInstance().shootPosition());
    joy_drive.rightTrigger().whileTrue(CoralShooter.getInstance().shootPosition());

    joy_op.b().whileTrue(CoralShooter.getInstance().shoot(0.3));
    joy_op.x().whileTrue(CoralShooter.getInstance().shoot(0.2));
    joy_op.x().whileTrue(CoralShooter.getInstance().shoot(0.4));

    joy_Alga.a().whileTrue(Algae.getInstance().goToPosition(0));
    joy_Alga.b().whileTrue(Algae.getInstance().goToPosition(60));
    joy_Alga.y().whileTrue(Algae.getInstance().goToPosition(100));
    joy_Alga.x().whileTrue(Algae.getInstance().goToPosition(80));
    joy_Alga.leftBumper().whileTrue(Algae.getInstance().take(-1,60));
    joy_Alga.rightBumper().whileTrue(Algae.getInstance().take(1,100));
    joy_Alga.rightTrigger().whileTrue(Algae.getInstance().shoot());
    joy_Alga.povUp().whileTrue(Algae.getInstance().reset());

    joy_Alga.rightBumper().whileTrue(NewElevator.getInstance().goToPosition(13).alongWith(Algae.getInstance().goDown()));

    joy_Elevator.rightBumper().and(joy_Elevator.x()).whileTrue(NewElevator.getInstance().manualMove(0.3));
    joy_Elevator.rightBumper().and(joy_Elevator.a()).whileTrue(NewElevator.getInstance().manualMove(-0.25));
    joy_Elevator.rightBumper().and(joy_Elevator.b()).whileTrue(CoralShooter.getInstance().shoot(0.2));

    joy_Elevator.leftBumper().and(joy_Elevator.b()).whileTrue(NewElevator.getInstance().goToPosition(35.4));
    joy_Elevator.leftBumper().and(joy_Elevator.y()).whileTrue(NewElevator.getInstance().goToPosition(71.75));
    joy_Elevator.leftBumper().and(joy_Elevator.a()).onTrue(NewElevator.getInstance().goToPosition(0.7));
    joy_Elevator.leftBumper().and(joy_Elevator.x()).onTrue(NewElevator.getInstance().goToPosition(10.4));


    joy_drive.a().whileTrue(Climber.getInstance().brazo(0.3));//bajar brazo
    joy_drive.b().whileTrue(Climber.getInstance().brazo(-0.2));//subir brazo
    joy_drive.x().whileTrue(Climber.getInstance().Spool(-0.2));//subir spool
    joy_drive.y().whileTrue(Climber.getInstance().Spool(0.3)); //bajar spool
  }

  public Command getAutonomousCommand() {
    return m_chooser.getSelected();  }
}
