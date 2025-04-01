// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.NewElevatorConstants;
import frc.robot.commands.DriveCmd;
import frc.robot.commands.CoralIntake;
import frc.robot.commands.Auto.Anotar;
import frc.robot.commands.Auto.IsInPositionElevator;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CoralShooter;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.LedLights;
import frc.robot.subsystems.NewElevator;

public class RobotContainer {
  
  private final CommandXboxController joy_drive = new CommandXboxController(0);
  private final CommandXboxController joy_op = new CommandXboxController(1);
  private final CommandXboxController joy_Prueba = new CommandXboxController(3);
  private final DriveTrain tankDrive = DriveTrain.getInstance();
  //private final Climber climber = Climber.getInstance();
  private final CoralIntake comandoCoral = new CoralIntake();
  private final DriveCmd comandoDrive = new DriveCmd(joy_drive, joy_drive.leftTrigger());
  //private  final climbing climbCmd = new climbing(joy_drive.a(),joy_drive.b(),joy_drive.x(),joy_drive.y());
  private final LedLights leds = new LedLights();
  

  SendableChooser<Command> m_chooser = new SendableChooser<>();
  private Command avanzar = Commands.none();
  private Command lados = new Anotar(NewElevatorConstants.kL4Height ).andThen(new IsInPositionElevator());

  //private Command anotarL1 = new Anotar(NewElevatorConstants.kStowHeight);
  private final Command anotarL2 = new Anotar(NewElevatorConstants.kL2Height);
  private final Command anotarL22 = new Anotar(NewElevatorConstants.kL2Height);

  private final Command anotarL3 = new Anotar(NewElevatorConstants.kL3Height);
  private Command anotarL4 = new Anotar(NewElevatorConstants.kL4Height);
  private final Command checkPosition = new IsInPositionElevator();
  





  

  public RobotContainer() {
    
    tankDrive.setDefaultCommand(comandoDrive);
    //climber.setDefaultCommand(climbCmd);
    leds.setDefaultCommand(leds.run(()->{leds.LEdCommand(joy_op);}));
    configureBindings();
    NamedCommands.registerCommand("Intake", comandoCoral);   
    NamedCommands.registerCommand("L2", anotarL2);    NamedCommands.registerCommand("L3", anotarL3); NamedCommands.registerCommand("L4", anotarL4);
    NamedCommands.registerCommand("L22", anotarL22);
    NamedCommands.registerCommand("Check", checkPosition);

    m_chooser.setDefaultOption("Centro", avanzar);
    m_chooser.addOption("L4", lados);
    

    final Command PruebaPathPlanner = new PathPlannerAuto("TEST1");

    final Command vuelta = new PathPlannerAuto("TEST 2");
    final Command cervantes = new PathPlannerAuto("TEST 3");
    m_chooser.addOption("Pathplanner", PruebaPathPlanner);
    m_chooser.addOption("Vuelta", vuelta);
    m_chooser.addOption("Cervantes", cervantes);


    m_chooser.addOption("Ir recto", avanzar);
    SmartDashboard.putData(m_chooser);
  }

  private void configureBindings() {

     joy_op.leftStick().onTrue(comandoCoral);
     //joy_op.rightTrigger().whileTrue(CoralShooter.getInstance().shootPosition()).onFalse(NewElevator.getInstance().goToBeggining());
    joy_drive.rightTrigger().whileTrue(CoralShooter.getInstance().shootPosition()).onFalse(NewElevator.getInstance().goToBeggining());
    //joy_op.rightTrigger().whileTrue(Climber.getInstance().brazo(-0.2));
    //joy_op.leftTrigger().whileTrue(Climber.getInstance().brazo(0.2));
    joy_drive.leftTrigger().whileTrue(Climber.getInstance().Spool(-0.7));
    joy_drive.leftBumper().whileTrue(Climber.getInstance().Spool(0.7));
    joy_drive.a().whileTrue(CoralShooter.getInstance().Commandspeed(-0.1));
    joy_op.rightTrigger().whileTrue(Climber.getInstance().Spool(0.7));



    joy_op.b().onTrue(NewElevator.getInstance().goToPosition(NewElevatorConstants.kL3Height));
    joy_op.y().onTrue(NewElevator.getInstance().goToPosition(NewElevatorConstants.kL4Height));
    joy_op.a().onTrue(NewElevator.getInstance().goToPosition(NewElevatorConstants.kStowHeight));
    joy_op.x().onTrue(NewElevator.getInstance().goToPosition(NewElevatorConstants.kL2Height));
    joy_op.leftBumper().onTrue(Climber.getInstance().GoToPosition(-7.3));
    joy_op.rightBumper().onTrue(Climber.getInstance().GoToPosition(-0.2));

    joy_op.povUp().whileTrue(NewElevator.getInstance().manualMove(0.3));
    joy_op.povDown().whileTrue(NewElevator.getInstance().manualMove(-0.3));

    joy_Prueba.x().whileTrue(DriveTrain.getInstance().stop());

  }

  
  
  public Command getAutonomousCommand() {
    return m_chooser.getSelected();  }

    
}
