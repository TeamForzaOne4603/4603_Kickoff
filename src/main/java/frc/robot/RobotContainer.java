// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

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

  private final CoralIntake IntakeCmd = new CoralIntake();
  private final DriveCmd driveCmd = new DriveCmd(joy_drive, joy_drive.leftTrigger());
  

  SendableChooser<Command> m_chooser = new SendableChooser<>();
  private final SendableChooser<Command> autoChooser;

  private Command avanzar = Commands.none();
  private Command lados = new Anotar(NewElevatorConstants.kL4Height ).andThen(new IsInPositionElevator());

  //private Command anotarL1 = new Anotar(NewElevatorConstants.kStowHeight);
  private final Command anotarL2 = new Anotar(NewElevatorConstants.kL2Height);
  private final Command anotarL22 = new Anotar(NewElevatorConstants.kL2Height);
  private final Command anotarL3 = new Anotar(NewElevatorConstants.kL3Height);
  private final Command anotarL4 = new Anotar(NewElevatorConstants.kL4Height);
  private final Command checkPosition = new IsInPositionElevator();

  public RobotContainer() {
    DriveTrain.getInstance().setDefaultCommand(driveCmd);
    LedLights.getInstance().setDefaultCommand(LedLights.getInstance().run(()->{LedLights.getInstance().LEdCommand(joy_op);}));
    configureBindings();

    NamedCommands.registerCommand("Intake", IntakeCmd);   
    NamedCommands.registerCommand("L2", anotarL2);    NamedCommands.registerCommand("L3", anotarL3); NamedCommands.registerCommand("L4", anotarL4);
    NamedCommands.registerCommand("L22", anotarL22);
    NamedCommands.registerCommand("L3", anotarL3);
    NamedCommands.registerCommand("Check", checkPosition);
    NamedCommands.registerCommand("BajarAlga", Climber.getInstance().GoToPosition(-7.3));
    NamedCommands.registerCommand("SubirAlga", Climber.getInstance().GoToPosition(-0.2));


    m_chooser.setDefaultOption("Centro", avanzar);
    m_chooser.addOption("L4", lados);
    m_chooser.addOption("Ir recto", avanzar);
    m_chooser.addOption("Path Planner", Commands.none());
    SmartDashboard.putData(m_chooser);

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
    
  }

  private void configureBindings() {

    joy_drive.rightTrigger().whileTrue(CoralShooter.getInstance().shootPosition()).onFalse(NewElevator.getInstance().goToBeggining());
    joy_drive.a().whileTrue(CoralShooter.getInstance().Commandspeed(-0.1));
    joy_drive.leftTrigger().whileTrue(Climber.getInstance().Spool(-0.7));
    joy_drive.leftBumper().whileTrue(Climber.getInstance().Spool(0.7));

    joy_op.leftStick().onTrue(IntakeCmd);
  //  joy_op.rightTrigger().whileTrue(Climber.getInstance().Spool(0.7));
    joy_op.b().onTrue(NewElevator.getInstance().goToPosition(NewElevatorConstants.kL3Height));
    joy_op.y().onTrue(NewElevator.getInstance().goToPosition(NewElevatorConstants.kL4Height));
    joy_op.a().onTrue(NewElevator.getInstance().goToPosition(NewElevatorConstants.kStowHeight));
    joy_op.x().onTrue(NewElevator.getInstance().goToPosition(NewElevatorConstants.kL2Height));
    joy_op.leftBumper().onTrue(Climber.getInstance().GoToPosition(-7.3));
    joy_op.rightBumper().onTrue(Climber.getInstance().GoToPosition(-0.2));
    joy_op.rightTrigger().whileTrue(Climber.getInstance().brazo(0.2));
    joy_op.leftTrigger().whileTrue(Climber.getInstance().brazo(-0.2));



    joy_op.povUp().whileTrue(NewElevator.getInstance().manualMove(0.3));
    joy_op.povDown().whileTrue(NewElevator.getInstance().manualMove(-0.3));

    joy_Prueba.x().whileTrue(DriveTrain.getInstance().stop());

  }

  
  
  public Command getAutonomousCommand() {
    //if(m_chooser.getSelected().getName() == "Path Planner"){
      return autoChooser.getSelected();}// else{return m_chooser.getSelected();  }
      //}
     // return m_chooser.getSelected();}
}
