// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.IntFunction;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.NewElevatorConstants;
import frc.robot.commands.DriveCmd;
import frc.robot.commands.CoralIntake;
import frc.robot.commands.climbing;
import frc.robot.commands.Auto.Anotar;
import frc.robot.commands.Auto.IsInPositionElevator;
import frc.robot.commands.Auto.Lados;
import frc.robot.commands.Auto.Recto;
import frc.robot.subsystems.Algae;
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
  private Command avanzar = new Recto(2);
  private Command lados = new Lados(2);

  private Command anotarL1 = new Anotar(NewElevatorConstants.kStowHeight);
  private final Command anotarL2 = new Anotar(NewElevatorConstants.kL2Height);
  private final Command anotarL3 = new Anotar(NewElevatorConstants.kL3Height);
  private Command anotarL4 = new Anotar(NewElevatorConstants.kL4Height);
  private final Command checkPosition = new IsInPositionElevator();
  





  

  public RobotContainer() {
    
    tankDrive.setDefaultCommand(comandoDrive);
    //climber.setDefaultCommand(climbCmd);
    leds.setDefaultCommand(leds.run(()->{leds.LEdCommand(joy_op);}));
    configureBindings();
    NamedCommands.registerCommand("Intake", comandoCoral);   
    NamedCommands.registerCommand("L2", anotarL2);    NamedCommands.registerCommand("L3", anotarL3);

    NamedCommands.registerCommand("Check", checkPosition);



    
    
    final Command Angulo = new frc.robot.commands.Auto.Angulo(45);

    m_chooser.setDefaultOption("Centro", avanzar);
    m_chooser.addOption("Angulo", Angulo);
    m_chooser.addOption("Lados", lados);
    
    final Command PruebaPathPlanner = new PathPlannerAuto("Auto Prueba");
    m_chooser.addOption("Pathplanner", PruebaPathPlanner);
    m_chooser.addOption("Ir recto", avanzar);
    SmartDashboard.putData(m_chooser);
  }

  private void configureBindings() {

     joy_op.leftStick().onTrue(comandoCoral);
    joy_drive.a().whileTrue(CoralShooter.getInstance().shootPosition()).onFalse(NewElevator.getInstance().goToBeggining());
    joy_drive.rightTrigger().whileTrue(Climber.getInstance().brazo(0.2));
    joy_drive.rightBumper().whileTrue(Climber.getInstance().brazo(-0.2));
    joy_drive.leftTrigger().whileTrue(Climber.getInstance().Spool(-0.3));
    joy_drive.leftBumper().whileTrue(Climber.getInstance().Spool(0.3));
    //joy_drive.b().onTrue(climber.GoToPosition());

    joy_op.b().onTrue(NewElevator.getInstance().goToPosition(NewElevatorConstants.kL3Height));
    joy_op.y().onTrue(NewElevator.getInstance().goToPosition(NewElevatorConstants.kL4Height));
    joy_op.a().onTrue(NewElevator.getInstance().goToPosition(NewElevatorConstants.kStowHeight));
    joy_op.x().onTrue(NewElevator.getInstance().goToPosition(NewElevatorConstants.kL2Height));

    //joy_Prueba.rightBumper().onTrue(anotarL2);//anotarL2);
    joy_Prueba.a().whileTrue(NewElevator.getInstance().manualMove(0.3));
    joy_Prueba.b().whileTrue(NewElevator.getInstance().manualMove(-0.3));


  }

  
  
  public Command getAutonomousCommand() {
    return m_chooser.getSelected();  }

    
}
