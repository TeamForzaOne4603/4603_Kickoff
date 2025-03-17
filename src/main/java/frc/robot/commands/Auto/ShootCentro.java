// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.NewElevatorConstants;
import frc.robot.commands.CoralIntake;
import frc.robot.subsystems.CoralShooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootCentro extends SequentialCommandGroup {
  /** Creates a new ShootCentro. */
  public ShootCentro() {
    
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());0
    addCommands(new Recto(1.74), CoralShooter.getInstance().shoot(0.2));//new CoralIntake(), new Anotar(NewElevatorConstants.kStowHeight), new Recto(1));
  }
}
