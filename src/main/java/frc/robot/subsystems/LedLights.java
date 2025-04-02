// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class LedLights extends SubsystemBase {
  /** Creates a new LedLights. */
  private static LedLights mInstance;

  public static LedLights getInstance() {
    if (mInstance == null) {
      mInstance = new LedLights();
    }
    return mInstance;
  }

  private Spark leds = new Spark(9);


  public LedLights() {
  }

  @Override
  public void periodic() {
  }

  public void LEdCommand(CommandXboxController control){
    if (CoralShooter.getInstance().getColor()) {
        control.setRumble(RumbleType.kBothRumble, 0.9);
        leds.set(0.48);
    } else if(!CoralShooter.getInstance().getColor())
    leds.set(0.61);
    control.setRumble(RumbleType.kBothRumble, 0);
  }
}
