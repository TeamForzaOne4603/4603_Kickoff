// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class LedLights extends SubsystemBase {
  /** Creates a new LedLights. */
  private Spark leds = new Spark(9);

  private Timer time = new Timer();
  private boolean reset = false;

  public LedLights() {
    time.stop();
    time.reset();
  }

  @Override
  public void periodic() {
   /*  if (CoralShooter.getInstance().getColor()) {
      
      if (!time.isRunning() && reset == false) {
        control.setRumble(RumbleType.kBothRumble, 0.9);
        time.start();
        reset = true;
        leds.set(0.77);
      } 
      if(time.isRunning() && time.get() > 3){
        control.setRumble(RumbleType.kBothRumble, 0.9);
        leds.set(0.48);
      }
    } else if(!CoralShooter.getInstance().getColor())
    leds.set(0.61);
    time.stop();
    time.reset();
    control.setRumble(RumbleType.kBothRumble, 0);
    reset = false;*/
    // This method will be called once per scheduler run
  }

  public void LEdCommand(CommandXboxController control){
    if (CoralShooter.getInstance().getColor()) {
      
      if (!time.isRunning() && reset == false) {
        control.setRumble(RumbleType.kBothRumble, 0.9);
        time.start();
        reset = true;
        leds.set(0.77);
      } 
      if(time.isRunning() && time.get() > 3){
        control.setRumble(RumbleType.kBothRumble, 0.9);
        leds.set(0.48);
      }
    } else if(!CoralShooter.getInstance().getColor())
    leds.set(0.61);
    time.stop();
    time.reset();
    control.setRumble(RumbleType.kBothRumble, 0);
    reset = false;
  }

  public void unrumbleCommand(){
    
  }
}
