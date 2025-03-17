// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Leds extends Command {
  /** Creates a new Leds. */
   private AddressableLED m_led = new AddressableLED(9);
  private AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(1000);

  private int ledStates = 0;
  private Timer time = new Timer();
  public Leds() {
    // Use addRequirements() here to declare subsystem dependencies.
    time.stop();
    time.reset();
    // PWM port 9
    // Must be a PWM header, not MXP or DIO
    m_led.setLength(m_ledBuffer.getLength());

    // Set the data
    m_led.setData(m_ledBuffer);
    m_led.start();
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    switch (ledStates) {
        case 0:
          time.start();
          LEDPattern.solid(Color.kBlue).applyTo(m_ledBuffer);
          ledStates = 1;
          break;
        case 1:
          if (time.get()>=0.5) {
            time.restart();
            LEDPattern.solid(Color.kRed).applyTo(m_ledBuffer);
            ledStates = 2;
          }
          break;
        case 2:
        if (time.get()>=0.5) {
          time.restart();
          LEDPattern.solid(Color.kBlue).applyTo(m_ledBuffer);
          ledStates = 1;
        }
          break;
        default: ledStates = 0;
          break;
      }
      
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
