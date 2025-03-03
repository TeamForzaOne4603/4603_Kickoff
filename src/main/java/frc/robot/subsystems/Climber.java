// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgueConstants;

public class Climber extends SubsystemBase {
  private static Climber mInstance;

  public static Climber getInstance() {
    if (mInstance == null) {
      mInstance = new Climber();
    }
    return mInstance;
  }

  /** Creates a new Climber. */
  private SparkMax m_leftClimber = new SparkMax(7, MotorType.kBrushless);
 // private SparkMax m_rightClimber = new SparkMax(8, MotorType.kBrushless);
  private RelativeEncoder m_Encoder = m_leftClimber.getEncoder();

  public Climber() {
    SparkMaxConfig RighConfig = new SparkMaxConfig();
    SparkMaxConfig LeftConfig = new SparkMaxConfig();
    LeftConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(80)
        .inverted(true);

    RighConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(80)
        .inverted(false);

    m_leftClimber.configure(LeftConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
   // m_rightClimber.configure(RighConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  public Command subir(){
    return runEnd(()-> {m_leftClimber.set(0.85);}, ()-> {m_leftClimber.set(0);});
  }
  

  public Command bajar(){
    return runEnd(()-> {m_leftClimber.set(-0.85);}, ()-> {m_leftClimber.set(0);});
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Escalador", m_Encoder.getPosition());
  }
}
