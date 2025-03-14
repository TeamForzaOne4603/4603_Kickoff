// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
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
import frc.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase {
  private static Climber mInstance;

  public static Climber getInstance() {
    if (mInstance == null) {
      mInstance = new Climber();
    }
    return mInstance;
  }

  /** Creates a new Climber. */
  private SparkMax m_leftClimber = new SparkMax(ClimberConstants.k_armId, MotorType.kBrushless);
  private TalonFX m_rope = new TalonFX(42);
 // private SparkMax m_rightClimber = new SparkMax(8, MotorType.kBrushless);
  private RelativeEncoder m_Encoder = m_leftClimber.getEncoder();

  public Climber() {
    SparkMaxConfig LeftConfig = new SparkMaxConfig();
    TalonFXConfiguration tconfig = new TalonFXConfiguration();
    LeftConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(45)
        .inverted(true);

      tconfig.CurrentLimits.SupplyCurrentLimit = 40;
      tconfig.CurrentLimits.StatorCurrentLimit = 140;

      m_rope.getConfigurator().apply(tconfig);
      m_rope.setNeutralMode(NeutralModeValue.Brake);



    m_leftClimber.configure(LeftConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
   // m_rightClimber.configure(RighConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  public Command brazo(double speed){
    return runEnd(()-> {m_leftClimber.set(speed);}, ()-> {m_leftClimber.set(0);});
  }
  

  public Command Spool(double speed){
    return runEnd(()-> {m_rope.set(speed);}, ()-> {m_rope.set(0);});
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Escalador", m_Encoder.getPosition());
  }
}
