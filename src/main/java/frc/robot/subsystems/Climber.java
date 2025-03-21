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

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  private static Climber mInstance;

  public static Climber getInstance() {
    if (mInstance == null) {
      mInstance = new Climber();
    }
    return mInstance;
  }

  private PIDController simplePID = new PIDController(0.1, 0, 0.0001);
  private double setpoint = 101.71684265136719;
  private boolean positionControl = false;

  /** Creates a new Climber. */
  private SparkMax m_leftClimber = new SparkMax(23, MotorType.kBrushless);
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
      tconfig.CurrentLimits.StatorCurrentLimit = 80;

      m_rope.getConfigurator().apply(tconfig);
      m_rope.setNeutralMode(NeutralModeValue.Brake);

      m_Encoder.setPosition(0);


    m_leftClimber.configure(LeftConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  public Command brazo(double speed){
    return runEnd(()-> {m_leftClimber.set(speed);positionControl = false;}, ()-> {m_leftClimber.set(0);});
  }
  

  public Command Spool(double speed){
    return runEnd(()-> {m_rope.set(speed);positionControl = false;}, ()-> {m_rope.set(0);});
  }

  public void spool(double speed){
    m_rope.set(speed);
  }
  public void Brazo(double speed){
    m_rope.set(speed);
  }

  public Command GoToPosition(double position){
    return runOnce(()-> {positionControl = true;setpoint=position;});
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Braazo", m_Encoder.getPosition());

    if (positionControl) {
      m_leftClimber.set(simplePID.calculate(m_Encoder.getPosition(), setpoint));
    }
  }

  public boolean isInPosition(){
    if(m_Encoder.getPosition() <= setpoint + 0.1 && m_Encoder.getPosition() >= setpoint - 0.1){
        return true;
    } else {
        return false;
    }
}
}

