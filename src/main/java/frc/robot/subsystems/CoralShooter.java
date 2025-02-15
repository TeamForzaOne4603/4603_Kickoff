// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CoralConstants;

public class CoralShooter extends SubsystemBase {
  private static Desalguificador m_instance;

  private SparkMax m_leftMotor = new SparkMax(CoralConstants.k_leftMotor, MotorType.kBrushless);
  private SparkMaxConfig m_rightConfig = new SparkMaxConfig();
  private SparkMax m_rightMotor = new SparkMax(CoralConstants.k_rightMotor, MotorType.kBrushless);
  private SparkMaxConfig m_leftConfig = new SparkMaxConfig();
  

  public CoralShooter() {
    m_leftConfig
      .smartCurrentLimit(40)
      .closedLoopRampRate(0.3)
      .idleMode(IdleMode.kBrake)
      .inverted(false);
    
    m_rightConfig
      .smartCurrentLimit(40)
      .closedLoopRampRate(0.3)
      .idleMode(IdleMode.kBrake)
      .follow(32, true)
      .inverted(false);
      

    m_leftMotor.configure(m_leftConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    m_rightMotor.configure(m_rightConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);


  }

  public void setSpeed(double speed){
    m_leftMotor.set(speed);
  }

  public Command pruebaCoral(){
    return runEnd(
      ()->{
        setSpeed(0.4);
      }
      ,()->{
        setSpeed(0);
      });
  }

  public static Desalguificador getInstance() {
    if (m_instance == null) {
      m_instance = new Desalguificador();
    }
    return m_instance;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
