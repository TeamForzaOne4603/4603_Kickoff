// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgueConstants;

public class Desalguificador extends SubsystemBase {
  /** Creates a new Desalguificador. */
  private static Desalguificador m_instance;
  private SparkMax m_algueMotor = new SparkMax(AlgueConstants.k_AlgueMotor, MotorType.kBrushless);
  private SparkMaxConfig m_algueConfig = new SparkMaxConfig();
  private SparkMax m_brazoMotor = new SparkMax(AlgueConstants.k_brazoMotor, MotorType.kBrushless);
  private SparkMaxConfig m_brazoConfig = new SparkMaxConfig();

  private RelativeEncoder m_Encoder = m_brazoMotor.getEncoder();
  private PIDController m_controller = new PIDController(0.15,0,0.0001);

  private double position = 0;

  public Desalguificador() {
    m_algueConfig
      .smartCurrentLimit(40)
      .closedLoopRampRate(0.3)
      .idleMode(IdleMode.kBrake)
      .inverted(false);

    m_brazoConfig
      .smartCurrentLimit(40)
      .closedLoopRampRate(0.3)
      .idleMode(IdleMode.kBrake)
      .inverted(false);

    m_algueMotor.configure(m_algueConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    m_brazoMotor.configure(m_brazoConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    m_Encoder.setPosition(0);
  }



  public void setAlgue(double speed){
    m_algueMotor.set(speed);
  }

  public void setPosition(double Position){
    position = Position;
  }

  public Command pruebaBrazo(){
    return runEnd(
      ()-> {
        m_brazoMotor.set(0.1);
      },
      ()-> {
        m_brazoMotor.set(0);
      }
    );
  }
  public Command pruebaAlgas(){
    return runEnd(
      ()-> {
        m_algueMotor.set(0.1);
      },
      ()-> {
        m_algueMotor.set(0);
      }
    );
  }

  public static Desalguificador getInstance() {
    if (m_instance == null) {
      m_instance = new Desalguificador();
    }
    return m_instance;
  }

  public boolean isInPosition (){
    if (m_Encoder.getPosition() >= position -0.2) {
      return true;
    }
    return false;
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    //m_muñecaMotor.set(m_controller.calculate(m_Encoder.getPosition(), position));
    SmartDashboard.putNumber("Posicion", m_Encoder.getPosition());
  }
}
