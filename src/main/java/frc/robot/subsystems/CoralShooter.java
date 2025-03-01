// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import au.grapplerobotics.LaserCan;

import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CoralConstants;

public class CoralShooter extends SubsystemBase {
  //El sensor la
  private static CoralShooter m_instance;

  private SparkMax m_leftMotor = new SparkMax(CoralConstants.k_leftMotor, MotorType.kBrushless);
  private SparkMaxConfig m_rightConfig = new SparkMaxConfig();
  private SparkMax m_rightMotor = new SparkMax(CoralConstants.k_rightMotor, MotorType.kBrushless);
  private SparkMaxConfig m_leftConfig = new SparkMaxConfig();
  private LaserCan m_laserSensor = new LaserCan(21);
  

  
  private DigitalInput digitalSensor = new DigitalInput(9);

  public CoralShooter() {
    m_rightConfig
      .smartCurrentLimit(40)
      .closedLoopRampRate(0.3)
      .idleMode(IdleMode.kBrake)
      .inverted(false);

    m_leftConfig
      .smartCurrentLimit(40)
      .closedLoopRampRate(0.3)
      .idleMode(IdleMode.kBrake)
      .follow(CoralConstants.k_rightMotor, true);
    
    
      
    
    m_rightMotor.configure(m_rightConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    m_leftMotor.configure(m_leftConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    
  }

  public void setSpeed(double speed){
    m_rightMotor.set(speed);
  }

  public Command pruebaCoral(){
    return runEnd(
      ()->{
        if (digitalSensor.get() || m_laserSensor.getMeasurement().distance_mm < 60) {
          
          m_rightMotor.set(0.2);
        } else {
          m_rightMotor.set(0);
        }

      }
      ,()->{
        m_rightMotor.set(0);
      });
  }

  public Command pruebaTirar(){
    return runEnd(
      ()->{
        m_rightMotor.set(0.4);
      }
      ,()->{
        m_rightMotor.set(0);
      });
  }



  public static CoralShooter getInstance() {
    if (m_instance == null) {
      m_instance = new CoralShooter();
    }
    return m_instance;
  }

  public double getLaser(){
    //regresa la distancia, la mamadota de codigo es por si falla en medir
    LaserCan.Measurement measurement = m_laserSensor.getMeasurement();
    if (measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
      return measurement.distance_mm;
    } 
    return 250;
  }

  public boolean getSensor(){
    //cuidado, verdadero para el sensor es no detectar nada
    return digitalSensor.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
    SmartDashboard.putBoolean("sensor", digitalSensor.get());
    SmartDashboard.putNumber("LaserCAN", m_laserSensor.getMeasurement().distance_mm);
  }
}
