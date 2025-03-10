// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import au.grapplerobotics.LaserCan;

import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CoralConstants;

public class CoralShooter extends SubsystemBase {
  private static CoralShooter m_instance;
  public static CoralShooter getInstance() {
    if (m_instance == null) {
      m_instance = new CoralShooter();
    }
    return m_instance;
  }

  private SparkMax m_leftMotor = new SparkMax(CoralConstants.k_leftMotor, MotorType.kBrushless);
  private SparkMaxConfig m_rightConfig = new SparkMaxConfig();
  private SparkMax m_rightMotor = new SparkMax(CoralConstants.k_rightMotor, MotorType.kBrushless);
  private SparkMaxConfig m_leftConfig = new SparkMaxConfig();
  private LaserCan m_laserSensor = new LaserCan(CoralConstants.k_laserCAN);
  private ColorSensorV3 m_colorSensorV3 = new ColorSensorV3(I2C.Port.kOnboard);  

  public CoralShooter() {
    
    //Motor Config
    m_rightConfig
      .smartCurrentLimit(40)
      .closedLoopRampRate(0.3)
      .idleMode(IdleMode.kBrake)
      .inverted(true);

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

  public Command shoot(double speed){
    return runEnd(
      ()->{
        m_rightMotor.set(speed);
      }
      ,()->{
        m_rightMotor.set(0);
      });
  }

 public double getLaser(){
    LaserCan.Measurement measurement = m_laserSensor.getMeasurement();
    if (measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
      return measurement.distance_mm;
    }
    return 250;
  }

  public boolean getColor(){
    if (m_colorSensorV3.getBlue() > 500 || m_colorSensorV3.getRed()> 500 || m_colorSensorV3.getGreen() > 600) {
      return true;
    } else return false ;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("sensor", getColor());
    SmartDashboard.putNumber("LaserCan", getLaser());
  }
}
