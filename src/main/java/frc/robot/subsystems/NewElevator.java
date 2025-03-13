
package frc.robot.subsystems;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.NewElevatorConstants;

public class NewElevator extends SubsystemBase {
    // Creates a new NewElevator. 
    private static NewElevator mInstance;
    public static NewElevator getInstance() {
      if (mInstance == null) {
        mInstance = new NewElevator();
      }
      return mInstance;
    }
  
    private SparkMax m_leftMotor = new SparkMax(NewElevatorConstants.k_leftMotor, MotorType.kBrushless);
    private SparkMax m_rightMotor = new SparkMax(NewElevatorConstants.k_rightMotor, MotorType.kBrushless);
    private Encoder m_encoder = new Encoder(0, 1);
  
    private final TrapezoidProfile.Constraints m_constraints =
        new TrapezoidProfile.Constraints(NewElevatorConstants.k_maxVelocity, NewElevatorConstants.k_maxAcceleration);
    private final ProfiledPIDController m_controller =
        new ProfiledPIDController(NewElevatorConstants.k_P, NewElevatorConstants.k_I, NewElevatorConstants.k_D, m_constraints, 0.02);
    private final ElevatorFeedforward m_feedforward = new ElevatorFeedforward(NewElevatorConstants.k_S, NewElevatorConstants.k_G, NewElevatorConstants.k_V);
  
    private boolean isInPositionControl = false;
    private double setPoint = 0;
  
    public NewElevator() {
        m_controller.reset(0);
        
      m_encoder.setDistancePerPulse(NewElevatorConstants.k_encoderDistancePerPulse);
      m_encoder.setReverseDirection(false);  
  
      SparkMaxConfig elevatorConfig = new SparkMaxConfig();
      elevatorConfig.smartCurrentLimit(ElevatorConstants.k_supplyLimit);
      elevatorConfig.idleMode(IdleMode.kBrake);
      elevatorConfig.limitSwitch.reverseLimitSwitchEnabled(false);
      m_leftMotor.configure(elevatorConfig, com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  
      // RIGHT ELEVATOR MOTOR
      m_rightMotor.configure(elevatorConfig.follow(m_leftMotor, true), com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
  
    
    @Override
    public void periodic() {
      // This method will be called once per scheduler run
      if (isInPositionControl) {
        m_leftMotor.setVoltage(m_controller.calculate(m_encoder.getDistance(), setPoint) + m_feedforward.calculate(m_controller.getSetpoint().velocity));
      }
      SmartDashboard.putBoolean("IsInPosition", isInPosition());
    }
    
  
    //*Manual Commands
    public Command manualMove(double speed){
    return runEnd(()->{
    isInPositionControl = false;
    m_leftMotor.set(speed);
   }, ()-> {
    m_leftMotor.set(0);
   });}
   
   //Position Commands
   public Command goToPosition (double position){
    return runOnce(()->{
      setPosition(position);
    });}
  
    public void setPosition(double position){
      isInPositionControl = true;
      setPoint = position;
    }

    public double getPosition(){
        return m_encoder.getDistance();
    }

    public double getSetpoint(){return setPoint;}

    public boolean isInPosition(){
        if(m_encoder.getDistance() >= setPoint + 0.3 && m_encoder.getDistance() <= setPoint - 0.3){
            return true;
        } else {
            return false;
        }
    }
    
  }
   
  