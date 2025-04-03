// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class Constants {
    public static class ChassisConstants{
        public static final int k_leftLeader = 3; 
        public static final int k_leftFollower = 2;
        public static final int k_rightLeader = 5;  
        public static final int k_rightFollower = 4;
        public static final int k_pygeon = 13;

        public static final int k_statorLimit = 160;
        public static final int k_supplyLimit = 60;

        public static final double k_encoderCPR = 2048;
        public static final double k_wheelDiameterMeters = Units.inchesToMeters(6);
        public static final double k_encoderDistancePerPulse = -(Math.PI * k_wheelDiameterMeters) / k_encoderCPR;

        public static final double k_chasssisKP = 2.4;
        public static final double k_chasssisKV = 2.12;//0.1792
        public static final double k_chasssisKS = 0.1792;//0.8281
        public static final double k_chasssisKA = 0.24;
    }

    public static class AlgueConstants{
        public static final int k_AlgueMotor =23;
        public static final int k_WristhMotor = 20;

        public static final double k_position = 24.0;
        public static final int k_currentBrazo = 40;

        public static final double kWristP = 0.42;
    public static final double kWristI = 0.0;
    public static final double kWristD = 0.0;

    public static final double kWristKS = 0.03;
    public static final double kWristKG = 0.4;
    public static final double kWristKV = 0.150;
    public static final double kWristKA = 0.0;

    public static final double kWristOffset = 90;

    public static final double kWristMaxVelocity = 690.0;
    public static final double kWristMaxAcceleration = 1380.0;
    }

    public static class CoralConstants{
        
        public static final  int k_rightMotor = 26;
        public static final  int k_leftMotor = 22;
        public static final int k_laserCAN = 21;
    }

    public static class NewElevatorConstants{
        public static final  int k_rightMotor = 24;
        public static final  int k_leftMotor = 25;

        public static final double k_P = 1.9;
        public static final double k_I = 0;
        public static final double k_D = 0;
        public static final double k_IZone = 17;
        public static final double k_G = 0.25;
        public static final double k_maxVelocity = 100;
        public static final double k_maxAcceleration = 205;
        public static final double k_V = 0.25;
        public static final double k_S = 0.05;

        public static final double kStowHeight = 0.55;
        public static final double kL2Height = 10.6;
        public static final double kL3Height = 35.4;
        public static final double kL4Height = 73.88;
        public static final double kMaxHeight = 74;

        //65%

        //0.7 10.4 71.75.35.4

        public static final double k_encoderCPR = 2048;
        public static final double k_wheelDiameterMeters = Units.inchesToMeters(1+(7/8));
        public static final double k_encoderDistancePerPulse = (Math.PI * k_wheelDiameterMeters) / k_encoderCPR*200;
    }

    public static class ClimberConstants {
        public static final int k_armId = 32;
    }
}
