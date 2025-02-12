// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class Constants {
    public static class ChassisConstants{
        public static final int k_leftLeaderId = 4; 
        public static final int k_leftFollowerId = 5;
        public static final int k_rightLeaderId = 2;  
        public static final int k_rightFollowerID = 3;
        public static final int k_pygeonId = 13;

        public static final int k_statorLimit = 140;
        public static final int k_supplyLimit = 65;

        public static final double k_encoderCPR = 2048;
        public static final double k_wheelDiameterMeters = Units.inchesToMeters(6);
        public static final double k_encoderDistancePerPulse = -(Math.PI * k_wheelDiameterMeters) / k_encoderCPR;

        public static final double k_chasssisKP = 1.0474;
        public static final double k_chasssisKV = 0.1392;
        public static final double k_chasssisKS = 1.0281;
        public static final double k_chasssisKA = 0.12516;
    }

    public static class ElevatorConstants{
        public static final  int k_rightMotor = 9;
        public static final  int k_leftMotor = 8;

        public static final double k_elevatorKP = 0.05;
        public static final double k_elevatorKI = 0;
        public static final double k_elevatorKD = 0;
        public static final double k_elevatorKIZone = 15;
        public static final double kG = 0.5;
        public static final double k_maxVelocity = 65;
        public static final double k_maxAcceleration = 200;

        public static final int k_supplyLimit = 40;

        public static final double kStowHeight = 0.0;
        public static final double kL2Height = 9.0;
        public static final double kL3Height = 25.14;
        public static final double kL4Height = 52.0;
        public static final double kMaxHeight = 56.2;
        public static final double kGroundAlgaeHeight = 0.0;
        public static final double kScoreAlgaeHeight = 0.0;
        public static final double kLowAlgaeHeight = 24.8;
        public static final double kHighAlgaeHeight = 42.5;
    }

}
