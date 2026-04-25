package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.OTOSConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class orgCompBotConstants {
    public static FollowerConstants followerConstants = new FollowerConstants()
         .forwardZeroPowerAcceleration(31.9247)
        .lateralZeroPowerAcceleration(70.968)
        .translationalPIDFCoefficients(new PIDFCoefficients(0.03,0, 0.01, 0.03))
            .headingPIDFCoefficients(new PIDFCoefficients(1.0,0.003, 0.09,0.025))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.015, 0, 0.0015, 0.6, 0.00001))
            .mass(14.9)
            ;

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(.3) //this should be 1 for tuning
            .rightFrontMotorName("RDM1")
            .rightRearMotorName("RDM2")
            .leftRearMotorName("LDM2")
            .leftFrontMotorName("LDM1")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            //  .xVelocity(79.29855)
            .xVelocity(83.187) //redone 11/30
            //    .yVelocity(63.26871)
            .yVelocity(61.3762)
            ;

    public static OTOSConstants localizerConstants =  new OTOSConstants()
            .hardwareMapName("otto")
            .linearUnit(DistanceUnit.INCH)
            .angleUnit(AngleUnit.RADIANS)
            .offset(new SparkFunOTOS.Pose2D(-5.5, -2.0, 0))
//            .offset(myOffset) 2.25 5.5
            .linearScalar(1.1211) //Multiplier
            .angularScalar(0.9915) ;//Multiplier

    public static PathConstraints pathConstraints = new PathConstraints(
            0.98,
            3.0,
            02.0,
            .98,
            50,
            1,10,1);


    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .OTOSLocalizer(localizerConstants)
                .build();
    }
}
