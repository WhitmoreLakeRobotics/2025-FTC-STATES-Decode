package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PredictiveBrakingCoefficients;
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
//this is for adjusting the competition robot.
// you need to change which constants file the code points to for the different robots.
public class CompBotConstants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            //.predictiveBrakingCoefficients(new PredictiveBrakingCoefficients(0.01,0.017157,.898306))
            .forwardZeroPowerAcceleration(-24.7796)//was 31.9247
            .lateralZeroPowerAcceleration(-46.20165) //was 70.968
            .translationalPIDFCoefficients(new PIDFCoefficients(0.03,0.0, 0.01, 0.03))
            .headingPIDFCoefficients(new PIDFCoefficients(1,0,0,0.011))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.015, 0.0, 0.0015, 0.06, 0.00001))
            .mass(15.0)
            ;

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1) //this should be 1 for tuning
            .rightFrontMotorName("RDM1")
            .rightRearMotorName("RDM2")
            .leftRearMotorName("LDM2")
            .leftFrontMotorName("LDM1")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .xVelocity(64.855)
            .yVelocity(39.0775)
            ;

    public static OTOSConstants localizerConstants =  new OTOSConstants()
            .hardwareMapName("otto")
            .linearUnit(DistanceUnit.INCH)
            .angleUnit(AngleUnit.RADIANS)
            .offset(new SparkFunOTOS.Pose2D(-5.5, -2.0, 0))
            .linearScalar(0.45275)
            //.angularScalar(0.3938925)
            ;


    public static PathConstraints pathConstraints = new PathConstraints(
            0.99,
            100,
            1,
            1);



    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .OTOSLocalizer(localizerConstants)
                .build();
    }
}
