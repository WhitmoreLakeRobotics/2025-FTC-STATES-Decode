package org.firstinspires.ftc.teamcode.pedroPathing;

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

public class zBlankppConstants {
    public static FollowerConstants followerConstants = new FollowerConstants()

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

            ;

    public static OTOSConstants localizerConstants =  new OTOSConstants()
            .hardwareMapName("otto")
            .linearUnit(DistanceUnit.INCH)
            .angleUnit(AngleUnit.RADIANS)
            .offset(new SparkFunOTOS.Pose2D(-5.5, -2.0, 0))
            ;


    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);



    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .OTOSLocalizer(localizerConstants)
                .build();
    }
}
