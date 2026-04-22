package org.firstinspires.ftc.teamcode.Hardware;


import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;


import org.firstinspires.ftc.teamcode.Common.CommonLogic;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;


import java.util.function.Supplier;


public class Robot extends BaseHardware {


    private static final Logger log = LoggerFactory.getLogger(Robot.class);


    public DriveTrain driveTrain = new DriveTrain();
    //public Lighting lighting = new Lighting();
    public Sensors sensors = new Sensors();


    public AutoRPM autoRPM;


    public Intake intake = new Intake();
    public Launcher launcher = new Launcher();
    public Uppies uppies = new Uppies();
    public TransitionRoller transitionRoller = new TransitionRoller();
    public LauncherBlocker launcherBlocker = new LauncherBlocker();
    public Limey limey = new Limey();


    private Follower follower;
    public static Pose startingPose;
    private boolean automatedDrive;
    private Supplier<PathChain> pathChain;
    private TelemetryManager telemetryM;
    private boolean slowMode = false;
    private double slowModeMultiplier = 0.5;
    public boolean bCkSenors = false;


    //auto align constants
    public double minTargetVertPos = 65;
    public double minTargetDist = 26;
    public double maxTargetVertPos = 169;
    public double maxTargetDist = 78;


    public double nominalTagWidthRatio = 0.95;
    public double nominalTagAngle = 0;
    public double extremeTagWidthRatio = 0.6829;
    public double extremeTagAngle = 75;
    public double tagExtremeRightPos = 296;
    public double tagExtremeRightAngle = 65;
    public double targetPointFromTag = 12;


    @Override
    public void init() {


        // drivetrain
        driveTrain.hardwareMap = this.hardwareMap;
        driveTrain.telemetry = this.telemetry;
        driveTrain.init();


        //sensors
        sensors.hardwareMap = this.hardwareMap;
        sensors.telemetry = this.telemetry;
        sensors.init();


        // intake
        intake.hardwareMap = this.hardwareMap;
        intake.telemetry = this.telemetry;
        intake.init();


        // launcher
        launcher.hardwareMap = this.hardwareMap;
        launcher.telemetry = this.telemetry;
        launcher.init();


        // launcher blocker
        launcherBlocker.hardwareMap = this.hardwareMap;
        launcherBlocker.telemetry = this.telemetry;
        launcherBlocker.init();


        // transition roller
        transitionRoller.hardwareMap = this.hardwareMap;
        transitionRoller.telemetry = this.telemetry;
        transitionRoller.init();


        // limey
        limey.hardwareMap = this.hardwareMap;
        limey.telemetry = this.telemetry;
        limey.setTelemetry(telemetry);
        limey.init();


        // uppies
        uppies.hardwareMap = this.hardwareMap;
        uppies.telemetry = this.telemetry;
        uppies.init();


        // autoRPM
        autoRPM = new AutoRPM();
        autoRPM.hardwareMap = this.hardwareMap;
        autoRPM.telemetry = this.telemetry;
        autoRPM.init();
    }


    @Override
    public void init_loop() {
        driveTrain.init_loop();
        sensors.init_loop();
        intake.init_loop();
        limey.init_loop();
        autoRPM.init_loop();
        launcher.init_loop();
        launcherBlocker.init_loop();
        transitionRoller.init_loop();
        uppies.init_loop();
    }


    @Override
    public void start() {
        driveTrain.start();
        sensors.start();
        intake.start();
        limey.start();
        autoRPM.start();
        launcher.start();
        launcherBlocker.start();
        transitionRoller.start();
        uppies.start();
    }


    @Override
    public void loop() {
        driveTrain.loop();
        sensors.loop();
        intake.loop();
        limey.loop();
        autoRPM.setDistance(limey.getTagDistance());
        autoRPM.loop();
        launcher.setTargetRPMs(autoRPM.getRPMs());
        launcher.loop();
        launcherBlocker.loop();
        transitionRoller.loop();
        uppies.loop();


        basicSystem();


    }


    public void autonLoop() {
        sensors.loop();
        intake.loop();
        limey.loop();
        autoRPM.loop();
        launcher.loop();
        launcherBlocker.loop();
        transitionRoller.loop();
        uppies.loop();


        basicSystem();


    }


    @Override
    public void stop() {
        driveTrain.stop();
        sensors.stop();
        intake.stop();
        autoRPM.stop();
        launcher.stop();
        launcherBlocker.stop();
        transitionRoller.stop();
        limey.stop();
        uppies.stop();
    }
    public double targetDistanceCalc() {
        double targetOffsetAngle_Vertical = limey.getTy();
        double limelightMountAngleDegrees = 14.5;
        double limelightLensHeightInches = 14.0;
        double goalHeightInches = 29.5;


        double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
        double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);


        double distanceFromLimelightToGoalInches =
                (goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians);


        return distanceFromLimelightToGoalInches;
    }


    public double targetAngleCalc() {


        if (launcher.CurrentPosition == Launcher.Position.LaunchFar || launcher.CurrentCalcPos == Launcher.CalcPos.Far) {


            double currentTagId = limey.getTagID();
            if (currentTagId != -1) {
                double targetOffsetAngle_Horizontal = limey.getTx();


                double tagAngle = limey.getTagAngle() + 90;
                double targetDistanceCalc = targetDistanceCalc();
                double hypotenuse = Math.sqrt((targetDistanceCalc * targetDistanceCalc)
                        + (targetPointFromTag * targetPointFromTag)
                        - (2 * (targetDistanceCalc * targetPointFromTag
                        * Math.cos(Math.toRadians(tagAngle)))));


                double compensationAngle = 180 - tagAngle
                        - Math.toDegrees(Math.asin(Math.sin(Math.toRadians(tagAngle))
                        * targetDistanceCalc) / hypotenuse);


                if (currentTagId == 24) {
                    return driveTrain.getCurrentHeading() + targetOffsetAngle_Horizontal;
                } else if (currentTagId == 20) {
                    return driveTrain.getCurrentHeading() + targetOffsetAngle_Horizontal - 5;
                } else {
                    return driveTrain.getCurrentHeading();
                }
            } else {
                double defaultAngle = 25;
                if (driveTrain.getCurrentHeading() >= 90) return defaultAngle;
                if (driveTrain.getCurrentHeading() <= -90) return -defaultAngle;
            }


        } else {


            double currentTagId = limey.getTagID();
            if (currentTagId != -1) {
                double targetOffsetAngle_Horizontal = limey.getTx();


                double tagAngle = limey.getTagAngle() + 90;
                double targetDistanceCalc = targetDistanceCalc();
                double hypotenuse = Math.sqrt((targetDistanceCalc * targetDistanceCalc)
                        + (targetPointFromTag * targetPointFromTag)
                        - (2 * (targetDistanceCalc * targetPointFromTag
                        * Math.cos(Math.toRadians(tagAngle)))));


                double compensationAngle = 180 - tagAngle
                        - Math.toDegrees(Math.asin(Math.sin(Math.toRadians(tagAngle))
                        * targetDistanceCalc) / hypotenuse);


                if (currentTagId == 24) {
                    return driveTrain.getCurrentHeading() + targetOffsetAngle_Horizontal - 4;
                } else if (currentTagId == 20) {
                    return driveTrain.getCurrentHeading() + targetOffsetAngle_Horizontal;
                } else {
                    return driveTrain.getCurrentHeading();
                }
            } else {
                double defaultAngle = 25;
                if (driveTrain.getCurrentHeading() >= 90) return defaultAngle;
                if (driveTrain.getCurrentHeading() <= -90) return -defaultAngle;
            }
        }


        return driveTrain.getCurrentHeading();
    }


    public void basicSystem(){
        if(autoRPM.Measure){
            launcher.CurrentPosition = Launcher.Position.LaunchCalc;
            if(limey.getTagDistance() < 2){
                launcher.CurrentCalcPos = Launcher.CalcPos.Near;
            }else if(limey.getTagDistance() >= 2){
                launcher.CurrentCalcPos = Launcher.CalcPos.Far;
            }else{
                launcher.CurrentCalcPos = Launcher.CalcPos.Unknown;
            }
        }else if(launcher.CurrentPosition == Launcher.Position.LaunchCalc){
            launcher.CurrentPosition = Launcher.Position.Off;
            launcher.CurrentCalcPos = Launcher.CalcPos.NotCalc;
        }

        if (transitionRoller.CurrentMode == TransitionRoller.Mode.Stop
                && intake.CurrentMode == Intake.Mode.NTKforward) {
            sensors.cmdBLUE();
        }

        if(intake.CurrentMode == Intake.Mode.NTKforward || intake.CurrentMode == Intake.Mode.NTKbackward){
            sensors.cmdGREEN();
        }else{
            sensors.cmdRED();
        }

        if (intake.CurrentMode == Intake.Mode.NTKforward) {
            if ( ((sensors.CurrentDistance2 == Sensors.Distance2.FILLED2
                    && sensors.CurrentDistance3 == Sensors.Distance3.FILLED3)
                    || intake.InPain)
                    && intake.MentallyStable && !launcherBlocker.AtUnBlocked) {
                transitionRoller.cmdStop(); // possibly remove
                intake.cmdStop();
                intake.autoStopped = true;
            }
        }
    }


}