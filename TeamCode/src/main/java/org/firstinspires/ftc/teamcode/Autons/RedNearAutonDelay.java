package org.firstinspires.ftc.teamcode.Autons;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Common.Settings;
import org.firstinspires.ftc.teamcode.Hardware.Robot;


@Disabled
@Autonomous(name = "RedNearAutonDelay", group = "Auton")
// @Autonomous(...) is the other common choice

public class RedNearAutonDelay extends OpMode {

    //RobotComp robot = new RobotComp();
    Robot robot = new Robot();




    private stage currentStage = stage._unknown;
    // declare auton power variables
    //private double AUTO_DRIVE_TURBO_SPEED = DriveTrain.DRIVETRAIN_TURBOSPEED;
    //private double AUTO_DRIVE_SLOW_SPEED = DriveTrain.DRIVETRAIN_SLOWSPEED;
    // private double AUTO_DRIVE_NORMAL_SPEED = DriveTrain.DRIVETRAIN_NORMALSPEED;
    // private double AUTO_TURN_SPEED = DriveTrain.DRIVETRAIN_TURNSPEED;

    private String RTAG = "8492-Auton";

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();


    //Code to run ONCE when the driver hits INIT

    @Override
    public void init() {
        //----------------------------------------------------------------------------------------------
        // These constants manage the duration we allow for callbacks to user code to run for before
        // such code is considered to be stuck (in an infinite loop, or wherever) and consequently
        // the robot controller application is restarted. They SHOULD NOT be modified except as absolutely
        // necessary as poorly chosen values might inadvertently compromise safety.
        //----------------------------------------------------------------------------------------------
        msStuckDetectInit = Settings.msStuckDetectInit;
        msStuckDetectInitLoop = Settings.msStuckDetectInitLoop;
        msStuckDetectStart = Settings.msStuckDetectStart;
        msStuckDetectLoop = Settings.msStuckDetectLoop;
        msStuckDetectStop = Settings.msStuckDetectStop;

        robot.hardwareMap = hardwareMap;
        robot.telemetry = telemetry;
        robot.init();
        telemetry.addData("Test Auton", "Initialized");

        //Initialize Gyro
        robot.driveTrain.ResetGyro();
    }


    //Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY

    @Override
    public void init_loop() {
        // initialize robot
        robot.init_loop();

    }


    //Code to run ONCE when the driver hits PLAY

    @Override
    public void start() {
        // start robot
        runtime.reset();
        robot.start();
    }


    //Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP

    @Override
    public void loop() {

        telemetry.addData("Auton_Current_Stage ", currentStage);
        robot.loop();

        switch (currentStage){
            case  _unknown:
                currentStage = stage._00_preStart;
                break;
            case _00_preStart:
                currentStage = stage._20_DriveBack;
                break;


            case _20_DriveBack:
                if (robot.driveTrain.getCmdComplete())     {
                    robot.driveTrain.CmdDrive(25,180,0.35,0);
                    robot.launcher.cmdOuttouch();
                    currentStage = stage._25_Turn;
                }

            case _25_Turn:
                if (robot.driveTrain.getCmdComplete())     {
                    robot.driveTrain.cmdTurn(-5,0.25);
                    runtime.reset();
                    currentStage = stage._30_Shoot1;
                }

                break;
            case _30_Shoot1:
                if (runtime.milliseconds() >=20000)  {
                    robot.intake.cmdFoward();
                    robot.transitionRoller.cmdSpin();
                    robot.launcherBlocker.cmdUnBlock();
                    runtime.reset();
                    currentStage = stage._40_Shoot2;
                }
                break;
            case _40_Shoot2:
                if (runtime.milliseconds() >=5000){
                    robot.driveTrain.CmdDrive(0,0,0.0,0);
                    robot.launcher.cmdStop();
                    robot.transitionRoller.cmdStop();
                    robot.launcherBlocker.cmdBlock();
                    robot.intake.cmdStop();
                    currentStage = stage._60_End;
                }


            case _60_End:
                if(robot.driveTrain.getCmdComplete()){
                    robot.stop();


                }







                break;
        }



    }  //  loop


    //Code to run ONCE after the driver hits STOP

    @Override
    public void stop() {
        robot.stop();
    }

    private enum stage {
        _unknown,
        _00_preStart,
        _10_turn,
        _20_DriveBack,
        _25_Turn,
        _30_Shoot1,
        _40_Shoot2,
        _60_End,



    }
}

