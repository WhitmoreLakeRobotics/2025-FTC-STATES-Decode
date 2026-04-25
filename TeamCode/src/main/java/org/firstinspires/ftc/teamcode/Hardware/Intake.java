package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Common.CommonLogic;

/**
 * Base class for FTC Team 8492 defined hardware
 */
public class Intake extends BaseHardware{


    /**
     * The {@link #telemetry} field contains an object in which a user may accumulate data which
     * is to be transmitted to the driver station. This data is automatically transmitted to the
     * driver station on a regular, periodic basis.
     */
    public Telemetry telemetry = null;

    /**
     * Hardware Mappings
     */
    public HardwareMap hardwareMap = null; // will be set in Child class
    private TransitionRoller transitionRoller = new TransitionRoller();


    /**
     * BaseHardware constructor
     * <p>
     * The op mode name should be unique. It will be the name displayed on the driver station. If
     * multiple op modes have the same name, only one will be available.
     */
    private DcMotorEx NTKM01;

    //public LED PeaLight;  //off
    //public LED green_PeaLight; //on
    //public LED yellow_Pealight; //transitionroller of intake running

    public Mode CurrentMode;


    private double NTKM01Power;

    public final double minPower = -1.0;
    public final double maxPower = 1.0;

    public static final double stopSpeed = 0;
    public static final double inSpeed = -1;  // use to be 0.5,then 0.75;
    public static final double outSpeed = 0.65;
    public static final double autoSpeed = -1.0;
  //  public static final double snailoutSpeed = -0.25;


    public boolean DriverHappy = false;
    public boolean AtIntakeStop = true;
    public boolean InPain = false;
    public boolean MentallyStable = false;
    public boolean autoStopped = false;

    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime timerun = new ElapsedTime();

    //private double targRange = 10.2; //in cm
    /**
     * User defined init method
     * <p>
     * This method will be called once when the INIT button is pressed.
     */
    public void init() {


        NTKM01 = hardwareMap.get(DcMotorEx.class, "NTKM01");
        //green_PeaLight = hardwareMap.get(LED.class,"green_PeaLight");
        //yellow_PeaLight = hardwareMap.get(LED.class,"yellow_PeaLight");

        //cmdRED();

    }

    /**
     * User defined init_loop method
     * <p>
     * This method will be called repeatedly when the INIT button is pressed.
     * This method is optional. By default this method takes no action.
     */
    public void init_loop(){

    }

    /**
     * User defined start method.
     * <p>
     * This method will be called once when the PLAY button is first pressed.
     * This method is optional. By default this method takes not action.
     * Example usage: Starting another thread.
     */
    public void start(){
        runtime.reset();
    }

    /**
     * User defined loop method
     * <p>
     * This method will be called repeatedly in a loop while this op mode is running
     */
    public void loop(){
/*
        if(CurrentMode == Mode.NTKforward){
            AtIntakeStop = false;

        }else{
            AtIntakeStop = true;
        }

        if(CurrentMode == Mode.NTKbackward){
            AtIntakeStop = false;

        }else{
            AtIntakeStop = true;
        }
*/


        /*if((CommonLogic.inRange(getMotorRPM(NTKM01), 600, 600))){
            InPain = true;
        }else{
            InPain = false;
        }
*/
        if(runtime.milliseconds() >= 1000){
            MentallyStable = true;
        }else{
            MentallyStable = false;
        }
            //possibly put 400 time with sensortime for less glitching if glitching occurs.
            //if(sensorTime.milliseconds() >= 400) {
            //}


            //loopTime.reset();



           /*
        if((transitionRoller.CurrentMode == TransitionRoller.Mode.Spin) && (CurrentMode == Mode.NTKforward) && (timerun.milliseconds() >= 250)){
           cmdGREEN();
           timerun.reset();
        }else

        if((transitionRoller.CurrentMode == TransitionRoller.Mode.Stop) && (CurrentMode == Mode.NTKforward) && (timerun.milliseconds() >= 250)){
           cmdYELLOW();
           timerun.reset();
        }else

        if((transitionRoller.CurrentMode == TransitionRoller.Mode.Back) && (CurrentMode == Mode.NTKforward) && (timerun.milliseconds() >= 250)){
            cmdGREEN();
            timerun.reset();
        }else

        if((transitionRoller.CurrentMode == TransitionRoller.Mode.Spin) && (CurrentMode == Mode.NTKbackward) && (timerun.milliseconds() >= 250)){
            cmdGREEN();
            timerun.reset();
        }else

        if((transitionRoller.CurrentMode == TransitionRoller.Mode.Stop) && (CurrentMode == Mode.NTKbackward) && (timerun.milliseconds() >= 250)){
            cmdYELLOW();
            timerun.reset();
        }else

        if((transitionRoller.CurrentMode == TransitionRoller.Mode.Back) && (CurrentMode == Mode.NTKbackward) && (timerun.milliseconds() >= 250)){
            cmdGREEN();
            timerun.reset();
        }else if((transitionRoller.CurrentMode == TransitionRoller.Mode.Stop) && (CurrentMode == Mode.NTKstop) && (timerun.milliseconds() >= 250)){
            cmdRED();
            timerun.reset();
        }

            */



        /*
        if(DriverHappy){
            cmdPURPLE();
            DriverHappy = false;
        }
         */




/*

        if(transitionRoller.CurrentMode == TransitionRoller.Mode.Spin &&
        CurrentMode == Mode.NTKforward){

        DriverHappy = true;
        }else {
                DriverHappy = false;

        }

 */


    }

    /**
     * User defined stop method
     * <p>
     * This method will be called when this op mode is first disabled
     * <p>
     * The stop method is optional. By default this method takes no action.
     */
    void stop(){

    }

    public void cmdBackward(){
        CurrentMode = Mode.NTKbackward;
        NTKM01.setPower (outSpeed);
        //cmdGREEN();
        //PeaLight.setPosition(Green);
        //PeaLight.enableLight(false);
    }
    public void cmdFoward(){
        CurrentMode = Mode.NTKforward;
        NTKM01.setPower (inSpeed);
        runtime.reset();
        autoStopped = false;
        //sensorTime.reset();
        //cmdGREEN();
        //PeaLight.setPosition(Green);
        //PeaLight.enableLight(false);
    }

    public void cmdStop(){
        CurrentMode = Mode.NTKstop;
        NTKM01.setPower (stopSpeed);
        //cmdRED(); //line not needed when finished
        runtime.reset();
        //PeaLight.enableLight(true);

    }

    public void cmdAutoFoward(){
        CurrentMode = Mode.NTKautoIn;
        NTKM01.setPower (autoSpeed);
        //runtime.reset();
        //PeaLight.enableLight(false);

    }

    public enum Mode {
        NTKstop,
        NTKforward,
        NTKautoIn,
        NTKbackward
    }

    public double getMotorRPM(DcMotorEx motor){
        double ticksPerRevolution = 28; //update and double check
        double gearRatio = 1.0; //update and double check
        double ticksPerSecond = motor.getVelocity();
        return (ticksPerSecond / ticksPerRevolution) * 60 * gearRatio;
    }








}
