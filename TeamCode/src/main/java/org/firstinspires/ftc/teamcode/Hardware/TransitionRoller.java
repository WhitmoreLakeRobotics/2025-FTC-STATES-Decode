package org.firstinspires.ftc.teamcode.Hardware;

import android.transition.Transition;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Common.CommonLogic;


/**
 * Base class for FTC Team 8492 defined hardware
 */
public class TransitionRoller extends BaseHardware{



    /**
     * The {@link #telemetry} field contains an object in which a user may accumulate data which
     * is to be transmitted to the driver station. This data is automatically transmitted to the
     * driver station on a regular, periodic basis.
     */

    public Mode CurrentMode;

    public double getMaxPower() {
        return maxPower;
    }

    private DcMotorEx TRM01;
    private double TRPower;
    public Telemetry telemetry = null;

    public final double minPower = -1.0;
    public final double maxPower = 1.0;

    public static final double TRSpeed = 1.0; //0.85;
    public static final double TRSpeedFar = 0.80;
    public static final double stopSpeed = 0.0;
    static final double TRBack = -0.5;
    private ElapsedTime runtime = new ElapsedTime();
    public double TRSpeedCurrent = TRSpeed;
   // public boolean bSpin = false;

    /**
     * Hardware Mappings
     */
    public HardwareMap hardwareMap = null; // will be set in Child class

    // Voltage Sensor
    private VoltageSensor Raichu;



    /**
     * BaseHardware constructor
     * <p>
     * The op mode name should be unique. It will be the name displayed on the driver station. If
     * multiple op modes have the same name, only one will be available.
     */
    public TransitionRoller() {

    }

    /**
     * User defined init method
     * <p>
     * This method will be called once when the INIT button is pressed.
     */
    public void init(){

        TRM01 = hardwareMap.get(DcMotorEx.class,"TRM01");

        // Find a valid voltage sensor
        for (VoltageSensor sensor : hardwareMap.getAll(VoltageSensor.class)) {
            if (sensor.getVoltage() > 0) {
                Raichu = sensor;
                break;
            }
        }
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

    }

    /**
     * User defined loop method
     * <p>
     * This method will be called repeatedly in a loop while this op mode is running
     */
    public void loop(){
/*
         if (CurrentMode == Mode.Spin) {
             if ((CommonLogic.inRange(getMotorRPM(TRM01), 1100, 1100))) {
                 if (runtime.milliseconds() >= 1000) {
                     cmdStop();
                 }
             }
         }

 */
        if(CurrentMode == Mode.Spin){
            setCompensatedPower(TRSpeedCurrent);

        }


    }
    void stop (){

    }
    /**
     * User defined stop method
     * <p>
     * This method will be called when this op mode is first disabled
     * <p>
     * The stop method is optional. By default this method takes no action.
     */
    public void cmdStop(){
        CurrentMode = Mode.Stop;
        setCompensatedPower(stopSpeed);
    }

    public void cmdSpin() {
        CurrentMode = Mode.Spin;
       // setCompensatedPower(TRSpeedCurrent);
       // bSpin = true;
        runtime.reset();
    }

    public void cmdBack() {
        CurrentMode = Mode.Back;
        setCompensatedPower(TRBack);
    }

    public enum Mode {
        Spin,
        Back,
        Stop;
    }

    public double getMotorRPM(DcMotorEx motor){
        double ticksPerRevolution = 28; //update and double check
        double gearRatio = 1.0; //update and double check
        double ticksPerSecond = motor.getVelocity();
        return (ticksPerSecond / ticksPerRevolution) * 60 * gearRatio;
    }


    /**
     * Voltage Compensation (Limited to 11V)
     */
    private double getVoltage() {
        if (Raichu == null) return 11.0;
        return Raichu.getVoltage();
    }

    private void setCompensatedPower(double targetPower) {

        double measuredVoltage = getVoltage();

        // Cap voltage at 11V max
        double limitedVoltage = Math.min(measuredVoltage, 11.0);

        // Compensation formula
        double compensated = targetPower * (9.0 / limitedVoltage);// was 11.0

        // Clamp to [-1, 1]
        compensated = Math.max(-1.0, Math.min(1.0, compensated));

        TRPower = compensated;
        TRM01.setPower(compensated);


    }
}
