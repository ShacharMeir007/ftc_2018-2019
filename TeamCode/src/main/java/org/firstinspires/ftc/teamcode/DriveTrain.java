package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;


/**
 * Created by shach on 10/25/2018.
 * interface for class{@link DriveTrain_Interface}
 * This Class describes the driveTrain of the robot
 */
public class DriveTrain implements DriveTrain_Interface {
    private ElapsedTime runtime = new ElapsedTime();
    enum Shape {BALL,CUBE}
    //Constants
    private final int ENCODERS = 1120;
    private final int DIAMETER = 10;
    private final double ENCODERS_PER_CM = ENCODERS /(DIAMETER*Math.PI);
    private final double ENCODERS_PER_DEGREE = 50; //TEMPORARY

    //Motors Objects
    DcMotor leftMotor = null;
    DcMotor rightMotor = null;
    // sensor objects
    ColorSensor colorSensor = null;
    //PID variables
    private static int p1=0;
    private static int p2=0;
    private static double i=0;
    private static double d=0;
    int leftEncorders = 0;
    int rightEncoders=0;

    public void DriveTrain(){}
    /**************************************************************************************
    *Function name: init
    *Input: HardwareMap of an opMode
    *Output: void
    *Operation: initialize the hardware of the drive train
    ***************************************************************************************/
    @Override
    public void init(HardwareMap hwm){
        leftMotor = hwm.get(DcMotor.class, "leftMotor");
        rightMotor = hwm.get(DcMotor.class, "rightMotor");
        colorSensor = hwm.get(ColorSensor.class, "colorSensor");

        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }
    private void t(){}
    /**************************************************************************************
     *Function name: driveForward
     *Input: a power value and a distance value
     *Output: void
     *Operation: drive in the given power the distance given
     ***************************************************************************************/
    public void driveForward(int cm, double power) {
        int target =(int) Math.floor (cm * ENCODERS_PER_CM);
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setTargetPosition(target);
        rightMotor.setTargetPosition(-target);
        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (leftMotor.isBusy()&& rightMotor.isBusy ()){
            leftMotor.setPower(power);
            rightMotor.setPower(-power);

        }
        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }

    /**************************************************************************************
     *Function name: PID_SpeedCalculator
     *Input: a target power
     *Output: a power value after the PID controller's calculation
     *Operation: measures errors of the encoders values and returns a power
     * value accordingly
     ***************************************************************************************/
    private final double kP =0.001;
    private final double kI =0.001;
    private final double kD=0.005;
    private double PID_SpeedCalculator(double targetPower) {
        if (leftMotor.isBusy()&& rightMotor.isBusy ()) {
            double result;
            leftEncorders= leftMotor.getCurrentPosition()-leftEncorders;
            rightEncoders= Math.abs(rightMotor.getCurrentPosition())- rightEncoders;
            p2 =p1;
            p1 = (rightEncoders) - (leftEncorders);
            if (Math.abs(leftMotor.getPower())<0.9) {
                i += p1;
            }
            d= p1-p2;
            result =targetPower+ kP*p1 + kI*i + kD*d;
            if (Math.abs(result)<=1) {
                return result;
            }
            else if (result>0){
                return 1;
            }
            else {
                return -1;
            }

        }
        return 0;
    }
    /**************************************************************************************
     *Function name: resetPID
     *Input: nothing
     *Output: void
     *Operation: resets PID variables
     ***************************************************************************************/
    private void resetPID(){
        p1=0;
        p2=0;
        i=0;
        d=0;
        leftEncorders = 0;
        rightEncoders=0;
    }
    /**************************************************************************************
     *Function name: driveForward_PID
     *Input: a power value and a distance value
     *Output: void
     *Operation: drives forward a given distance in a given power while changing the
     * power of the left motor (using PID)in order to truly drive straight
     ***************************************************************************************/
    void driveForward_PID(int cm, double power) {
        int target =(int) Math.floor (cm * ENCODERS_PER_CM);
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setTargetPosition(target);
        rightMotor.setTargetPosition(-target);
        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (leftMotor.isBusy()&& rightMotor.isBusy ()){

            leftMotor.setPower(PID_SpeedCalculator(power));
            rightMotor.setPower(-power);
        }
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        resetPID();
    }
    /**************************************************************************************
     *Function name: driveRight
     *Input: a power value and a degree value
     *Output: void
     *Operation: turn to the right in the given power the given amount of degrees
     ***************************************************************************************/
    public void driveRight(double power, int degrees) {
        int target =(int) Math.floor (degrees * ENCODERS_PER_DEGREE);
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setTargetPosition(target);
        rightMotor.setTargetPosition(target);
        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (leftMotor.isBusy()&& rightMotor.isBusy ()){
            leftMotor.setPower(power);
            rightMotor.setPower(power);
        }
        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }
    /**************************************************************************************
     *Function name: driveLeft
     *Input: a power value and a degree value
     *Output: void
     *Operation: turn to the left in the given power the given amount of degrees
     ***************************************************************************************/
    public void driveLeft(double power, int degrees) {
        int target =(int) Math.floor (degrees * ENCODERS_PER_DEGREE);
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setTargetPosition(-target);
        rightMotor.setTargetPosition(-target);
        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (leftMotor.isBusy()&& rightMotor.isBusy ()){
            leftMotor.setPower(-power);
            rightMotor.setPower(-power);
        }
        leftMotor.setPower(0);
        rightMotor.setPower(0);
}

    /**
     * this function uses the color sensor to determine the shape
     * in front of the color sensor.
     * @param telemetry the telemetry of the op Mode.
     * @return the shape detected.
     */
    Shape ShapeCheck(Telemetry telemetry){
        colorSensor.enableLed(true);
        telemetry.addData("color value: ", colorSensor.argb());
        telemetry.addData("red: ",colorSensor.red());
        telemetry.addData("green: ",colorSensor.green());
        telemetry.addData("blue: ",colorSensor.blue());
        telemetry.addData("alpha: ",colorSensor.alpha());
        telemetry.addData("argb: ",colorSensor.argb());
        if((double)colorSensor.blue()/colorSensor.red()<0.4){
            telemetry.addData("shape: ","Cube");
            telemetry.update();
            return Shape.CUBE;
        }
        else {
            telemetry.addData("shape: ","Ball");
            telemetry.update();
            return Shape.BALL;
        }

    }


}

