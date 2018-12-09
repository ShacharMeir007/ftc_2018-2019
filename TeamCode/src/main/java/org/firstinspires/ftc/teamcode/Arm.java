package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 *
 * Created by shach on 12/6/2018.
 */

public class Arm extends ArmStracture {
    DcMotor A1 =null;
    DcMotor A2 =null;
    int upperLim =0;
    int lowerLim =0;
    private DcMotor[] dcMotors;

    public void Arm(int numOfMotors){
        dcMotors = new DcMotor[numOfMotors];
    }

    /**
     * an experiment to define motors in an array
     * @param hwm a {@link HardwareMap} to initialize the {@link DcMotor}Objects
     */
    @Override
    void init(HardwareMap hwm) {
        for (int i = 0; i < dcMotors.length; i++) {
            dcMotors[i] = hwm.get(DcMotor.class, "ArmMotor"+(i+1));
        }
    }
    void setLimits(){

    }
     void MoveWithLimits(double power){
        if (lowerLim<upperLim){
            if (dcMotors[0].getCurrentPosition()<upperLim &&dcMotors[0].getCurrentPosition()>lowerLim)
            for (int i = 0; i <dcMotors.length ; i++) {
                dcMotors[i].setPower(power);
            }
        }
     }
}
