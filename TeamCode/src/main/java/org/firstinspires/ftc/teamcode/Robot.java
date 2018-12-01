package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Function;
import org.firstinspires.ftc.robotcore.external.Supplier;

/**
 * Created by shach on 11/14/2018.
 *
 */

public class Robot  {
    public static class Roller{

    }
    private ElapsedTime runtime = new ElapsedTime();
    public DriveTrain driveTrain = new DriveTrain();
    public Robot(){}


    public void init(HardwareMap hwm){
        driveTrain.init(hwm);
    }



    public void run_Autonomous(){

    }
}

