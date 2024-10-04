package org.firstinspires.ftc.teamcode.subsystems.Vertex;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class LinearHorizontal {

    public int targetPosition = 0;
    public static double p = 0.008, i = 0, d = 0.001, f = 0.000001;

    private int floorPos = 10, lowPos = 1600, midPos = 2900, highPos = 3400;

    private boolean /*autoConditionals*/ autoFloorActivator, autoLowActivator, autoMidActivator, autoHighActivator;
    public boolean autoCorrectionSwitch = false;
    public boolean start;

    private final DcMotorEx motor;
    int position;
    public double power;

    private final Encoder encoderElevador;



    public LinearHorizontal(HardwareMap hardwareMap) {
        this.motor = hardwareMap.get(DcMotorEx.class, "horizontal");

        this.power = motor.getPower();

        this.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.motor.setDirection(DcMotorSimple.Direction.REVERSE);

        this.encoderElevador = new OverflowEncoder(new RawEncoder(this.motor));

        this.position = motor.getCurrentPosition();
        this.targetPosition = this.position;

    }
    public void reset() {
        this.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }


    public void PIDF() {

        // Battery adaptation

        // FeedForward
        double ff = targetPosition  * f;
        //Controller
        PIDController controller = new PIDController(p, i, d);
        controller.setPID(p, i, d);
        //Start Correction
        double pid = controller.calculate(this.getPosition(), targetPosition);
        motor.setPower(pid + ff);
    }

    public void upSetPoint() {
        this.targetPosition += 30;
    }
    public void downSetPoint() {
        this.targetPosition -= 30;
    }

    public void setPower(double power){
        motor.setPower(power);
    }


    /*===========================*\
                GETTERS
     /*===========================*/
    public int getPosition() { return this.position = motor.getCurrentPosition(); }

}
