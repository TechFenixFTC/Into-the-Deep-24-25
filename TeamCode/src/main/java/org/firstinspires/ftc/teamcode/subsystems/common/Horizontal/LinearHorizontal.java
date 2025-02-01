package org.firstinspires.ftc.teamcode.subsystems.common.Horizontal;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.HashMap;

@Config
public class LinearHorizontal {
    public Servo servoLinearHorizontal;
    public static boolean monitor = false;
    private double targetPosition = 0;

    public double servoLinearHorizontalPosition;
    public HashMap<LinearHorizontalStates, Double> mapStateHorizontal = new HashMap<>();
    public LinearHorizontalStates linearHorizontalInferiorState = LinearHorizontalStates.RETRACTED;
    public LinearHorizontal(HardwareMap hardwareMap, String hardwareName) {
        this.servoLinearHorizontal = hardwareMap.get(Servo.class, hardwareName);

    }
    public Action goToExtended(){


        return new InstantAction(() -> {
            linearHorizontalInferiorState = LinearHorizontalStates.EXTENDED;
            servoLinearHorizontalPosition = mapStateHorizontal.get(linearHorizontalInferiorState);
            servoLinearHorizontal.setPosition(mapStateHorizontal.get(linearHorizontalInferiorState));

        });
    }
    public Action goToRetracted(){


        return new InstantAction(() -> {
            linearHorizontalInferiorState = LinearHorizontalStates.RETRACTED;
            servoLinearHorizontalPosition = mapStateHorizontal.get(linearHorizontalInferiorState);
            servoLinearHorizontal.setPosition(mapStateHorizontal.get(linearHorizontalInferiorState));


        });
    }
    public void monitor(Telemetry telemetry,String hozizontal) {
        if (monitor) {
            telemetry.addLine("*********************************");
            telemetry.addData("TELEMETRIA DO LINEAR HORIZONTAL ",hozizontal);
            telemetry.addLine("*********************************");
            telemetry.addData("-Posição do Servo: ",servoLinearHorizontal.getPosition());
            telemetry.addData("-PWM status: ",servoLinearHorizontal.getController().getPwmStatus());
            telemetry.addData("-Porta: ", servoLinearHorizontal.getPortNumber());
        }
    }
}