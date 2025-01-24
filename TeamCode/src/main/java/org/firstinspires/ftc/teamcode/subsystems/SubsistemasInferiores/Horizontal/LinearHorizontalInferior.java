package org.firstinspires.ftc.teamcode.subsystems.SubsistemasInferiores.Horizontal;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.HardwareNames;
import org.firstinspires.ftc.teamcode.subsystems.common.Horizontal.LinearHorizontalStates;
import org.firstinspires.ftc.teamcode.subsystems.common.Horizontal.LinearHorizontalV4;

import java.util.HashMap;

public class LinearHorizontalInferior extends LinearHorizontalV4 {

    LinearHorizontalStates linearHorizontalState  = LinearHorizontalStates.RETRACTED;
    HashMap<LinearHorizontalStates, Double> mapStateHorizontal = new HashMap<>();


    public LinearHorizontalInferior(HardwareMap hardwareMap) {
        super(hardwareMap, HardwareNames.horizontalInferiorServo);
        mapStateHorizontal.put(LinearHorizontalStates.EXTENDED, 1.0);
        mapStateHorizontal.put(LinearHorizontalStates.INTERMEDIATE,0.785);
        mapStateHorizontal.put(LinearHorizontalStates.RETRACTED,0.5);

    }

    public Action goToExtended(){
        linearHorizontalState = LinearHorizontalStates.EXTENDED;

        return new InstantAction(() -> {
           servoLinearHorizontal.setPosition(mapStateHorizontal.get(linearHorizontalState));

        });
    }
    public Action goToRetracted(){
        linearHorizontalState = LinearHorizontalStates.RETRACTED;

        return new InstantAction(() -> {
            servoLinearHorizontal.setPosition(mapStateHorizontal.get(linearHorizontalState));

        });
    }

    public void monitor(Telemetry telemetry){
        super.monitor(telemetry,"Inferior");
    }
}
