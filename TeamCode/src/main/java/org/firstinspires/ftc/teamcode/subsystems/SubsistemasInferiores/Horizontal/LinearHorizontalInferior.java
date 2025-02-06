package org.firstinspires.ftc.teamcode.subsystems.SubsistemasInferiores.Horizontal;

import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.HardwareNames;
import org.firstinspires.ftc.teamcode.subsystems.common.Horizontal.LinearHorizontalStates;
import org.firstinspires.ftc.teamcode.subsystems.common.Horizontal.LinearHorizontal;

public class LinearHorizontalInferior extends LinearHorizontal {

    public LinearHorizontalInferior(HardwareMap hardwareMap) {
        super(hardwareMap, HardwareNames.horizontalInferiorServo);
        mapStateHorizontal.put(LinearHorizontalStates.EXTENDED, 0.627);
        mapStateHorizontal.put(LinearHorizontalStates.INTERMEDIATE,0.371);
        mapStateHorizontal.put(LinearHorizontalStates.RETRACTED,0.0);

    }

    public Action goToExtended() {return super.goToExtended();}
    public Action goToRetracted() {
        return super.goToRetracted();
    }

    public void monitor(Telemetry telemetry){
        super.monitor(telemetry,"Inferior");
    }
}
