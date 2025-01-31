package org.firstinspires.ftc.teamcode.subsystems.SubsistemasInferiores.Horizontal;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.HardwareNames;
import org.firstinspires.ftc.teamcode.subsystems.common.Horizontal.LinearHorizontalStates;
import org.firstinspires.ftc.teamcode.subsystems.common.Horizontal.LinearHorizontalV4;

public class LinearHorizontalInferior extends LinearHorizontalV4 {

    public LinearHorizontalInferior(HardwareMap hardwareMap) {
        super(hardwareMap, HardwareNames.horizontalInferiorServo);
        mapStateHorizontal.put(LinearHorizontalStates.EXTENDED, 1.0);
        mapStateHorizontal.put(LinearHorizontalStates.INTERMEDIATE,0.785);
        mapStateHorizontal.put(LinearHorizontalStates.RETRACTED,0.0);

    }

    public Action goToExtended() {
        return this.goToExtended();
    }
    public Action goToRetracted() {
        return this.goToRetracted();
    }

    public void monitor(Telemetry telemetry){
        super.monitor(telemetry,"Inferior");
    }
}
