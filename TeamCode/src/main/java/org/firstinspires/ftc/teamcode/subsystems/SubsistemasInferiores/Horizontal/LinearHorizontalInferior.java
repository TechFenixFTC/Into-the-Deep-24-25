package org.firstinspires.ftc.teamcode.subsystems.SubsistemasInferiores.Horizontal;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.HardwareNames;
import org.firstinspires.ftc.teamcode.subsystems.common.Horizontal.LinearHorizontalStates;
import org.firstinspires.ftc.teamcode.subsystems.common.Horizontal.LinearHorizontal;
@Config
public class LinearHorizontalInferior extends LinearHorizontal {



    public LinearHorizontalInferior(HardwareMap hardwareMap) {
        super(hardwareMap, HardwareNames.horizontalInferiorMotor);


    }


    public Action goToExtended() {return super.goToExtended();}
    public Action goToRetracted() {

        return super.goToRetracted();
    }



}
