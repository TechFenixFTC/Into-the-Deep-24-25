package org.firstinspires.ftc.teamcode.subsystems.Sensors;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.SensorDistanceEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
@Config
public class DistanceSensor {

    public SensorDistanceEx sensorDistanceRight;
    public SensorDistanceEx sensorDistanceLeft;
    double distance;


    public double getDistance(){this.distance = sensorDistanceRight.getDistance(DistanceUnit.CM); return this.distance;}
    public DistanceSensor(HardwareMap hardwareMap) {
      sensorDistanceLeft = hardwareMap.get(SensorDistanceEx.class, "sensorDistanceLeft");
      sensorDistanceRight = hardwareMap.get(SensorDistanceEx.class,"sensorDistanceRight");
    }
}
