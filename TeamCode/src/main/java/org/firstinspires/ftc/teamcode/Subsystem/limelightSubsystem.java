package org.firstinspires.ftc.teamcode.Subsystem;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants;

public class limelightSubsystem extends SubsystemBase {

    Limelight3A limelight;
    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    private static final double camera_Height   = Constants.camera_Height; //meters
    private static final double target_Height   = Constants.target_Height; //meters
    private static final double camera_Angle    = Constants.camera_Angle; //degrees


    public limelightSubsystem(HardwareMap hardwareMap){
        this.limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.start();
    }
    public boolean hasTarget(){
        LLResult r = limelight.getLatestResult();
        return r != null && r.isValid();
    }
    public double getTx(){ //horizontal offset in degrees
        LLResult r = limelight.getLatestResult();
        return (r != null && r.isValid()) ? r.getTx() : -1.0; //-1 means no target
    }
    public double getDistance(){
        LLResult r = limelight.getLatestResult();
        if (r != null && r.isValid()){
            double ty = r.getTy();
            double angle = Math.toRadians(camera_Angle + ty);
            return (target_Height - camera_Height) / Math.tan(angle);
        } else {
            return -1; //NO TARGET
        }
    }

    @Override
    public void periodic() {
        TelemetryPacket llpacket = new TelemetryPacket();
        llpacket.put("Tx: ", limelight.getLatestResult().getTx());
        llpacket.put("Ty: ", limelight.getLatestResult().getTy());
        llpacket.put("Ta: ", limelight.getLatestResult().getTa());
        dashboard.sendTelemetryPacket(llpacket);
    }
}
