package org.firstinspires.ftc.teamcode.subsystems.vision;

import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.cos;
import static java.lang.Math.sin;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.lib.Levels;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvWebcam;

public class AutoAlign {
    private OpenCvCamera webcam;
    private SampleMecanumDrive drive;
    private StickObserverPipeline stickObserverPipeline = null;
    Pose2d polePos = new Pose2d(0,0,0);

    public AutoAlign(OpenCvCamera camera, SampleMecanumDrive drive) {
        webcam = camera;
        this.drive = drive;
    }

    public AutoAlign(SampleMecanumDrive drive) {
        this.drive = drive;
    }

    public void setCamera(OpenCvCamera camera) {
        this.webcam = camera;
    }

    public void observeStick(){
        stickObserverPipeline = new StickObserverPipeline();
        webcam.setPipeline(stickObserverPipeline);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                /*
                 * Tell the webcam to start streaming images to us! Note that you must make sure
                 * the resolution you specify is supported by the camera. If it is not, an exception
                 * will be thrown.
                 *
                 * Keep in mind that the SDK's UVC driver (what OpenCvWebcam uses under the hood) only
                 * supports streaming from the webcam in the uncompressed YUV image format. This means
                 * that the maximum resolution you can stream at and still get up to 30FPS is 480p (640x480).
                 * Streaming at e.g. 720p will limit you to up to 10FPS and so on and so forth.
                 *
                 * Also, we specify the rotation that the webcam is used in. This is so that the image
                 * from the camera sensor can be rotated such that it is always displayed with the image upright.
                 * For a front facing camera, rotation is defined assuming the user is looking at the screen.
                 * For a rear facing camera or a webcam, rotation is defined assuming the camera is facing
                 * away from the user.
                 */
                webcam.setPipeline(stickObserverPipeline);
//                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);

//                dashboard.startCameraStream(webcam, 10);

            }

            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });
    }

    public boolean lookingAtPole() {
        double[] coords = rotatedPolarCoord();
//        coords[1]+=5;
        coords[1] -=1;
        Pose2d pos = drive.getPoseEstimate();
        pos = new Pose2d(pos.getX(),pos.getY(),pos.getHeading()+coords[0]*PI/180+PI);
        polePos = new Pose2d(pos.getX()+cos(pos.getHeading())*coords[1]+sin(pos.getHeading()),pos.getY()+sin(pos.getHeading())*coords[1]+cos(pos.getHeading()),pos.getHeading());
        if (abs(coords[1])<5&&abs(coords[1])>0){
//            setDoneLookin(true);
        }
//        if(abs(pos.vec().distTo(roadrun.getCurrentTraj().end().vec()))<2){
//            setDoneLookin(true);
//        }

        if(abs(pos.vec().distTo(drive.getCurrentTraj().end().vec()))<10&&abs(coords[1])<18&&coords[1]>3&&(drive.getCurrentTraj()==null||abs(polePos.vec().distTo(drive.getCurrentTraj().end().vec()))<2.8)){
            return true;
        }
        return false;
    }

    public double centerOfPole(){
        return stickObserverPipeline.centerOfPole();
    }

    public double poleSize(){
        return stickObserverPipeline.poleSize();
    }

    public double[] rotatedPolarCoord(){
        return stickObserverPipeline.poleRotatedPolarCoord();
    }

    public Pose2d polePos() {
        return polePos;
    }

    public void updateTrajectory(Robot robot) {
        if (lookingAtPole()) {
            Pose2d target = polePos();
            TrajectorySequence trajectory = drive.getCurrentTraj();
            drive.changeTrajectorySequence(drive.trajectorySequenceBuilder(trajectory.start())
//                    .setReversed(true)
////                                .splineTo(target.vec(), target.getHeading())
//                    .splineTo(target.vec(), trajectory.end().getHeading()+PI)
                    .setReversed(true)
                    .splineTo(target.vec(), Math.toRadians(226))
                    .addTemporalMarker(1, ()->{
                        robot.highPreset(true);
                    })
                    .addTemporalMarker(1.7, ()->{
                        robot.v4b.runToPreset(Levels.AUTODEPOSIT);
                    })
                    .addTemporalMarker(1.7, ()->{
                        robot.autoDeposit(true);
                    })
                    .addTemporalMarker(1.9, ()->{
                        robot.slides.runToPosition(-220);
                    })
                    .waitSeconds(0.6)
                    .build());
        }
    }
}
