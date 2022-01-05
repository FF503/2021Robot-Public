package org.frogforce503.robot2021.tests.modes;

import org.frogforce503.robot2021.subsystems.swerve.Swerve;
import org.frogforce503.robot2021.tests.RobotTest;

public class AlignWheels extends RobotTest {
    // int[] positions;
    // int count = 0;

    public AlignWheels() {
        super();
        // this.setSubsystem(Swerve.getInstance());
        // backRight, backLeft, frontLeft, frontRight
        this.setName("align_wheels");
        // positions = new int[] { getOffset("BackRight"), getOffset("BackLeft"),
        // getOffset("FrontLeft"),
        // getOffset("FrontRight"), };
    }

    // public int getOffset(String position) {
    // JSONParser helper = new JSONParser();
    // JSONObject object;

    // try {
    // object = (JSONObject) helper.parse(new FileReader(
    // Filesystem.getDeployDirectory().getAbsolutePath() + "/SwerveModules/Comp" +
    // position + ".json"));
    // } catch (IOException | ParseException e) {
    // e.printStackTrace();
    // return 0;
    // }

    // return ((Long) object.get("startingEncoderClick")).intValue();
    // }

    @Override
    public void run() {
        // Swerve.getInstance().getModules().forEach((mod) -> {
        // // mod.drive(503.0, SwerveModule.encoderToAngle(positions[count]));
        // // count++;
        // mod.drive(503.0, SwerveModule.encoderToAngle(mod.getBaseEncoderClicks()));
        // });
        Swerve.getInstance().snapForward();
        this.writeOutput("You are currrently running All Forward test");
    }

    @Override
    public void stop() {
    }
}
