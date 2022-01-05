package org.frogforce503.lib.motion;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import org.frogforce503.lib.util.Util;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;

public final class HolonomicTrajectoryUtil {

    private HolonomicTrajectoryUtil() {
        throw new UnsupportedOperationException("This is a utility class!");
    }

    /**
     * Imports a Trajectory from a PathPlanner-style CSV file.
     *
     * @param filename the name of the csv file to import from
     * @return The trajectory represented by the file.
     * @throws IOException if reading from the file fails
     */
    public static HolonomicTrajectory fromPathPlannerCsv(String filename) throws IOException {

        List<HolonomicTrajectory.State> states = new ArrayList<>();

        try (BufferedReader br = new BufferedReader(
                new FileReader(new File(Filesystem.getDeployDirectory(), "paths/" + filename + ".csv")))) {
            String line;
            while ((line = br.readLine()) != null) {
                String[] point = line.split(",");

                double poseX = Double.parseDouble(point[0]);
                double poseY = Double.parseDouble(point[1]);
                double vel = Double.parseDouble(point[2]);
                double heading = Double.parseDouble(point[3]);
                double rotation = Double.parseDouble(point[4]);

                states.add(new HolonomicTrajectory.State((states.size() + 1) * 0.005, vel * 12.0, 0.0,
                        new Pose2d(poseX * -12.0, poseY * 12.0,
                                Rotation2d.fromDegrees(Util.boundAngle0to360Degrees(180 - heading))),
                        0.0, Util.boundAngle0to360Degrees(180 + rotation)));
            }
        } catch (Exception e) {
            e.printStackTrace();
        }
        return new HolonomicTrajectory(states);
    }

}
