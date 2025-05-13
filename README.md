# ProbRob-Project: Planar Monocular SLAM

This project implements a full SLAM pipeline using only a monocular camera and odometry in a planar environment (SE(2) motion embedded in SE(3)). It includes:

- Landmark triangulation from 2D image correspondences
- Pose graph and projection-based Bundle Adjustment (Total Least Squares)
- Evaluation against ground truth for both trajectory and map

## Requirements

- Octave (tested on version ≥ 6)
- Standard packages (no toolboxes required)

## Running the Code

Run the main entry point:
```bash
octave main.m
```

## Note
This is an attempt at implementing Planar Monocular SLAM from scratch which is currently **not successful**:

Current problems:
- Triangulated landmarks suffer from high reprojection error.
- Bundle Adjustment (BA) diverges and unstable.
- The trajectory estimate does not meaningfully improve over raw odometry.

