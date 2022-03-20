# camera-solver
Calculation of ground truth correspondences and essential matrix for two images of the same object captured from different angles using the 8 point algorithm and triangulation approach. 
# Problem
Consider two images I1 (“im1.jpg”) and I2 (“im2.jpg”) of a static scene captured from a single
camera with the given intrinsic camera matrix K (“Intrinsic Matrix K.txt”).
Assume that the world-coordinate system is aligned with the coordinate-system of
the camera location.
● Find a set of ground-truth correspondences {(pi, p0i)} n i=1 using any of the existing
implementations. Ensure that there are at least n = 100 true correspondences.
● Implement the algorithm taught in the class to find the Essential matrix E.
● Decompose the obtained Essential matrix E into the camera motion rotation matrix R and the translation vector t.
● Let Pi be the corresponding 3D point for the pixel pair (pi, p0i). Find Pi ∀i ∈ {1, 2, . . ., n}
using the triangulation approach learned in the class.
● Plot the obtained Pi, ∀i ∈ {1, 2, . . . , n} and the camera center t.

# System requirements
1. MATLAB R2020b (verified) (you can try for other versions as well)
2. Computer Vision Toolbox (Add-on)

# Usage

Run the main.m file in your MATLAB environment and you are good to go!!
