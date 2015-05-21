Data contained in the HandData directory:

	JointAngles: *.mat files with 26D joint angle vectors
	ViconMarkers: *.csv files with 3D positions of 26 markers

Kinematic model for the 26-dimensional joint angle data (*.mat files):

	          Joint name           Local axis
	
	Value  1: "pose"               X             (global translation axis)
	Value  2: "pose"               Y             (global translation axis)
	Value  3: "pose"               Z             (global translation axis)
	
	Value  4: "pose"               X             (global rotation axis)
	Value  5: "pose"               Y             (global rotation axis)
	Value  6: "pose"               Z             (global rotation axis)
	
	Value  7: "LeftHandThumb1"     Z             (local rotation axis)
	Value  8: "LeftHandThumb1"     X             (local rotation axis)
	Value  9: "LeftHandThumb2"     X             (local rotation axis)
	Value 10: "LeftHandThumb3"     X             (local rotation axis)
	
	Value 11: "LeftHandIndex1"     Z             (local rotation axis)
	Value 12: "LeftHandIndex1"     X             (local rotation axis)
	Value 13: "LeftHandIndex2"     X             (local rotation axis)
	Value 14: "LeftHandIndex3"     X             (local rotation axis)
	
	Value 15: "LeftHandMiddle1"    Z             (local rotation axis)
	Value 16: "LeftHandMiddle1"    X             (local rotation axis)
	Value 17: "LeftHandMiddle2"    X             (local rotation axis)
	Value 18: "LeftHandMiddle3"    X             (local rotation axis)
	
	Value 19: "LeftHandRing1"      Z             (local rotation axis)
	Value 20: "LeftHandRing1"      X             (local rotation axis)
	Value 21: "LeftHandRing2"      X             (local rotation axis)
	Value 22: "LeftHandRing3"      X             (local rotation axis)
	
	Value 23: "LeftHandPinky1"     Z             (local rotation axis)
	Value 24: "LeftHandPinky1"     X             (local rotation axis)
	Value 25: "LeftHandPinky2"     X             (local rotation axis)
	Value 26: "LeftHandPinky3"     X             (local rotation axis)

Format of the marker position data (*.csv files):

	First a header with metadata, then in each line comma-separated values
	containing frame index and three positional values (X,Y,Z) for each marker

Types of tracked hand motions:

	General Movements: general hand movements that cover large parts of the
	hand's natural degrees of freedom

		Trial 01: flexion of all fingers simultaneously / closing hand
		Trial 02: abduction / splaying out the fingers
		Trial 03: touching the thumb with every finger
		Trial 04: flexion of the fingers (open -> closed)
		Trial 05: flexion of the fingers (closed -> open)
		Trial 07: miscellaneous movements and gestures
		Trial 08: miscellaneous movements and gestures

	Interaction: movements with object interactions -- grasping motions (power
	grasps, precision grasps), other interaction movements (handling of pens,
	jar lids, paper, smartphone)

		Trial 01: closing a big jar (large twist motion)
		Trial 02: closing a small jar (small twist motion)
		Trial 03: grasp movements (precision grasp und power grasp)
		Trial 04: grasp movements with a small object (power grasp)
		Trial 05: grasp movements with a small object (precision/power grasp)
		Trial 06: grasping and holding of a pen (precision grasp)
		Trial 07: interaction with a smartphone
		Trial 08: folding a piece of paper, flattening with one finger
		Trial 09: folding a piece of paper, flattening with all fingers

	Sign Language: various sybmols from the American Sign Language Alphabet

		Trial 02: N/A
		Trial 03: N/A
		Trial 04: N/A

