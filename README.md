#  Visual_Odometry_Custom

Visual odometry on DROP Sphere AUV.

## Depedencies

 * opencv

## Installation

1. Make sure you have opencv installed.

2. Clone the repository:

	git clone git@bitbucket.org:droplabumich/visual-odometry-custom.git

3. Create an empty `/build` directory (replace current one if exists)
		
        mkdir build

4. Makefile

		cd build
		cmake ..
		make

## Run Odometry
```
		./bin/visualOdometry
```



