# Robot 2021

Public Release for Frog Force 503's 2021 robot: "AZack of the Clones". A Zack of the Clones' code is written in Java and is based off of WPILib's Java control system.

## Setup Instructions

### General
1. Clone this repo
1. Run `./gradlew` to download gradle and needed FRC/Vendor libraries
1. Run `./gradlew tasks` to see available options
1. Enjoy!

### Visual Studio Code (Official IDE)
1. Get the WPILib extension for easiest use from the VSCode Marketplace - Requires Java 11 or greater
1. In [`.vscode/settings.json`](.vscode/settings.json), set the User Setting, `java.home`, to the correct directory pointing to your JDK 11 directory

### IntelliJ
1. Run `./gradlew idea`
1. Open the `robot-2021.ipr` file with IntelliJ

### Eclipse
1. Run `./gradlew eclipse`
1. Open Eclipse and go to File > Open Projects from File System...
1. Set the import source to the `robot-2021` folder then click finish

### Basic Gradle Commands
* Run `./gradlew deploy` to deploy to the robot in Terminal (*nix) or Powershell (Windows)
* Run `./gradlew build` to build the code.  Use the `--info` flag for more details

## Code Highlights
	
## Variable Naming Conventions
- k*** (i.e. `kDriveWheelTrackWidthInches`): Final constants, especially those found in the [`Constants.java`](src/main/java/org/frogforce503/robot2021/Constants.java) file
- m*** (i.e. `mPathFollower`): Private instance variables
