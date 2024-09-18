# Capture Project

## Running
Start Simulation
```
ros2 launch capture sim.launch.py
```

Start the real vehicle
```
ros2 launch capture mb690b.launch.py
```

## Installation

1. Before installing the Capture project you need to add the pegasus external workspace, according to the [Pegasus Installation Guide](https://pegasusresearch.github.io/pegasus/source/setup/installation.html).
```
mkdir -p ~/pegasus_external
cd ~/pegasus_external

# Install the dependencies for CasADi (IPOPT)
sudo apt install coinor-libipopt-dev

# Clone the repository (SSH)
git clone git@github.com:PegasusResearch/pegasus_external.git src

# Checkout to this version
cd src
git checkout 278f0f3
cd ..

# Compile the code
colcon build --symlink-install

# Add the source to your .bashrc
echo "source ~/pegasus_external/install/setup.bash" >> ~/.bashrc
```

2. The next step is to install the Capture project
```
# Create the workspace
mkdir -p ~/capture/src
cd ~/capture/src

# Clone the repository (SSH)
git clone git@github.com:rmssalgueiro/capture.git --recursive

# Go to the workspace
cd ~/capture

# Compile the code
colcon build --symlink-install

# Add the source to your .bashrc
echo "source ~/capture/install/setup.bash" >> ~/.bashrc
```

## Extra Installation steps running simulations

1. Install the dependencies (to be able to compile PX4-Autopilot):
```
# Linux packages
sudo apt install git make cmake python3-pip

# Python packages
pip install kconfiglib jinja2 empy jsonschema pyros-genmsg packaging toml numpy future

# GStreamer (for video streaming)
sudo apt-get install libgstreamer-plugins-base1.0-dev gstreamer1.0-plugins-bad gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-ugly -y
```

2. Clone the `PX4-Autopilot <https://github.com/PX4/PX4-Autopilot>`__:

```
# Option 1: With HTTPS
git clone https://github.com/PX4/PX4-Autopilot.git
# Option 2: With SSH (you need to setup a github account with ssh keys)
git clone git@github.com:PX4/PX4-Autopilot.git
```

3. Checkout to the stable version 1.14.3 and compile the code for software-in-the-loop (SITL) mode:

```
# Go to the PX4 directory
cd PX4-Autopilot

# Checkout to the latest stable release
git checkout v1.14.3

# Compile the code in SITL mode
make px4_sitl gazebo-classic
```

4. Add the following line to your .bashrc file:
```
echo "export PX4_DIR=$(pwd)" >> ~/.bashrc
```
Adding this line to the .bashrc file is important as the Pegasus Gazebo package will need to know the location of the PX4-Autopilot directory, and the launch files will use this environment variable to find the necessary files.
