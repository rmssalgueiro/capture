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
git checkout bdef110
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