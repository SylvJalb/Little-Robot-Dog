This projet is a simple 3d printed robot dog.     
I got the 3D design from [thingiverse](https://www.thingiverse.com/thing:73530).     
After print and make it, I developed this code, which for now only allows the robot to do push-ups.

#  Requirement
## Hardware
- raspberry pi 3 +
- 3D printer

## Installs (on raspberry pi)
[WiringPi](https://github.com/WiringPi/WiringPi)   
```bash
sudo apt-get install wiringpi
sudo apt-get install -y python-smbus
sudo apt-get install -y i2c-tools
```

[PCA9685](https://github.com/Reinbert/pca9685)
```bash
git clone https://github.com/Reinbert/pca9685.git
cd ./pca9685/src
sudo make install
```

## Enable I2C interface 
on raspberry pi
```bash
sudo raspi-config
```

#  Check Requirement
```bash
gpio -v
```

# INSTALL
```bash
git clone https://github.com/SylvJalb/Little-Robot-Dog.git
cd ./Little-Robot-Dog
make
```
Put the "robotdog" file in the raspberry pi and execute.    
