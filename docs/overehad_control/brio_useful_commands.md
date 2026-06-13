# Brio quick commands

# Go to project folder
cd ~/Master/Test

# Show camera device list
v4l2-ctl --list-devices

# Show current camera controls
v4l2-ctl -d /dev/video2 --list-ctrls

# Show current resolution / format
v4l2-ctl -d /dev/video2 --get-fmt-video

# Show current FPS
v4l2-ctl -d /dev/video2 --get-parm

# Show all important camera info
v4l2-ctl -d /dev/video2 --list-ctrls
v4l2-ctl -d /dev/video2 --get-fmt-video
v4l2-ctl -d /dev/video2 --get-parm

# Save current camera settings to text file
v4l2-ctl -d /dev/video2 --list-ctrls > camera_settings_saved.txt
v4l2-ctl -d /dev/video2 --get-fmt-video >> camera_settings_saved.txt
v4l2-ctl -d /dev/video2 --get-parm >> camera_settings_saved.txt

# Make camera setup script executable
chmod +x brio_manual_stable_4k.sh

# Run camera setup script
./src/overhead_control/camera_settings/brio_4k_manual.sh
./src/overhead_control/camera_settings/brio_4k_auto.sh

# Run main homography file
python3 homography_live_linux.py

# Normal startup
cd ~/Master/Test
./brio_manual_stable_4k.sh
python3 homography_live_linux.py

# Check if camera is busy
fuser /dev/video2

# Stop running program
Ctrl + C


# Main file keyboard controls inside homography_live_linux.py

q = quit
c = recalibrate homography
s = save homography
l = load homography
p = save raw image
o = save overlay image
g = toggle grid
b = toggle border
j = print detected Jenga positions


# Useful files

homography_live_linux.py       main live mapping file
brio_manual_stable_4k.sh       camera setup script
arena_homography.json          saved homography
snapshots/                     saved raw and overlay images

Changable settings 

# auto_exposure: min=0 max=3 default=3 current=1
# 1 = Manual Mode
v4l2-ctl -d /dev/video2 -c auto_exposure=1

# exposure_time_absolute: min=3 max=2047 default=250 current=130
v4l2-ctl -d /dev/video2 -c exposure_time_absolute=130

# exposure_dynamic_framerate: min=0 max=1 default=0 current=0
v4l2-ctl -d /dev/video2 -c exposure_dynamic_framerate=0

# brightness: min=0 max=255 default=128 current=80
v4l2-ctl -d /dev/video2 -c brightness=80

# contrast: min=0 max=255 default=128 current=125
v4l2-ctl -d /dev/video2 -c contrast=125

# saturation: min=0 max=255 default=128 current=120
v4l2-ctl -d /dev/video2 -c saturation=120

# white_balance_automatic: min=0 max=1 default=1 current=0
v4l2-ctl -d /dev/video2 -c white_balance_automatic=0

# white_balance_temperature: min=2000 max=7500 default=4000 current=2800
v4l2-ctl -d /dev/video2 -c white_balance_temperature=2800

# gain: min=0 max=255 default=0 current=0
v4l2-ctl -d /dev/video2 -c gain=0

# sharpness: min=0 max=255 default=128 current=200
v4l2-ctl -d /dev/video2 -c sharpness=200

# power_line_frequency: min=0 max=2 default=2 current=1
# 1 = 50 Hz
v4l2-ctl -d /dev/video2 -c power_line_frequency=1

# backlight_compensation: min=0 max=1 default=1 current=1
v4l2-ctl -d /dev/video2 -c backlight_compensation=1

# focus_automatic_continuous: min=0 max=1 default=1 current=0
v4l2-ctl -d /dev/video2 -c focus_automatic_continuous=0

# focus_absolute: min=0 max=255 default=0 current=5
v4l2-ctl -d /dev/video2 -c focus_absolute=5

# zoom_absolute: min=100 max=500 default=100 current=100
v4l2-ctl -d /dev/video2 -c zoom_absolute=100

# pan_absolute: min=-36000 max=36000 default=0 current=0
v4l2-ctl -d /dev/video2 -c pan_absolute=0

# tilt_absolute: min=-36000 max=36000 default=0 current=0
v4l2-ctl -d /dev/video2 -c tilt_absolute=0