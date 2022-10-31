sudo cp udev_rules/*.rules /etc/udev/rules.d
sudo udevadm control --reload-rules && sudo systemctl restart systemd-udevd && sudo udevadm trigger
