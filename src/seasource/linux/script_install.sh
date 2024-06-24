#!/usr/bin/zsh
sudo cp ~/seasource-ros/install/seasource/share/seasource/linux/seasource.service /etc/systemd/system/
cp -r ~/seasource-ros/install/seasource/share/seasource/config ~/

sudo systemctl daemon-reload
sudo systemctl enable seasource.service
