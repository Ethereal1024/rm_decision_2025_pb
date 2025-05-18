#!/bin/bash
export DISPLAY=:0

echo "mechax" | sudo -S sudo chmod 777 /dev/ttyACM0
path=/home/mechax/rm_2025/rm_navigation/nav_2025_gicp/scripts
cd $path

cmds=(  
	"./serial.sh"
	"./nav.sh"
	"./test_alpha.sh"
	)

for cmd in "${cmds[@]}";
do
	echo Current CMD : "$cmd"
	gnome-terminal -- bash -ic "source ~/.bashrc;cd $path;source ../install/setup.bash;$cmd;"
	sleep 5.0
done

while [ 1 ]
do 
echo "- Commands Running -"
done
