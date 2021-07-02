#!/bin/sh

tmux new-session -d -s demo
tmux rename-window 'Demo'
tmux select-window -t demo:0
tmux send-keys 'source ~/workspaces/nick_ws/devel/setup.bash && source ~/env.sh && roslaunch kuri_launch kuri.launch use_navigation:=false use_transcription:=false use_vision:=false' Enter
tmux split-window -h -t 1
tmux send-keys 'sudo docker run --rm --network host -v /home/mayfield/workspaces/nick_melodic_ws:/workspace -v /home/mayfield/.aws:/root/.aws -v /mayfield:/mayfield -it demo' Enter
tmux -2 attach-session -t demo