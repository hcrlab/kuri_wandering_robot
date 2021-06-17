#! /bin/bash
# usage:
#    First manually enter "hostname -I" to determine which ip address you want to use.
#    Then, run this script with two arguments. The first is the index of the ip address you want, and the second is "core" if you are configuring the roscore environment, and anything else otherwise.
ip_addresses=$(hostname -I)
# splitting them by space
ip_addresses=(${ip_addresses//" "/ })
selected_ip=${ip_addresses[$1]}
echo "Using IP address" $selected_ip
if [[ $2 == "core" ]]
then
    echo "Configuring Env for roscore"
    export ROS_MASTER_URI=http://localhost:11311
    export ROS_IP=$selected_ip
else
    echo "Configuring Env for ros nodes"
    export ROS_MASTER_URI=http://$selected_ip:11311
    export ROS_IP=$selected_ip
fi
unset ROS_HOSTNAME
