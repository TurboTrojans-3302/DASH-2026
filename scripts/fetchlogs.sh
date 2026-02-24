#!/bin/sh
while true; do 
    hostlist='10.33.02.2 172.22.11.2 roborio-3302-FRC.local'
    for host in $hostlist; do
        #rsync -av lvuser@$host:/home/lvuser/logs/ :/u/logs ~/Documents/3302/logfiles/ 
        sshpass -p"" rsync -av lvuser@$host:/home/lvuser/logs/ :/u/logs ~/Documents/3302/logfiles/ 
    done
    sleep 60 
done