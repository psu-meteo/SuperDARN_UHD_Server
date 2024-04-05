#!/bin/bash

NAME=$1
PRIORITY=$2

pid_list=`/bin/ps -ef | grep $NAME | grep -v grep | grep -v kill_by_name | awk '{print $2}'`

for pid in $pid_list;
do 
    echo $pid
    /bin/ps -p $pid
    chrt -p $PRIORITY $pid
done

