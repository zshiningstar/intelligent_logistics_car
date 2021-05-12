#!/bin/sh

rtkpath=$(cd `rospack find driverless`/../rtcm3.2; pwd)
echo ${rtkpath}

cd ${rtkpath}

./main


