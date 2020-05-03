HOST=${1:-10.1.10.205}

ssh ${HOST} -- pkill java
ssh ${HOST} -- java -cp Kumquat-Vision-1.0-SNAPSHOT.jar com.palyrobotics.LidarServer