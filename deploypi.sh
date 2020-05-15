HOST=${1:-10.1.10.205}

./gradlew jar && scp build/libs/Kumquat-Vision*.jar "$USER"@"$HOST":