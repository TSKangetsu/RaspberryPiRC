ssh $1@$2 "killall gdbserver"
echo "[RaspberryPiRC-DEBUG] start scp EXE to Target ..."
scp build/RaspberryPiRC $1@$2:/usr/bin
scp APMconfig.json $1@$2:/etc
echo "[RaspberryPiRC-DEBUG] lanuch RaspberryPiRC Directly ..."
ssh $1@$2 "/usr/bin/gdbserver :9590 /usr/bin/RaspberryPiRC $3"