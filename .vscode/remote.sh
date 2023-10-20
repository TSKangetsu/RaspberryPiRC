rm .vscode/connect-ok

$5 ssh $1@$2 "killall gdbserver"
echo "[$3-DEBUG] start scp EXE to Target ..."
$5 scp build/$3 $1@$2:/usr/bin
$5 $6
echo "[$3-DEBUG] lanuch $3 Directly ..."
touch .vscode/connect-ok
$5 ssh $1@$2 "/usr/bin/gdbserver :9590 /usr/bin/$3 $4"
