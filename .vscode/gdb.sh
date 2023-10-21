#!/bin/bash
sleep 1

while ! [ -f $3/.vscode/connect-ok ]; do
    sleep 0.2
done

sleep 2

proxychains -f /etc/proxychains4_dev1.conf -q $2 "$@"
