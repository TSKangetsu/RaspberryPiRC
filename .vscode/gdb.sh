#!/bin/bash
sleep 1

while ! [ -f /home/user/build/RPiSingleAPM/.vscode/connect-ok ]; do
    sleep 0.2
done

sleep 2

proxychains -q $2 "$@"
