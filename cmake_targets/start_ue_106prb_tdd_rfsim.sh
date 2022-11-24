#!/bin/bash
##
sudo cp DpssSeq2048.txt dpssSeq.txt
sudo RFSIMULATOR=192.168.1.124 ./ran_build/build/nr-uesoftmodem -r 106 --rfsim --numerology 1 --band 78 -C 3619200000 -s 516 --sa --ue-fo-compensation --nokrnmod 1 -O ~/ue_oai.conf --ue-timing-correction-disable 0 --ffo-corr-shift 5
