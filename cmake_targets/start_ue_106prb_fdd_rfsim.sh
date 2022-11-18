#!/bin/bash
##
sudo cp DpssSeq2048.txt dpssSeq.txt

sudo RFSIMULATOR=192.168.1.124 ./ran_build/build/nr-uesoftmodem -r 106 --rfsim --numerology 1 -C 2169080000 --CO -400000000 -s 396 --sa --ue-fo-compensation --nokrnmod 1 -O ~/ue_oai.conf --ue-timing-correction-disable 0 --ffo-corr-shift 5
