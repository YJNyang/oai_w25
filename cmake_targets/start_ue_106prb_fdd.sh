#!/bin/bash
##
sudo cp DpssSeq2048.txt dpssSeq.txt


sudo ./ran_build/build/nr-uesoftmodem -r 106 --rfsim --numerology 1 -C 2169080000 --CO -400000000 -s 396 --sa --ue-fo-compensation --nokrnmod 1 -O ~/ue_oai.conf --ue-txgain 35 --ue-rxgain 65 --usrp-args "addr=192.168.20.2,clock_source=internal,time_source=internal" --ue-timing-correction-disable 0  --dlsch-parallel 16 --rfsimu-if-enable 0 --usrp-freq-off -1550 --doppler-shift 0 --ffo-corr-shift 10

# --usrp-freq-off用于手动校正UE usrp Tx/Rx Freq
# 信道模拟器测试时需要加上 --config-freq-satellite 1 --rfsimu-if-enable 1
# 选择性删去 --doppler-shift 0
# --ue-timing-correction-disable 1 关闭OAI定时跟踪,启动会影响修改方案运行,需要始终置1

# 信道模拟器测试时需要加上 --rfsimu-if-enable 1
# 选择性删去 --doppler-shift 0

# --ue-timing-correction-disable 1 需要始终存在

