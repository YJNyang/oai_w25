#!/bin/bash
##
sudo cp DpssSeq2048.txt dpssSeq.txt


sudo ./ran_build/build/nr-uesoftmodem -r 106 --numerology 1 --band 78 -C 3619200000 -s 516 --sa --ue-fo-compensation --nokrnmod 1 -O ~/ue_oai.conf --usrp-args "addr=192.168.20.2,clock_source=internal,time_source=internal"   --ue-txgain 35 --ue-rxgain 65 --ue-timing-correction-disable 1  --dlsch-parallel 16 --rfsimu-if-enable 0 --usrp-freq-off -1550 --doppler-shift -46543 

# --usrp-freq-off用于手动校正UE usrp Tx/Rx Freq
# 信道模拟器测试时需要加上 --config-freq-satellite 1 --rfsimu-if-enable 1
# 选择性删去 --doppler-shift 0
# --ue-timing-correction-disable 1 关闭OAI定时跟踪,启动会影响修改方案运行,需要始终置1

# 信道模拟器测试时需要加上 --rfsimu-if-enable 1
# 选择性删去 --doppler-shift 0

# --ue-timing-correction-disable 1 需要始终存在

