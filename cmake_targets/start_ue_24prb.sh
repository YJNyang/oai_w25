#!/bin/bash
##
sudo cp DpssSeq512.txt dpssSeq.txt

#sudo ./ran_build/build/nr-uesoftmodem -r 24 --numerology 1 -C 3604320000 -s 24 --sa --ue-fo-compensation --nokrnmod 1 -O ~/openairinterface_w25/openairinterface5g/targets/PROJECTS/GENERIC-NR-5GC/CONF/ue_oai.conf --usrp-args "addr=192.168.20.2,clock_source=internal,time_source=internal" --ue-txgain 35 --ue-rxgain 40


sudo ./ran_build/build/nr-uesoftmodem -r 24 --numerology 1 --band 78 -C 3604320000 -s 24 --sa --ue-fo-compensation --nokrnmod 1 -O ~/ue_oai.conf --usrp-args "addr=192.168.20.2,clock_source=internal,time_source=internal" --dlsch-parallel 8 --ue-timing-correction-disable 1 --rfsimu-if-enable 0 --ue-txgain 35 --ue-rxgain 45 --usrp-freq-off -1500 --doppler-shift 47329 --ffo-corr-shift 0 

# --usrp-freq-off用于手动校正UE usrp Tx/Rx Freq
# 信道模拟器测试时需要加上 --config-freq-satellite 1 --rfsimu-if-enable 1
# 选择性删去 --doppler-shift 0
# --ue-timing-correction-disable 1 关闭OAI定时跟踪,启动会影响修改方案运行,需要始终置1
# 在USRP直连 24PRB下,--ue-rxgain 45 --ffo-corr-shift 0即可
