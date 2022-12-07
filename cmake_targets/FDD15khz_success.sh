sudo ./ran_build/build/nr-uesoftmodem  -r 25 --band 66 --numerology 0 -C 2152250000 --CO -400000000 -s 48 --sa --nokrnmod 1 -O ~/ue_oai.conf --dlsch-parallel 8 --usrp-args "addr=192.168.20.2,clock_source=internal,time_source=internal"  --ue-txgain 35 --ue-rxgain 50 --ue-fo-compensation --ue-timing-correction-disable 1 --usrp-freq-off -900 --doppler-shift 54321 --ffo-corr-shift 0

