#!/bin/sh

sudo ./RadiationDetectorLinux --version
sleep 1
sudo ./RadiationDetectorLinux --help
sleep 1
sudo ./RadiationDetectorLinux --reset
sleep 1
sudo ./RadiationDetectorLinux --g_eng_th
sleep 1
sudo ./RadiationDetectorLinux --s_eng_th 30
sleep 1
sudo ./RadiationDetectorLinux --g_rd_up
sleep 1
sudo ./RadiationDetectorLinux --s_rd_up 1250
sleep 1
sudo ./RadiationDetectorLinux --g_rd_lo
sleep 1
sudo ./RadiationDetectorLinux --s_rd_lo 30
sleep 1
sudo ./RadiationDetectorLinux --g_data_and_temp
sleep 1
sudo ./RadiationDetectorLinux --g_int_temp
sleep 1
sudo ./RadiationDetectorLinux --r_eeprom 0A
sleep 1
sudo ./RadiationDetectorLinux --r_eeprom 0C
sleep 1
sudo ./RadiationDetectorLinux --r_eeprom 0E
sleep 1
sudo ./RadiationDetectorLinux --r_eeprom 10
sleep 1
sudo ./RadiationDetectorLinux --g_sievert 1 20
