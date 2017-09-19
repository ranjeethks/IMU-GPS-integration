%Ranjeeth KS, University of Calgary

clear all;
close all;
clc;
novatel_gps_pos = load('BESTGPSPOS.mat');
novatel_gps_vel = load('BESTGPSVEL.mat');
ins_PVA = load('INSPVAS.mat');
raw_nova_imu = load('RAWIMUS.mat');
raw_xbow_imu=load('Xbow_Apr_22_11.mat');

length = 1000;
std_lat=std(+novatel_gps_pos.GP_Lat(1:length+1)-(ins_PVA.INS_Lat(133:133+length)))
std_long=std(+novatel_gps_pos.GP_Long(1:length+1)-(ins_PVA.INS_Long(133:133+length)))
std_alt=std(+novatel_gps_pos.GP_Alt(1:length+1)-(ins_PVA.INS_Alt(133:133+length)))

std_ve=std(+novatel_gps_vel.GV_ve(1:length+1)-(ins_PVA.INS_ve(133:133+length))')
std_vn=std(+novatel_gps_vel.GV_vn(1:length+1)-(ins_PVA.INS_vn(133:133+length))')
std_vu=std(+novatel_gps_vel.GV_vu(1:length+1)-(ins_PVA.INS_vu(133:133+length)))

