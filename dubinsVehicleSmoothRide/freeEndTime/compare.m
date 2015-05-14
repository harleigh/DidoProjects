
clear all;
close all;

load v_01;
primalRun1 = primal;

load v_02;
primalRun2 = primal;

error = abs(primalRun1.states - primalRun2.states);
