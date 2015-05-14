

clear all;
close all;

load run1;
primalRun1 = primal;
load run2;
primalRun2 = primal;


xOptDiff  = abs(primalRun1.states - primalRun2.states);
omegaDiff = abs(primalRun1.controls - primalRun2.controls);
nodesDiff = abs(primalRun1.nodes - primalRun2.nodes);
figure;
    plot(primal.nodes, xOptDiff);
    title('Difference between States');
figure;
    plot(primal.nodes, omegaDiff);
    title('Difference between Controls');
figure;
    plot(primal.nodes, nodesDiff);
    title('Difference between Nodes');