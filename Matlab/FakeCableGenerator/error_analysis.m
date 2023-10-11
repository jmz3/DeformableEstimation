clear;
clc;

testx = [ 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25 26 27 28 29 30];
testy = [ inf 300 294 321 209 110 99 67 40 42 20 23 19 22 11 9 11 7 8 7 6 9 8 8 8 9 5 4 3 3];
plot(testx, testy)

title('Average Error on Interpolated Points')
xlabel('Total Number of Markers')
ylabel('Average Errors /mm')
