%X = [0.967242 1.90476 1.93548 1.96721 2 2.7907 2.85714 4.13793 4.28571  5.71429 6 6 6.31579 6.66667 7.05882 7.5 8 8.57143 8.57143 9.23077 9.23077 10 10 10.9091 10.9091 12 13.3333 15 15 15 17.1429 17.1429  17.1429 20 20 24 ];
%Y = [1 2 2 2 2 3 3 4 4 5 5 6 6 6 7 7 7 7 8 8 9 10 9 9 10 10 11 11 12 13 12 13 14 13 14 15];
X =  [440 325 260 215 185 161];
Y = [50    80   110   140   170   200];

n = size(X,2);
k = size(Y,2);

plot(X,Y,'bo')


[p P] = MMQ(X,Y,2);

p
%C = P(1).*ones(1,36) +P(2).*X + P(3).*X.*X + P(4).*X.*X.*X;
C = P(1).*ones(1,33) +P(2).*X + P(3).*X.*X;
%C = P(1).*ones(1,33) +P(2).*X;

hold
plot(X,C)

%X = 0.5
%C = P(1) +P(2).*X + P(3).*X.*X + P(4).*X.*X.*X

