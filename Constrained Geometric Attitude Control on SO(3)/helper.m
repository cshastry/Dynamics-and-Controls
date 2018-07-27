%% Symbolic code for helper function generation

% hat operator

vector = sym('vector',[3 1]);

skew_matrix = [0 -vector(3) vector(2);vector(3) 0 -vector(1);-vector(2) vector(1) 0];

matlabFunction(skew_matrix,'File','hat','Vars',{vector},'Outputs',{'skew_matrix'});

% inverse hat operator
skew_matrix = sym('skew_matrix',[3 3]);

vector = [-skew_matrix(2,3) skew_matrix(1,3) -skew_matrix(1,2)].';

matlabFunction(vector,'File','inv_hat','Vars',{skew_matrix},'Outputs',{'vector'});

