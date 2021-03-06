function mathematical_program_test

% Simple example of calling MathematicalProgram through the Python bindings.

% Setup PYTHONPATH for this MATLAB session. Move this to addpath_drake() if we
% expand our use of the Python bindings.
P = py.sys.path;
drake_python_path = fullfile(fileparts(getDrakePath), 'build', 'install', 'lib', 'python2.7', 'dist-packages');
if count(P, drake_python_path) == 0
    insert(P, int32(0), drake_python_path);
end

% QP test
prog = py.pydrake.solvers.mathematicalprogram.MathematicalProgram();
x = prog.NewContinuousVariables(int32(2), 'x');
prog.AddLinearConstraint(x.item(int32(0)) >= 1.0);
prog.AddLinearConstraint(x.item(int32(1)) >= 1.0);
prog.AddQuadraticCost(py.numpy.eye(int32(2)), py.numpy.zeros(int32(2)), x);
result = prog.Solve()

% Note: int32(0) is kSolutionFound; can't reference it directly.
assert(result == py.pydrake.solvers.mathematicalprogram.SolutionResult(int32(0)));

x_sol = prog.GetSolution(x);
assert(abs(x_sol.item(int32(0)) - 1.0) < 1e-6);
assert(abs(x_sol.item(int32(1)) - 1.0) < 1e-6);
